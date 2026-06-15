// ESP32-CAM — Return-to-Start Test
// On boot: runs a short wander (forward → random rotate → forward),
// then dead-reckoning computes the return path and drives back to start.
// No camera, no WiFi, no Android needed for this test.

#include <Arduino.h>
#include <math.h>

// Serial to Arduino UNO (P8 on expansion board)
#define RXD2 16
#define TXD2 17
HardwareSerial ArduinoSerial(2);

// ── Dead-reckoning constants — TUNE THESE after first test ──────────────────
// Measure: how far does robot travel in 1 second? Enter in mm.
// Measure: how many degrees does it rotate in 1 second?
const float FORWARD_SPEED_MM_S = 200.0;
const float ROTATE_SPEED_DEG_S = 90.0;
const int   ACK_TIMEOUT_MS     = 15000;  // max wait for Arduino ACK

// ── Odometry state (relative to start = 0,0,0) ──────────────────────────────
float rel_x     = 0.0;
float rel_y     = 0.0;
float rel_angle = 0.0;   // degrees, 0 = direction robot faced at start

// ── Helpers ─────────────────────────────────────────────────────────────────
float normalise180(float a) {
  a = fmod(a + 180.0, 360.0);
  if (a < 0) a += 360.0;
  return a - 180.0;
}

void waitForACK() {
  unsigned long t = millis();
  while (millis() - t < ACK_TIMEOUT_MS) {
    if (ArduinoSerial.available()) {
      String resp = ArduinoSerial.readStringUntil('\n');
      resp.trim();
      if (resp == "ACK") return;
    }
  }
  // Timed out — continue anyway
}

// Send command, wait for ACK, then update dead-reckoning
void runCommand(const char* cmd, int duration_ms) {
  char buf[32];
  if (duration_ms > 0)
    snprintf(buf, sizeof(buf), "%s,%d", cmd, duration_ms);
  else
    snprintf(buf, sizeof(buf), "%s", cmd);

  Serial.printf("[ESP32] sending: %s\n", buf);
  ArduinoSerial.println(buf);
  waitForACK();

  // Update odometry
  float dur_s = duration_ms / 1000.0;
  float rad   = rel_angle * PI / 180.0;

  if (strcmp(cmd, "F") == 0) {
    rel_x += FORWARD_SPEED_MM_S * dur_s * cos(rad);
    rel_y += FORWARD_SPEED_MM_S * dur_s * sin(rad);
  } else if (strcmp(cmd, "B") == 0) {
    rel_x -= FORWARD_SPEED_MM_S * dur_s * cos(rad);
    rel_y -= FORWARD_SPEED_MM_S * dur_s * sin(rad);
  } else if (strcmp(cmd, "R") == 0) {
    rel_angle = fmod(rel_angle + ROTATE_SPEED_DEG_S * dur_s, 360.0);
  } else if (strcmp(cmd, "L") == 0) {
    rel_angle = fmod(rel_angle - ROTATE_SPEED_DEG_S * dur_s + 360.0, 360.0);
  }

  Serial.printf("[Odom] x=%.0f y=%.0f angle=%.1f\n", rel_x, rel_y, rel_angle);
}

// ── Test sequence ────────────────────────────────────────────────────────────
void wander() {
  Serial.println("\n=== WANDER PHASE ===");

  // Step 1: forward 1–2 seconds
  int fwd1 = random(1000, 2001);
  Serial.printf("Forward %d ms\n", fwd1);
  runCommand("F", fwd1);
  delay(300);

  // Step 2: random rotation -90 to +90 degrees
  float deg = random(-90, 91);   // degrees to rotate
  int rot_ms = (int)(fabs(deg) / ROTATE_SPEED_DEG_S * 1000);
  const char* rot_cmd = (deg >= 0) ? "R" : "L";
  Serial.printf("Rotate %s %.0f deg (%d ms)\n", rot_cmd, fabs(deg), rot_ms);
  if (rot_ms > 0) {
    runCommand(rot_cmd, rot_ms);
  }
  delay(300);

  // Step 3: forward 1–2 seconds
  int fwd2 = random(1000, 2001);
  Serial.printf("Forward %d ms\n", fwd2);
  runCommand("F", fwd2);
  delay(300);

  float dist = sqrt(rel_x * rel_x + rel_y * rel_y);
  Serial.printf("\nWander done. Pos=(%.0f, %.0f) angle=%.1f deg | dist from start=%.0f mm\n",
                rel_x, rel_y, rel_angle, dist);
}

// ── Return-to-start ──────────────────────────────────────────────────────────
void returnToStart() {
  Serial.println("\n=== RETURN PHASE ===");

  float dist = sqrt(rel_x * rel_x + rel_y * rel_y);
  if (dist < 10.0) {
    Serial.println("Already at start.");
    return;
  }

  // 1. Compute heading to start in relative frame
  float heading_to_start = atan2(-rel_y, -rel_x) * 180.0 / PI;
  if (heading_to_start < 0) heading_to_start += 360.0;

  // 2. Bearing = how much to rotate from current angle
  float bearing = normalise180(heading_to_start - rel_angle);
  Serial.printf("Bearing to start: %.1f deg\n", bearing);

  // 3. Rotate to face start
  if (fabs(bearing) > 5.0) {
    int rot_ms = (int)(fabs(bearing) / ROTATE_SPEED_DEG_S * 1000);
    const char* rot_cmd = (bearing > 0) ? "R" : "L";
    Serial.printf("Aligning: %s %.1f deg (%d ms)\n", rot_cmd, fabs(bearing), rot_ms);
    runCommand(rot_cmd, rot_ms);
    delay(200);
  }

  // 4. Drive back the estimated distance
  int drive_ms = (int)(dist / FORWARD_SPEED_MM_S * 1000);
  Serial.printf("Driving home: %.0f mm (%d ms)\n", dist, drive_ms);
  runCommand("F", drive_ms);
  delay(300);

  // 5. Wall-seek: rotate to face start wall (180 deg in relative frame = back toward wall)
  float cur_angle = rel_angle;
  float wall_bearing = normalise180(180.0 - cur_angle);
  if (fabs(wall_bearing) > 5.0) {
    int rot_ms = (int)(fabs(wall_bearing) / ROTATE_SPEED_DEG_S * 1000);
    const char* rot_cmd = (wall_bearing > 0) ? "R" : "L";
    Serial.printf("Wall-seek align: %s %.1f deg\n", rot_cmd, fabs(wall_bearing));
    runCommand(rot_cmd, rot_ms);
    delay(200);
  }
  // Drive toward wall — fixed time to cover any remaining gap
  Serial.println("Wall-seek drive...");
  runCommand("F", 1500);

  runCommand("S", 0);
  Serial.println("\n=== DONE ===");
  Serial.printf("Final odom pos: (%.0f, %.0f)\n", rel_x, rel_y);
}

// ── Setup & loop ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  ArduinoSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  randomSeed(millis());

  delay(2000);   // give Arduino time to boot
  Serial.println("\n=== UnibotsMarkII Return Test ===");

  wander();
  delay(500);
  returnToStart();
}

void loop() {
  // nothing — test runs once on boot
}
