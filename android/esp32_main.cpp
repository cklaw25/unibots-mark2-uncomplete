#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- WiFi Access Point config ----------
const char* AP_SSID     = "PingPong_ESP32";
const char* AP_PASSWORD = "12345678";

// ---------- UDP config ----------
const int UDP_PORT = 4210;
WiFiUDP udp;

// ---------- LED ----------
#define LED_PIN 2

// ---------- Motor pins ----------
#define BACK_LEFT_IN1   26
#define BACK_LEFT_IN2   25
#define FRONT_LEFT_IN1  33
#define FRONT_LEFT_IN2  32

#define BACK_RIGHT_IN1  14
#define BACK_RIGHT_IN2  27
#define FRONT_RIGHT_IN1 23
#define FRONT_RIGHT_IN2 22

#define LIFT_UP   19
#define LIFT_DOWN 18

// ---------- Spinner motor (hits ball) ----------
#define SPINNER_IN3 15
#define SPINNER_IN4 21
// Time (ms) to spin ~120 degrees — tune based on motor speed
#define SPINNER_DEG120_MS 300

// ---------- Step-scan rotation constants ----------
#define ROTATE_STEP_MS  400   // ms to rotate ~45 degrees — tune based on motor speed
#define SCAN_PAUSE_MS   1500  // ms to pause still and scan for ball
#define ALIGN_STEP_MS   200   // ms per alignment pulse toward ball

// ---------- Packet structure ----------
struct BallPacket {
    bool has_primary;
    bool in_center;
    int  bx, by, bw, bh;
    int  err_x, err_y;
    int  frame_w, frame_h;
};

bool parse_packet(const char* buf, BallPacket& pkt) {
    unsigned int frame_seq, timestamp_ms;
    int fw, fh, num_targets, hp;
    int x, y, w, h, prob, ic, ex, ey;

    int parsed = sscanf(buf, "%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                        &frame_seq, &timestamp_ms,
                        &fw, &fh, &num_targets, &hp,
                        &x, &y, &w, &h, &prob, &ic, &ex, &ey);
    if (parsed < 6) return false;

    pkt.has_primary = hp;
    pkt.frame_w     = fw;
    pkt.frame_h     = fh;

    if (hp && parsed == 14) {
        pkt.bx        = x;  pkt.by    = y;
        pkt.bw        = w;  pkt.bh    = h;
        pkt.in_center = ic;
        pkt.err_x     = ex; pkt.err_y = ey;
    } else {
        pkt.bx = pkt.by = pkt.bw = pkt.bh = 0;
        pkt.in_center = false;
        pkt.err_x = pkt.err_y = 0;
    }
    return true;
}

// ---------- LED flash helper ----------
void flash_led_3_times() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

// ---------- Motor helpers ----------
void set_motor(int in1, int in2, bool forward) {
    digitalWrite(in1, forward ? HIGH : LOW);
    digitalWrite(in2, forward ? LOW  : HIGH);
}

void stop_all_motors() {
    digitalWrite(BACK_LEFT_IN1,   LOW); digitalWrite(BACK_LEFT_IN2,   LOW);
    digitalWrite(FRONT_LEFT_IN1,  LOW); digitalWrite(FRONT_LEFT_IN2,  LOW);
    digitalWrite(BACK_RIGHT_IN1,  LOW); digitalWrite(BACK_RIGHT_IN2,  LOW);
    digitalWrite(FRONT_RIGHT_IN1, LOW); digitalWrite(FRONT_RIGHT_IN2, LOW);
    digitalWrite(SPINNER_IN3, LOW);     digitalWrite(SPINNER_IN4,     LOW);
}

void run_spinner_sequence() {
    // CW 120 degrees
    digitalWrite(SPINNER_IN3, HIGH);
    digitalWrite(SPINNER_IN4, LOW);
    delay(SPINNER_DEG120_MS);
    // Stop
    digitalWrite(SPINNER_IN3, LOW);
    digitalWrite(SPINNER_IN4, LOW);
    delay(300);
    // CCW 120 degrees
    digitalWrite(SPINNER_IN3, LOW);
    digitalWrite(SPINNER_IN4, HIGH);
    delay(SPINNER_DEG120_MS);
    // Stop
    digitalWrite(SPINNER_IN3, LOW);
    digitalWrite(SPINNER_IN4, LOW);
    Serial.println("[SPINNER] Sequence complete");
}

void set_rotate() {
    set_motor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   true);
    set_motor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  true);
    set_motor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  false);
    set_motor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, false);
}

void move_forward() {
    set_motor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   false);
    set_motor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  false);
    set_motor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  false);
    set_motor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, false);
}

void move_backward() {
    set_motor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   true);
    set_motor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  true);
    set_motor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  true);
    set_motor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, true);
}

// Clockwise = left motors forward, right motors backward
// Counter-clockwise = left motors backward, right motors forward
void set_rotate_ccw() {
    set_motor(BACK_LEFT_IN1,   BACK_LEFT_IN2,   false);
    set_motor(FRONT_LEFT_IN1,  FRONT_LEFT_IN2,  false);
    set_motor(BACK_RIGHT_IN1,  BACK_RIGHT_IN2,  true);
    set_motor(FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, true);
}

// ---------- FSM ----------
// STATE_STEP_ROTATE : rotate CW for ROTATE_STEP_MS (~45 degrees)
// STATE_SCAN_PAUSE  : stopped still for SCAN_PAUSE_MS, camera detects ball
// STATE_ALIGN_ROTATE: short rotation pulse toward detected ball
// STATE_MOVING_FORWARD / STATE_COASTING : same as before
enum State { STATE_STEP_ROTATE, STATE_SCAN_PAUSE, STATE_ALIGN_ROTATE,
             STATE_MOVING_FORWARD, STATE_COASTING };

State current_state   = STATE_STEP_ROTATE;
unsigned long coast_start_ms  = 0;
unsigned long step_start_ms   = 0;
unsigned long scan_start_ms   = 0;
unsigned long align_start_ms  = 0;

// Part 2 mode flag — set when CMD packets are received, cleared on ball packets
bool g_part2_active = false;

void start_step_rotate() {
    set_rotate();
    step_start_ms = millis();
    current_state = STATE_STEP_ROTATE;
    Serial.println("[FSM] Step rotating CW");
}

// ---------- Part 2 command executor ----------
void execute_p2_command(const char* cmd) {
    Serial.print("[P2] "); Serial.println(cmd);
    if (strcmp(cmd, "STOP") == 0) {
        stop_all_motors();
        digitalWrite(LIFT_UP,   LOW);
        digitalWrite(LIFT_DOWN, LOW);
    } else if (strcmp(cmd, "ROTATE_CW") == 0) {
        set_rotate();
    } else if (strcmp(cmd, "ROTATE_CCW") == 0) {
        set_rotate_ccw();
    } else if (strcmp(cmd, "FORWARD") == 0) {
        move_forward();
    } else if (strcmp(cmd, "BACKWARD") == 0) {
        move_backward();
    } else if (strcmp(cmd, "LIFT_UP") == 0) {
        stop_all_motors();
        digitalWrite(LIFT_DOWN, LOW);
        digitalWrite(LIFT_UP,   HIGH);
    } else if (strcmp(cmd, "LIFT_DOWN") == 0) {
        stop_all_motors();
        digitalWrite(LIFT_UP,   LOW);
        digitalWrite(LIFT_DOWN, HIGH);
    }
}

// Returns true if buf was a Part 2 command packet (CMD,...) and was handled
bool try_handle_command(const char* buf) {
    if (strncmp(buf, "CMD,", 4) != 0) return false;

    // Extract command name (strip any trailing whitespace)
    char cmd[32];
    strncpy(cmd, buf + 4, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    int len = strlen(cmd);
    while (len > 0 && (cmd[len - 1] == '\r' || cmd[len - 1] == '\n' || cmd[len - 1] == ' '))
        cmd[--len] = '\0';

    if (!g_part2_active) {
        g_part2_active = true;
        stop_all_motors();
        Serial.println("[MODE] Part 2 active");
    }
    execute_p2_command(cmd);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    int pins[] = {BACK_LEFT_IN1, BACK_LEFT_IN2, FRONT_LEFT_IN1, FRONT_LEFT_IN2,
                  BACK_RIGHT_IN1, BACK_RIGHT_IN2, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2,
                  LIFT_UP, LIFT_DOWN, SPINNER_IN3, SPINNER_IN4};
    for (int p : pins) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }

    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("[WiFi] AP started. IP: ");
    Serial.println(WiFi.softAPIP());

    udp.begin(UDP_PORT);
    Serial.print("[UDP] Listening on port ");
    Serial.println(UDP_PORT);

    Serial.println("[FSM] Starting — step rotate scan");
    start_step_rotate();
}

// ---------- Shared FSM handler (called from both UDP and USB serial paths) ----------
void process_packet(const BallPacket& pkt) {
    // Returning from Part 2 — reset Part 1 FSM
    if (g_part2_active) {
        g_part2_active = false;
        digitalWrite(LIFT_UP,   LOW);
        digitalWrite(LIFT_DOWN, LOW);
        stop_all_motors();
        delay(100);
        start_step_rotate();
        Serial.println("[MODE] Part 1 active — FSM reset");
    }

    Serial.print("[PKT] has_primary="); Serial.print(pkt.has_primary);
    Serial.print(" in_center=");        Serial.print(pkt.in_center);
    Serial.print(" state=");            Serial.println(current_state);

    switch (current_state) {

        case STATE_STEP_ROTATE:
            // Ignore packets while stepping — let loop() handle the timeout
            break;

        case STATE_SCAN_PAUSE:
            if (pkt.has_primary && pkt.in_center) {
                Serial.println("[FSM] Ball in centre — flashing LED x3 then moving forward");
                flash_led_3_times();
                digitalWrite(LED_PIN, HIGH);
                stop_all_motors();
                delay(100);
                move_forward();
                current_state = STATE_MOVING_FORWARD;
            } else if (pkt.has_primary) {
                // Ball visible but not centred — do a short align pulse toward it
                Serial.print("[FSM] Ball off centre err_x="); Serial.print(pkt.err_x);
                Serial.println(" — aligning");
                scan_start_ms = millis(); // reset scan timer, we can see the ball
                if (pkt.err_x > 0) set_rotate_ccw(); // ball is right → rotate CCW
                else               set_rotate();     // ball is left  → rotate CW
                align_start_ms = millis();
                current_state  = STATE_ALIGN_ROTATE;
            }
            // No ball → let SCAN_PAUSE timeout in loop() trigger next step
            break;

        case STATE_ALIGN_ROTATE:
            // Ignore packets while aligning — let loop() handle the timeout
            break;

        case STATE_MOVING_FORWARD:
            if (!pkt.in_center) {
                Serial.println("[FSM] Ball left centre — coasting 2.5s");
                coast_start_ms = millis();
                current_state = STATE_COASTING;
            }
            break;

        case STATE_COASTING:
            if (millis() - coast_start_ms >= 2500) {
                if (pkt.in_center) {
                    Serial.println("[FSM] Back in centre — continue forward");
                    current_state = STATE_MOVING_FORWARD;
                } else {
                    Serial.println("[FSM] Not in centre — running spinner then step rotating");
                    digitalWrite(LED_PIN, LOW);
                    stop_all_motors();
                    delay(100);
                    run_spinner_sequence();
                    start_step_rotate();
                }
            }
            break;
    }
}

void loop() {
    // ---------- Part 1 step-scan timing (non-blocking) ----------
    if (!g_part2_active) {
        unsigned long now = millis();

        if (current_state == STATE_STEP_ROTATE) {
            if (now - step_start_ms >= ROTATE_STEP_MS) {
                stop_all_motors();
                scan_start_ms = now;
                current_state = STATE_SCAN_PAUSE;
                Serial.println("[FSM] Step done — scanning pause");
            }
        }
        else if (current_state == STATE_SCAN_PAUSE) {
            if (now - scan_start_ms >= SCAN_PAUSE_MS) {
                // No ball found in this window — take another step
                Serial.println("[FSM] No ball in pause — stepping again");
                start_step_rotate();
            }
        }
        else if (current_state == STATE_ALIGN_ROTATE) {
            if (now - align_start_ms >= ALIGN_STEP_MS) {
                // Short align pulse done — pause again to re-evaluate
                stop_all_motors();
                scan_start_ms = millis();
                current_state = STATE_SCAN_PAUSE;
            }
        }
    }

    // ---- UDP path (WiFi) ----
    int packet_size = udp.parsePacket();
    if (packet_size > 0) {
        char buf[256];
        int len = udp.read(buf, sizeof(buf) - 1);
        buf[len] = '\0';
        if (!try_handle_command(buf)) {
            BallPacket pkt;
            if (!parse_packet(buf, pkt)) {
                Serial.println("[WARN] Bad UDP packet");
            } else {
                Serial.print("[UDP] ");
                process_packet(pkt);
            }
        }
    }

    // ---- USB Serial path (Android OTG cable) ----
    static char serial_buf[256];
    static int  serial_buf_pos = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n') {
            serial_buf[serial_buf_pos] = '\0';
            int pos = serial_buf_pos;
            serial_buf_pos = 0;
            if (pos > 5) {
                if (!try_handle_command(serial_buf)) {
                    BallPacket pkt;
                    if (parse_packet(serial_buf, pkt)) {
                        Serial.print("[USB] ");
                        process_packet(pkt);
                    }
                }
            }
        } else if (serial_buf_pos < (int)sizeof(serial_buf) - 1) {
            serial_buf[serial_buf_pos++] = c;
        }
    }
}
