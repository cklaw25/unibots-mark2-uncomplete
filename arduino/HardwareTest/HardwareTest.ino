// UnibotsMarkII — Hardware Bring-Up Test
//
// Purpose: confirm the ESP32 boots and that all 4 TT drive motors + the N20
// motor respond. Pin map below was read off the actual photographed wiring
// in "pin codes details/" (pin codes.jpeg, pin code png.jpg, pin code png 2.jpg)
// — this is the IMPROVISED wiring, not the original planned schematic.
//
// Usage: flash this, open Serial Monitor at 115200 baud, send one of these
// characters (no newline needed):
//   1 = TT1 front-right     2 = TT2 front-left
//   3 = TT3 back-right      4 = TT4 back-left   *** see TX/RX warning below ***
//   5 = N20 motor (also prints encoder pulse delta)
//   a = all 4 TT motors forward together (straight-line sanity check)
//   m = monitor encoder while you spin N20 by hand, motor unpowered
//   s = stop everything immediately
//
// Each numbered test: forward TEST_DURATION_MS -> stop -> reverse -> stop.
// Speed is deliberately low (TEST_SPEED) for bench safety.
//
// ─────────────────────────────────────────────────────────────────────────
// WIRING HAZARD — TT motor 4 (back left)
// BIN1/BIN2 are wired to GPIO1 (TX0) and GPIO3 (RX0) — the same pins the
// ESP32 uses for USB Serial. Claiming them as GPIO outputs detaches UART0,
// so Serial goes silent the moment this motor is driven. This script only
// takes over those pins for the duration of the '4' test (and inside the
// 'a' test), then calls Serial.begin() again to reattach USB Serial
// afterward — that's why you'll see a gap in Serial Monitor output during
// those two tests specifically. Don't upload new code while this motor's
// H-bridge is powered. Worth re-routing to free GPIOs (e.g. 18/19) later.
//
// STBY — the schematic photo shows all 3 TB6612FNG STBY pins tied straight
// to 3.3V (always enabled), not driven by a GPIO. This script does not
// touch STBY. If a motor never responds, check that chip's STBY line with
// a meter first — floating/low STBY is the most common "looks dead" cause.
//
// N20 ENCODER — the diagram's arrows for "N20 Encoder C1/C2" point at the
// pin row just above the PWMA/AIN1/AIN2 group, which lines up with GPIO32
// and GPIO33 — but a hand-drawn arrow at that distance is not 100% certain.
// Treat N20_ENC_C1/C2 below as a best guess, not a confirmed fact. Use the
// 'm' command to verify: it prints the pulse count and raw pin levels while
// you spin the N20 shaft by hand — if nothing moves when you spin it, the
// encoder is wired to different pins than guessed and these two #defines
// are the only thing you need to change.
//
// Board: ESP32-WROOM-32 DevKit, Arduino framework.
// analogWrite() requires ESP32 Arduino core 3.x — if you're on an older
// core, swap analogWrite() calls for ledcWrite().
// ─────────────────────────────────────────────────────────────────────────

// ───────────── PIN MAP (from pin codes details/*.jpg) ─────────────

// TT motor 1 — front right
#define TT1_PWM  14   // PWMA
#define TT1_IN1  13   // AIN1
#define TT1_IN2  23   // AIN2

// TT motor 2 — front left
#define TT2_PWM  16   // PWMB
#define TT2_IN1  17   // BIN1
#define TT2_IN2   5   // BIN2

// TT motor 3 — back right
#define TT3_PWM   4   // PWMA
#define TT3_IN1  15   // AIN1
#define TT3_IN2   2   // AIN2

// TT motor 4 — back left (shares USB Serial pins, handled specially below)
#define TT4_PWM  12   // PWMB
#define TT4_IN1   1   // BIN1 -> TX0
#define TT4_IN2   3   // BIN2 -> RX0
#define TT4_INDEX 3   // index of TT4 in motors[] — used to special-case it

// N20 motor (single channel, third TB6612 chip)
#define N20_PWM   25   // PWMA
#define N20_IN1   27   // AIN1
#define N20_IN2   26   // AIN2
#define N20_INDEX  4   // index of N20 in motors[] — used to report encoder counts

// N20 quadrature encoder — ⚠ best guess from diagram position, see note above
#define N20_ENC_C1  32
#define N20_ENC_C2  33

#define TEST_SPEED        130   // 0-255, kept low for bench safety
#define TEST_DURATION_MS  600
#define PAUSE_MS          300

struct MotorPins { const char* name; int pwm; int in1; int in2; bool invert; };

// invert=true means this motor's "forward" command came out backwards on the
// bench (2026-06-16 test) — IN1/IN2 wiring is swapped relative to the other
// motors, so we just flip direction in software instead of re-wiring.
// NOTE: TT3 was briefly marked invert=true on the assumption both back
// wheels shared the same fault — turned out wrong (TT3 went reverse AFTER
// being inverted, meaning it didn't need it). Left/right wheel pairs commonly
// need opposite polarity anyway due to mirrored mounting, so don't assume
// same-axle motors share a fix — confirm each one individually.
MotorPins motors[] = {
  { "TT1 front-right", TT1_PWM, TT1_IN1, TT1_IN2, false },  // 0
  { "TT2 front-left",  TT2_PWM, TT2_IN1, TT2_IN2, true  },  // 1 — confirmed reversed
  { "TT3 back-right",  TT3_PWM, TT3_IN1, TT3_IN2, false },  // 2 — reverted, did not need inverting
  { "TT4 back-left",   TT4_PWM, TT4_IN1, TT4_IN2, true  },  // 3 — confirmed reversed, TX/RX special-cased
  { "N20",             N20_PWM, N20_IN1, N20_IN2, false },  // 4
};
const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

volatile long encoderCount = 0;

void IRAM_ATTR onEncoderC1() {
  if (digitalRead(N20_ENC_C2) == HIGH) encoderCount++;
  else encoderCount--;
}

void driveMotor(const MotorPins &m, bool forward, int speed) {
  if (m.invert) forward = !forward;
  digitalWrite(m.in1, forward ? HIGH : LOW);
  digitalWrite(m.in2, forward ? LOW  : HIGH);
  analogWrite(m.pwm, speed);
}

void stopMotor(const MotorPins &m) {
  digitalWrite(m.in1, LOW);
  digitalWrite(m.in2, LOW);
  analogWrite(m.pwm, 0);
}

// Reclaim GPIO1/GPIO3 from UART0 so they can drive TT4's H-bridge.
void claimTT4Pins() {
  pinMode(TT4_IN1, OUTPUT);
  pinMode(TT4_IN2, OUTPUT);
}

// Hand GPIO1/GPIO3 back to UART0 so Serial Monitor works again.
void restoreSerialAfterTT4() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("[TT4] done — Serial restored."));
}

void stopAllExceptTT4() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (i == TT4_INDEX) continue;
    stopMotor(motors[i]);
  }
}

void testMotor(int idx) {
  const MotorPins &m = motors[idx];
  bool isTT4 = (idx == TT4_INDEX);
  bool isN20 = (idx == N20_INDEX);

  if (isTT4) {
    Serial.println(F("[TEST] TT4 back-left — taking over TX/RX, Serial will go quiet..."));
    delay(50);
    claimTT4Pins();
  } else {
    Serial.print(F("[TEST] ")); Serial.println(m.name);
  }

  long before = encoderCount;
  driveMotor(m, true, TEST_SPEED);
  delay(TEST_DURATION_MS);
  stopMotor(m);
  if (isN20) {
    Serial.print(F("  forward encoder delta: ")); Serial.println(encoderCount - before);
  }
  delay(PAUSE_MS);

  before = encoderCount;
  driveMotor(m, false, TEST_SPEED);
  delay(TEST_DURATION_MS);
  stopMotor(m);
  if (isN20) {
    Serial.print(F("  reverse encoder delta: ")); Serial.println(encoderCount - before);
  }

  if (isTT4) {
    restoreSerialAfterTT4();
  } else {
    Serial.println(F("  done.\n"));
  }
}

// Spin the N20 by hand (motor unpowered) and watch pulse count + raw pin
// levels — confirms which physical pins the encoder is actually on,
// independent of the N20_ENC_C1/C2 guess above.
void monitorEncoder() {
  Serial.println(F("[MONITOR] spin the N20 shaft by hand now. Reporting for 8s..."));
  unsigned long start = millis();
  long lastCount = encoderCount;
  while (millis() - start < 8000) {
    if (encoderCount != lastCount) {
      lastCount = encoderCount;
      Serial.print(F("  count=")); Serial.print(encoderCount);
      Serial.print(F("  C1=")); Serial.print(digitalRead(N20_ENC_C1));
      Serial.print(F("  C2=")); Serial.println(digitalRead(N20_ENC_C2));
    }
  }
  Serial.print(F("[MONITOR] done. total count=")); Serial.println(encoderCount);
  if (lastCount == 0 && encoderCount == 0) {
    Serial.println(F("  No pulses seen — N20_ENC_C1/C2 are probably the wrong pins, check the photo again."));
  }
}

// Multimeter continuity check for the TT1/TT2 chip: holds each pin HIGH
// one at a time so you can probe the matching TB6612 leg and confirm the
// ESP32 GPIO is actually the one wired to it. If a pin shows ~3.3V at the
// expected leg but the motor still won't spin even with both AIN1/AIN2 set
// for "forward" and PWMA high, suspect that chip's STBY/VM power instead.
struct ProbePin { const char* label; int pin; };
ProbePin probeList[] = {
  { "TT1 PWMA (GPIO14)", TT1_PWM },
  { "TT1 AIN1 (GPIO13)", TT1_IN1 },
  { "TT1 AIN2 (GPIO23)", TT1_IN2 },
  { "TT2 PWMB (GPIO16)", TT2_PWM },
  { "TT2 BIN1 (GPIO17)", TT2_IN1 },
  { "TT2 BIN2 (GPIO5)",  TT2_IN2 },
};
const int NUM_PROBE = sizeof(probeList) / sizeof(probeList[0]);

void probeTT12() {
  Serial.println(F("[PROBE] Holding each TT1/TT2 pin HIGH for 3s, one at a time."));
  Serial.println(F("        Put your multimeter on the matching TB6612 leg and watch for ~3.3V."));
  for (int i = 0; i < NUM_PROBE; i++) {
    Serial.print(F("  -> ")); Serial.print(probeList[i].label); Serial.println(F(" = HIGH now"));
    digitalWrite(probeList[i].pin, HIGH);
    delay(3000);
    digitalWrite(probeList[i].pin, LOW);
    delay(200);
  }
  Serial.println(F("[PROBE] done — all pins back LOW."));
}

// Holds TT1 then TT2 in a steady forward drive for several seconds each —
// long enough to multimeter the TB6612 OUTPUT screw terminals (where the
// motor itself connects), not the ESP32 control pins. This is the more
// direct test for "chip is powered but motor won't spin": if these
// terminals never show a voltage, the chip isn't actually driving its
// output stage (STBY/VM problem) even if the p command shows clean input
// continuity. A01(-)/A02(+) = TT1's terminals, B01(-)/B02(+) = TT2's.
void holdOutputTest(int idx, const char* terminalNote) {
  const MotorPins &m = motors[idx];
  Serial.print(F("[OUTPUT TEST] ")); Serial.print(m.name);
  Serial.print(F(" — driving forward for 6s. Multimeter across "));
  Serial.println(terminalNote);
  driveMotor(m, true, TEST_SPEED);
  delay(6000);
  stopMotor(m);
  Serial.println(F("  done.\n"));
}

void testOutputTerminals() {
  holdOutputTest(0, "TT1's A01(-)/A02(+) screw terminals");
  delay(500);
  holdOutputTest(1, "TT2's B01(-)/B02(+) screw terminals");
}

void testAllTT() {
  Serial.println(F("[TEST] all 4 TT motors forward together — TX/RX will go quiet for TT4..."));
  delay(50);
  claimTT4Pins();

  for (int i = 0; i < 4; i++) driveMotor(motors[i], true, TEST_SPEED);
  delay(TEST_DURATION_MS * 2);
  for (int i = 0; i < 4; i++) stopMotor(motors[i]);

  restoreSerialAfterTT4();
}

void stopAll() {
  stopAllExceptTT4();
  // Only touch TT4's pins if they're currently claimed as GPIO; safe either way.
  claimTT4Pins();
  stopMotor(motors[TT4_INDEX]);
  restoreSerialAfterTT4();
  Serial.println(F("[STOP] all motors stopped"));
}

void printMenu() {
  Serial.println(F("\n=== UnibotsMarkII Hardware Bring-Up Test ==="));
  Serial.println(F("1 = TT1 front-right   2 = TT2 front-left"));
  Serial.println(F("3 = TT3 back-right    4 = TT4 back-left (shares TX/RX, Serial pauses)"));
  Serial.println(F("5 = N20 motor (also reports encoder pulse delta)"));
  Serial.println(F("a = all 4 TT motors forward together"));
  Serial.println(F("m = monitor encoder while you spin N20 by hand (motor unpowered)"));
  Serial.println(F("p = probe TT1/TT2 input pins one at a time (multimeter continuity check)"));
  Serial.println(F("o = hold TT1/TT2 forward 6s each (multimeter the A01/A02, B01/B02 output terminals)"));
  Serial.println(F("s = STOP everything"));
  Serial.println(F("Send a character to run that test.\n"));
}

unsigned long lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Set up every motor except TT4 — its IN1/IN2 stay on UART0 duty until
  // the '4' or 'a' test explicitly claims them.
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (i == TT4_INDEX) {
      pinMode(motors[i].pwm, OUTPUT);   // PWM pin (12) is safe to claim now
      analogWrite(motors[i].pwm, 0);
      continue;
    }
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    pinMode(motors[i].pwm, OUTPUT);
    stopMotor(motors[i]);
  }

  pinMode(N20_ENC_C1, INPUT_PULLUP);
  pinMode(N20_ENC_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(N20_ENC_C1), onEncoderC1, RISING);

  Serial.println(F("ESP32 booted and responding."));
  printMenu();
  lastHeartbeat = millis();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': testMotor(0); break;
      case '2': testMotor(1); break;
      case '3': testMotor(2); break;
      case '4': testMotor(3); break;
      case '5': testMotor(4); break;
      case 'a': case 'A': testAllTT(); break;
      case 'm': case 'M': monitorEncoder(); break;
      case 'p': case 'P': probeTT12(); break;
      case 'o': case 'O': testOutputTerminals(); break;
      case 's': case 'S': stopAll(); break;
      case '\n': case '\r': break;  // ignore newline noise
      default:
        Serial.print(F("unknown command: ")); Serial.println(c);
        printMenu();
    }
  }

  // Heartbeat so you know the ESP32 is alive even with no input.
  if (millis() - lastHeartbeat > 5000) {
    Serial.println(F("[heartbeat] ESP32 alive, waiting for command..."));
    lastHeartbeat = millis();
  }
}
