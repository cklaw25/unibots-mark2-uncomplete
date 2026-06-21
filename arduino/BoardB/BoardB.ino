// UnibotsMarkII — Board B Firmware
//
// Board B role: N20 spinner + N20 lift motor + encoders
// Powered by LM2596 5V buck from 12V LiPo
//
// GPIO4 protocol (Board A drives the wire, Board B reads it):
//   LOW  sustained        = Mode 1 active  → spin spinner
//   LOW → HIGH edge       = Mode 2 starting → stop spinner, rotate 90° gate, hold
//   HIGH → LOW 200ms → HIGH = deposit pulse → lift up, hold, lower
//   HIGH → LOW sustained  = Mode 1 resuming → rotate gate back -90°, resume spinner
//
// Pin map (from schematic_two_board.md, 2026-06-21):
//   TB6612 #1 Ch A = N20 #1 spinner: PWM=GPIO13, IN1=GPIO14, IN2=GPIO27
//   TB6612 #1 Ch B = N20 #2 lift:    PWM=GPIO16, IN1=GPIO17, IN2=GPIO18
//   N20 #1 encoder: C1=GPIO32, C2=GPIO33  (INPUT_PULLUP)
//   N20 #2 encoder: C1=GPIO19, C2=GPIO21  (unused — lift uses timed control)
//   Trigger IN: GPIO4
//
// STBY on TB6612 #1 hardwired HIGH — not a GPIO, do not touch.
//
// ══════════════════════════════════════════════════════
// ⚠️  CALIBRATION REMINDERS FOR TEAMMATES
//     Do these after first flash, before a real match:
//
//   1. SPINNER_PULSES_REV — open Serial Monitor, send 'm', spin the N20 #1
//      shaft slowly by hand for exactly ONE full revolution, note the delta
//      count printed. Set SPINNER_PULSES_REV to that value.
//      Current value (200) is a placeholder. Wrong value = gate angle is wrong.
//
//   2. LIFT_UP_MS / LIFT_DOWN_MS — run the lift (send 'l' in Serial Monitor),
//      watch if it fully extends and retracts. Increase if it doesn't reach the
//      end, decrease if it stalls. Current values (2000) are placeholders.
//
//   3. SPINNER_INVERT — if spinner runs backward (rejects balls instead of
//      pulling them in), flip SPINNER_INVERT from false → true.
// ══════════════════════════════════════════════════════
//
// Board: ESP32-D0WD-V3 (WROOM-32 classic, NOT S3)
// Flash: pio run -t upload --upload-port COMX   ⚠️ replace COMX with Board B's port
// Core:  ESP32 Arduino core 3.x (analogWrite available)

// ═══════════════════════════════════════════════════════════════════════════════
// PIN MAP
// ═══════════════════════════════════════════════════════════════════════════════

#define SPINNER_PWM   13
#define SPINNER_IN1   14
#define SPINNER_IN2   27

#define LIFT_PWM      16
#define LIFT_IN1      17
#define LIFT_IN2      18

#define ENC_C1        32   // N20 #1 encoder channel 1 (interrupt pin)
#define ENC_C2        33   // N20 #1 encoder channel 2 (direction sense)

#define PIN_TRIGGER    4   // INPUT from Board A

// ═══════════════════════════════════════════════════════════════════════════════
// TUNING CONSTANTS  ⚠️ calibrate before match
// ═══════════════════════════════════════════════════════════════════════════════

#define SPINNER_SPEED          180    // PWM for continuous intake spinning (0-255)
#define SPINNER_GATE_SPEED      80    // slower PWM for precise 90° gate moves
#define SPINNER_INVERT        false   // flip to true if spinner rejects balls

// ⚠️ CALIBRATE: send 'm' in Serial Monitor, spin shaft 1 full rev by hand, read delta
#define SPINNER_PULSES_REV     200

// ⚠️ CALIBRATE: time how long lift needs to fully extend and retract
#define LIFT_SPEED             180    // PWM for lift motor (0-255)
#define LIFT_UP_MS            2000UL  // time to fully raise lift
#define LIFT_HOLD_MS          1000UL  // time to hold at top before lowering
#define LIFT_DOWN_MS          2000UL  // time to fully lower lift

// Pulse width threshold: LOW pulse shorter than this = deposit signal.
// Longer = Mode 1 resuming. Must be > DEPOSIT_PULSE_MS (200ms) on Board A.
#define DEPOSIT_PULSE_THRESH_MS  500UL

// ═══════════════════════════════════════════════════════════════════════════════
// ENCODER  (N20 #1 quadrature)
// ═══════════════════════════════════════════════════════════════════════════════

volatile long encoderCount = 0;

void IRAM_ATTR onEncC1() {
    if (digitalRead(ENC_C2) == HIGH) encoderCount++;
    else                              encoderCount--;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void spinnerDrive(bool intakeDir, uint8_t speed) {
    bool fwd = SPINNER_INVERT ? !intakeDir : intakeDir;
    digitalWrite(SPINNER_IN1, fwd ? HIGH : LOW);
    digitalWrite(SPINNER_IN2, fwd ? LOW  : HIGH);
    analogWrite(SPINNER_PWM, speed);
}

void spinnerStop() {
    digitalWrite(SPINNER_IN1, LOW);
    digitalWrite(SPINNER_IN2, LOW);
    analogWrite(SPINNER_PWM, 0);
}

void liftDrive(bool up, uint8_t speed) {
    digitalWrite(LIFT_IN1, up ? HIGH : LOW);
    digitalWrite(LIFT_IN2, up ? LOW  : HIGH);
    analogWrite(LIFT_PWM, speed);
}

void liftStop() {
    digitalWrite(LIFT_IN1, LOW);
    digitalWrite(LIFT_IN2, LOW);
    analogWrite(LIFT_PWM, 0);
}

// Rotate spinner by exactly 'pulses' encoder counts in the given direction.
// Blocks until target reached. Uses SPINNER_GATE_SPEED for precision.
void spinnerRotatePulses(long pulses, bool forward) {
    long start   = encoderCount;
    long target  = start + (forward ? pulses : -pulses);
    spinnerDrive(forward, SPINNER_GATE_SPEED);
    unsigned long timeout = millis() + 5000;   // 5 s safety timeout
    if (forward) {
        while (encoderCount < target && millis() < timeout) {}
    } else {
        while (encoderCount > target && millis() < timeout) {}
    }
    spinnerStop();
    delay(50);
}

long pulsesFor90() {
    return (long)(SPINNER_PULSES_REV / 4);
}

// ═══════════════════════════════════════════════════════════════════════════════
// DEPOSIT SEQUENCE
// Called when Board A sends the 200 ms LOW pulse on GPIO4
// ═══════════════════════════════════════════════════════════════════════════════

void doDeposit() {
    Serial.println(F("[B] Lifting..."));
    liftDrive(true, LIFT_SPEED);
    delay(LIFT_UP_MS);
    liftStop();
    delay(LIFT_HOLD_MS);
    Serial.println(F("[B] Lowering..."));
    liftDrive(false, LIFT_SPEED);
    delay(LIFT_DOWN_MS);
    liftStop();
    Serial.println(F("[B] Deposit done."));
}

// ═══════════════════════════════════════════════════════════════════════════════
// BOARD B STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════════

enum BBState { BB_SPINNING, BB_GATE_HOLD, BB_GATE_OPENING };
BBState bbState = BB_SPINNING;

void loop() {
    // Serial test commands ('m' = encoder monitor, 'l' = test lift)
    checkSerialCommands();

    int trigger = digitalRead(PIN_TRIGGER);

    switch (bbState) {

        // ── SPINNING: Mode 1 active, spinner runs ──────────────────────────────
        case BB_SPINNING: {
            if (trigger == HIGH) {
                // Mode 2 just started — close the gate
                spinnerStop();
                Serial.println(F("[B] Mode 2: closing gate..."));
                spinnerRotatePulses(pulsesFor90(), true);
                Serial.println(F("[B] Gate closed. Holding."));
                bbState = BB_GATE_HOLD;
            }
            break;
        }

        // ── GATE_HOLD: Mode 2 active, gate at 90°, watch for pulse or resume ──
        case BB_GATE_HOLD: {
            if (trigger == LOW) {
                // Measure how long GPIO4 stays LOW
                unsigned long lowStart = millis();
                while (digitalRead(PIN_TRIGGER) == LOW) {
                    if (millis() - lowStart > DEPOSIT_PULSE_THRESH_MS) break;
                    delay(10);
                }

                if (digitalRead(PIN_TRIGGER) == HIGH) {
                    // Came back HIGH quickly → deposit pulse
                    Serial.println(F("[B] Deposit pulse — lifting."));
                    doDeposit();
                    // Remain in GATE_HOLD; Mode 2 is still active on Board A
                } else {
                    // Still LOW → Mode 1 is resuming
                    Serial.println(F("[B] Mode 1 resuming: opening gate..."));
                    bbState = BB_GATE_OPENING;
                }
            }
            break;
        }

        // ── GATE_OPENING: rotate back −90°, then resume spinning ──────────────
        case BB_GATE_OPENING: {
            spinnerRotatePulses(pulsesFor90(), false);
            Serial.println(F("[B] Gate open. Resuming spinner."));
            spinnerDrive(true, SPINNER_SPEED);
            bbState = BB_SPINNING;
            break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL TEST COMMANDS  (teammates use these to verify hardware before a match)
//   m = encoder monitor — spin N20 #1 shaft by hand for 8 s, prints counts
//   l = test lift cycle once (up → hold → down)
// ═══════════════════════════════════════════════════════════════════════════════

void checkSerialCommands() {
    if (!Serial.available()) return;
    char c = (char)Serial.read();

    if (c == 'm') {
        Serial.println(F("Encoder monitor — spin N20 shaft by hand for 8 s..."));
        Serial.println(F("One full revolution delta = SPINNER_PULSES_REV value to set."));
        long startCount = encoderCount;
        unsigned long t = millis();
        while (millis() - t < 8000) {
            Serial.print(F("  total: ")); Serial.print(encoderCount);
            Serial.print(F("  delta: ")); Serial.println(encoderCount - startCount);
            delay(250);
        }
        Serial.println(F("Done. If delta stayed 0, check C1/C2 pin wiring (ENC_C1=32, ENC_C2=33)."));
    }

    if (c == 'l') {
        Serial.println(F("Test lift cycle..."));
        doDeposit();
        Serial.println(F("Lift test done. Adjust LIFT_UP_MS/LIFT_DOWN_MS if needed."));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println(F("Board B booting..."));

    pinMode(SPINNER_PWM, OUTPUT);
    pinMode(SPINNER_IN1, OUTPUT);
    pinMode(SPINNER_IN2, OUTPUT);
    pinMode(LIFT_PWM,    OUTPUT);
    pinMode(LIFT_IN1,    OUTPUT);
    pinMode(LIFT_IN2,    OUTPUT);

    spinnerStop();
    liftStop();

    pinMode(ENC_C1, INPUT_PULLUP);
    pinMode(ENC_C2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_C1), onEncC1, RISING);

    pinMode(PIN_TRIGGER, INPUT);

    // Wait for Board A to finish booting and settle GPIO4
    delay(1500);

    if (digitalRead(PIN_TRIGGER) == LOW) {
        Serial.println(F("[B] Mode 1 at boot — starting spinner."));
        spinnerDrive(true, SPINNER_SPEED);
        bbState = BB_SPINNING;
    } else {
        Serial.println(F("[B] Mode 2 at boot — holding gate (no gate rotation at boot)."));
        bbState = BB_GATE_HOLD;
    }
}
