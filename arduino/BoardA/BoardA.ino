// UnibotsMarkII — Board A Firmware
//
// Board A role: 4x TT drive wheels + MPU6050 gyro + Android phone serial + GPIO4 trigger
// Board B role: N20 spinner + N20 lift motor (see BoardB.ino)
//
// GPIO4 protocol (Board A drives the wire, Board B reads it):
//   LOW  sustained        = Mode 1 active  → Board B spins spinner continuously
//   LOW → HIGH            = Mode 2 starting → Board B stops spinner, rotates 90° gate, holds
//   HIGH → LOW 200ms → HIGH = deposit pulse → Board B runs lift sequence
//   HIGH → LOW sustained  = Mode 1 resuming → Board B rotates gate back, resumes spinner
//
// Pin map: schematic_two_board.md (2026-06-21)
//   TB6612 #2 front wheels: GPIO13/14/16 (right), GPIO17/18/19 (left)
//   TB6612 #3 back wheels:  GPIO23/25/26 (right), GPIO27/32/33 (left)
//   MPU6050: SDA=GPIO21, SCL=GPIO22
//   Trigger out: GPIO4
//
// STBY on both TB6612 chips hardwired HIGH — not a GPIO, do not touch.
//
// ══════════════════════════════════════════════════════
// ⚠️  CALIBRATION REMINDERS FOR TEAMMATES
//     Do these immediately after first flash:
//
//   1. INVERT FLAGS — if any wheel drives backward when it should go forward,
//      open this file and flip its 'invert' field from false → true (or vice versa).
//      Test with: drive robot forward, watch which wheels go the wrong way.
//      Left-side wheels (TT2, TT4) are likely inverted; right-side likely not.
//
//   2. MS_PER_MM — put robot on flat floor, drive it exactly 500 mm forward,
//      time it in milliseconds. Set MS_PER_MM = measured_ms / 500.
//      Current value (3.5) is a placeholder. Wrong value = robot misses home.
//
//   3. Keep robot STILL for ~1 second after power-on (gyro calibration window).
// ══════════════════════════════════════════════════════
//
// Board: ESP32-D0WD-V3 (WROOM-32 classic, NOT S3)
// Flash: pio run -t upload --upload-port COM7   (from arduino/BoardA/)
// Core:  ESP32 Arduino core 3.x (analogWrite available)

#include <Wire.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════════
// PIN MAP  (from schematic_two_board.md)
// ═══════════════════════════════════════════════════════════════════════════════

struct MotorPins { const char* name; uint8_t pwm; uint8_t in1; uint8_t in2; bool invert; };

// ⚠️ Invert flags: flip to true if that wheel runs backward. Left-side motors
// typically need invert=true depending on how the motor wires land on AO/BO outputs.
MotorPins tt[] = {
    { "TT1 front-right", 13, 14, 16, false },   // TB6612 #2 Ch A: PWMA=13 AIN1=14 AIN2=16
    { "TT2 front-left",  17, 18, 19, true  },   // TB6612 #2 Ch B: PWMB=17 BIN1=18 BIN2=19
    { "TT3 back-right",  23, 25, 26, false },   // TB6612 #3 Ch A: PWMA=23 AIN1=25 AIN2=26
    { "TT4 back-left",   27, 32, 33, true  },   // TB6612 #3 Ch B: PWMB=27 BIN1=32 BIN2=33
};

#define PIN_TRIGGER  4    // GPIO4 output → Board B
#define PIN_SDA     21
#define PIN_SCL     22
#define MPU_ADDR  0x68

// ═══════════════════════════════════════════════════════════════════════════════
// TUNING CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define DRIVE_SPEED          150    // forward PWM (0-255)
#define TURN_SPEED           120    // rotation PWM
#define KP                   1.8f  // heading-hold proportional gain
#define TURN_THRESH          3.0f  // degrees — stop turning when within this
#define GYRO_SIGN            1     // flip to -1 if heading corrections go the wrong way

// ⚠️ CALIBRATE: drive 500 mm, time it, set = measured_ms / 500
#define MS_PER_MM            3.5f

#define MODE1_DURATION_MS   60000UL
#define STARTUP_FORWARD_MS    700UL
#define SCAN_STEP_MS          400UL
#define SCAN_PAUSE_MS        1500UL
#define ALIGN_STEP_MS         200UL
#define ALIGN_PAUSE_MS         80UL
#define COAST_MS             2000UL
#define FINE_APPROACH_MS      400UL
#define BACKUP_MS             700UL
#define CENTRE_ERR_PX          100
#define BALL_LOST_FRAMES         5

// GPIO4 deposit signalling
#define DEPOSIT_PULSE_MS     200UL   // how long GPIO4 dips LOW to trigger Board B lift
#define DEPOSIT_WAIT_MS     6000UL   // time for Board B to complete full lift cycle

// ═══════════════════════════════════════════════════════════════════════════════
// STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════════

enum Mode   { MODE_1, MODE_2 };
enum State1 { S1_SCAN_STEP, S1_SCAN_PAUSE, S1_ALIGN, S1_APPROACH, S1_COAST };
enum State2 { S2_COMPUTE, S2_NAVIGATE, S2_AT_POSITION, S2_DEPOSIT, S2_BACKUP };

Mode    currentMode   = MODE_1;
State1  currentState1 = S1_SCAN_STEP;
State2  currentState2 = S2_COMPUTE;

unsigned long mode1StartMs    = 0;
unsigned long stateStartMs    = 0;
float         approachHeading = 0.0f;
float         coastHeading    = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════════
// GYRO  (MPU6050 direct I2C, no library)
// Keep robot still for ~1 s after boot during mpuCalibrate()
// ═══════════════════════════════════════════════════════════════════════════════

float         heading_deg  = 0.0f;
int32_t       gyro_offset  = 0;
unsigned long last_gyro_us = 0;

void mpuInit() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);   // wake
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x00);   // ±250 °/s
    Wire.endTransmission();
}

void mpuCalibrate() {
    int32_t sum = 0;
    for (int i = 0; i < 200; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 2);
        int16_t gz = ((int16_t)Wire.read() << 8) | Wire.read();
        sum += gz;
        delay(5);
    }
    gyro_offset  = sum / 200;
    last_gyro_us = micros();
}

void updateHeading() {
    unsigned long now_us = micros();
    float dt = (now_us - last_gyro_us) / 1000000.0f;
    last_gyro_us = now_us;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);
    int16_t gz_raw = ((int16_t)Wire.read() << 8) | Wire.read();
    float rate = GYRO_SIGN * (gz_raw - gyro_offset) / 131.0f;
    if (fabsf(rate) < 1.0f) rate = 0.0f;   // deadband to suppress drift
    heading_deg += rate * dt;
}

float normalizeAngle(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ODOMETRY  (dead-reckoning x/y from home, updated after each COAST)
// ═══════════════════════════════════════════════════════════════════════════════

float odom_x = 0.0f;
float odom_y = 0.0f;

void odomAddForward(unsigned long duration_ms) {
    float dist_mm = (float)duration_ms / MS_PER_MM;
    float rad     = heading_deg * (PI / 180.0f);
    odom_x += dist_mm * cosf(rad);
    odom_y += dist_mm * sinf(rad);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void driveMotor(int idx, bool forward, uint8_t speed) {
    bool fwd = tt[idx].invert ? !forward : forward;
    digitalWrite(tt[idx].in1, fwd ? HIGH : LOW);
    digitalWrite(tt[idx].in2, fwd ? LOW  : HIGH);
    analogWrite(tt[idx].pwm, speed);
}

void stopMotor(int idx) {
    digitalWrite(tt[idx].in1, LOW);
    digitalWrite(tt[idx].in2, LOW);
    analogWrite(tt[idx].pwm, 0);
}

void stopAll() {
    for (int i = 0; i < 4; i++) stopMotor(i);
}

void rotateCW(uint8_t speed) {
    driveMotor(0, false, speed);   // right wheels back
    driveMotor(1, true,  speed);   // left wheels forward
    driveMotor(2, false, speed);
    driveMotor(3, true,  speed);
}

void rotateCCW(uint8_t speed) {
    driveMotor(0, true,  speed);
    driveMotor(1, false, speed);
    driveMotor(2, true,  speed);
    driveMotor(3, false, speed);
}

// Drive forward with gyro heading-hold. Returns false if Mode 1 timer fires.
bool driveForwardHold(unsigned long duration_ms) {
    float target_h = heading_deg;
    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        updateHeading();
        readSerial();
        // Only check the Mode 1 timer while actually in Mode 1
        if (currentMode == MODE_1 && millis() - mode1StartMs >= MODE1_DURATION_MS) {
            stopAll();
            return false;
        }
        float   err  = normalizeAngle(heading_deg - target_h);
        uint8_t lspd = (uint8_t)constrain((int)(DRIVE_SPEED + KP * err), 80, 255);
        uint8_t rspd = (uint8_t)constrain((int)(DRIVE_SPEED - KP * err), 80, 255);
        driveMotor(0, true, rspd);
        driveMotor(1, true, lspd);
        driveMotor(2, true, rspd);
        driveMotor(3, true, lspd);
    }
    stopAll();
    delay(80);
    return true;
}

void driveBackward(unsigned long duration_ms) {
    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        updateHeading();
        driveMotor(0, false, DRIVE_SPEED);
        driveMotor(1, false, DRIVE_SPEED);
        driveMotor(2, false, DRIVE_SPEED);
        driveMotor(3, false, DRIVE_SPEED);
    }
    stopAll();
    delay(80);
}

void turnToHeading(float target_deg) {
    float delta;
    do {
        updateHeading();
        delta = normalizeAngle(target_deg - heading_deg);
        uint8_t spd = (uint8_t)constrain((int)(fabsf(delta) * 2.5f), 70, TURN_SPEED);
        if      (delta >  TURN_THRESH) rotateCW(spd);
        else if (delta < -TURN_THRESH) rotateCCW(spd);
    } while (fabsf(delta) > TURN_THRESH);
    stopAll();
    delay(120);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL PARSER  (Android → Board A, ball detection CSV, non-blocking)
// Packet: seq,ts,fw,fh,ntgt,has_ball,bx,by,bw,bh,prob,is_centred,err_x,err_y\n
// ═══════════════════════════════════════════════════════════════════════════════

char    rxBuf[160];
uint8_t rxIdx       = 0;
bool    hasBall     = false;
bool    isCentred   = false;
int16_t errX        = 0;
uint8_t ballLostCnt = 0;

void parsePacket(char* buf) {
    int   field        = 0;
    char* tok          = strtok(buf, ",");
    int   hasBallRaw   = 0;
    int   isCentredRaw = 0;
    int   errXRaw      = 0;
    while (tok != NULL) {
        switch (field) {
            case 5:  hasBallRaw   = atoi(tok); break;
            case 11: isCentredRaw = atoi(tok); break;
            case 12: errXRaw      = atoi(tok); break;
        }
        field++;
        tok = strtok(NULL, ",");
    }
    hasBall   = (hasBallRaw   != 0);
    isCentred = (isCentredRaw != 0);
    errX      = (int16_t)errXRaw;
}

void readSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rxBuf[rxIdx] = '\0';
            parsePacket(rxBuf);
            rxIdx = 0;
        } else if (rxIdx < sizeof(rxBuf) - 1) {
            rxBuf[rxIdx++] = c;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// STATE HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void enterState1(State1 s) {
    currentState1 = s;
    stateStartMs  = millis();
    if (s == S1_APPROACH) approachHeading = heading_deg;
    if (s == S1_COAST)    coastHeading    = heading_deg;
    const char* names[] = { "SCAN_STEP","SCAN_PAUSE","ALIGN","APPROACH","COAST" };
    Serial.print(F("[M1] ")); Serial.println(names[s]);
}

void enterState2(State2 s) {
    currentState2 = s;
    stateStartMs  = millis();
    const char* names[] = { "COMPUTE","NAVIGATE","AT_POSITION","DEPOSIT","BACKUP" };
    Serial.print(F("[M2] ")); Serial.println(names[s]);
}

unsigned long stateElapsed() { return millis() - stateStartMs; }

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 1 — BALL COLLECTION  (60 s)
//
// GPIO4 = LOW the entire time → Board B spins spinner continuously.
//
// FSM:
//   SCAN_STEP  → rotate CW in short bursts to sweep field
//   SCAN_PAUSE → wait for ball packet from Android
//   ALIGN      → pulse-rotate to centre ball in frame
//   APPROACH   → drive forward with heading-hold until ball leaves frame
//   COAST      → continue 2 s to collect, log to odometry, back to SCAN_STEP
// ═══════════════════════════════════════════════════════════════════════════════

void runMode1() {
    Serial.println(F("=== MODE 1 START ==="));
    currentMode  = MODE_1;
    digitalWrite(PIN_TRIGGER, LOW);   // Board B: spinner ON
    mode1StartMs = millis();
    enterState1(S1_SCAN_STEP);

    while (millis() - mode1StartMs < MODE1_DURATION_MS) {
        readSerial();
        updateHeading();

        switch (currentState1) {

            case S1_SCAN_STEP: {
                rotateCW(TURN_SPEED);
                if (stateElapsed() >= SCAN_STEP_MS) {
                    stopAll();
                    enterState1(S1_SCAN_PAUSE);
                }
                break;
            }

            case S1_SCAN_PAUSE: {
                stopAll();
                if (hasBall) {
                    ballLostCnt = 0;
                    enterState1(S1_ALIGN);
                } else if (stateElapsed() >= SCAN_PAUSE_MS) {
                    enterState1(S1_SCAN_STEP);
                }
                break;
            }

            case S1_ALIGN: {
                if (!hasBall) { stopAll(); enterState1(S1_SCAN_STEP); break; }
                if (isCentred || abs(errX) < CENTRE_ERR_PX) {
                    stopAll();
                    enterState1(S1_APPROACH);
                    break;
                }
                if (stateElapsed() < ALIGN_STEP_MS) {
                    // err_x > 0 = ball right of centre → rotate CCW (verified correct)
                    if (errX > 0) rotateCCW(TURN_SPEED);
                    else          rotateCW(TURN_SPEED);
                } else {
                    stopAll();
                    delay(ALIGN_PAUSE_MS);
                    stateStartMs = millis();
                }
                break;
            }

            case S1_APPROACH: {
                if (hasBall) {
                    ballLostCnt = 0;
                } else {
                    ballLostCnt++;
                    if (ballLostCnt >= BALL_LOST_FRAMES) {
                        stopAll();
                        enterState1(S1_COAST);
                        break;
                    }
                }
                float   err  = normalizeAngle(heading_deg - approachHeading);
                uint8_t lspd = (uint8_t)constrain((int)(DRIVE_SPEED + KP * err), 80, 255);
                uint8_t rspd = (uint8_t)constrain((int)(DRIVE_SPEED - KP * err), 80, 255);
                driveMotor(0, true, rspd);
                driveMotor(1, true, lspd);
                driveMotor(2, true, rspd);
                driveMotor(3, true, lspd);
                break;
            }

            case S1_COAST: {
                float   err  = normalizeAngle(heading_deg - coastHeading);
                uint8_t lspd = (uint8_t)constrain((int)(DRIVE_SPEED + KP * err), 80, 255);
                uint8_t rspd = (uint8_t)constrain((int)(DRIVE_SPEED - KP * err), 80, 255);
                driveMotor(0, true, rspd);
                driveMotor(1, true, lspd);
                driveMotor(2, true, rspd);
                driveMotor(3, true, lspd);
                if (stateElapsed() >= COAST_MS) {
                    stopAll();
                    odomAddForward(COAST_MS);
                    enterState1(S1_SCAN_STEP);
                }
                break;
            }
        }
    }

    stopAll();
    Serial.println(F("=== MODE 1 END — switching to MODE 2 ==="));
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 2 — RETURN TO START AND DEPOSIT
//
// GPIO4 = HIGH on entry → Board B stops spinner, rotates 90° gate, holds.
// At deposit: pulse GPIO4 LOW 200 ms → Board B lifts, holds, lowers.
// GPIO4 = LOW on exit (set by runMode1 at start of next cycle) → Board B resumes.
//
// FSM:
//   COMPUTE     → bearing + distance to home from odometry
//   NAVIGATE    → turn to bearing, drive home, 4 fine-approach pulses
//   AT_POSITION → arrived home
//   DEPOSIT     → pulse GPIO4 to trigger Board B lift, wait for completion
//   BACKUP      → reverse away from deposit zone, reset odometry
// ═══════════════════════════════════════════════════════════════════════════════

void runMode2() {
    Serial.println(F("=== MODE 2 START ==="));
    currentMode = MODE_2;
    digitalWrite(PIN_TRIGGER, HIGH);   // Board B: gate closes

    // ── COMPUTE ───────────────────────────────────────────────────────────────
    enterState2(S2_COMPUTE);
    float dist_home    = sqrtf(odom_x * odom_x + odom_y * odom_y);
    float home_heading = atan2f(-odom_y, -odom_x) * (180.0f / PI);
    Serial.print(F("  pos mm: (")); Serial.print(odom_x);
    Serial.print(F(", "));          Serial.print(odom_y); Serial.println(F(")"));
    Serial.print(F("  dist:    ")); Serial.print(dist_home);    Serial.println(F(" mm"));
    Serial.print(F("  bearing: ")); Serial.print(home_heading); Serial.println(F(" deg"));
    delay(200);

    // ── NAVIGATE ──────────────────────────────────────────────────────────────
    enterState2(S2_NAVIGATE);
    turnToHeading(home_heading);
    driveForwardHold((unsigned long)(dist_home * MS_PER_MM));

    for (int i = 0; i < 4; i++) {
        turnToHeading(home_heading);
        driveForwardHold(FINE_APPROACH_MS);
    }
    stopAll();

    // ── AT_POSITION ───────────────────────────────────────────────────────────
    enterState2(S2_AT_POSITION);
    Serial.println(F("  Arrived at home."));
    delay(300);

    // ── DEPOSIT ───────────────────────────────────────────────────────────────
    enterState2(S2_DEPOSIT);
    Serial.println(F("  Pulsing GPIO4 → Board B lift sequence..."));
    digitalWrite(PIN_TRIGGER, LOW);
    delay(DEPOSIT_PULSE_MS);
    digitalWrite(PIN_TRIGGER, HIGH);
    delay(DEPOSIT_WAIT_MS);   // wait for Board B to complete lift + lower

    // ── BACKUP ────────────────────────────────────────────────────────────────
    enterState2(S2_BACKUP);
    driveBackward(BACKUP_MS);

    odom_x = 0.0f;
    odom_y = 0.0f;
    Serial.println(F("=== MODE 2 END — restarting MODE 1 ===\n"));
    // runMode1() will immediately set GPIO4 LOW → Board B opens gate, resumes spinner
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR TEST MODE
// At boot, there is a 5-second window to send 't' over Serial Monitor to enter
// test mode. If nothing is sent, the match starts automatically.
//
// Test mode commands:
//   1-4 = test that TT motor (forward 1s → stop → reverse 1s → stop)
//   a   = all 4 wheels forward together 2s (check straight-line drive)
//   d   = drive all 4 forward for 5s → stop (measure distance for MS_PER_MM)
//   q   = quit test mode and start the match
//
// HOW TO CALIBRATE MS_PER_MM using 'd':
//   1. Place robot on flat floor, mark the start position
//   2. Send 'd' — robot drives forward 5 seconds, then stops
//   3. Measure the distance traveled in mm (e.g. 450mm)
//   4. Calculate: MS_PER_MM = 5000 / 450 = 11.1
//   5. Set MS_PER_MM to that value at the top of this file, then re-flash
// ═══════════════════════════════════════════════════════════════════════════════

void motorTestMode() {
    Serial.println(F(""));
    Serial.println(F("=== MOTOR TEST MODE ==="));
    Serial.println(F("1=TT1  2=TT2  3=TT3  4=TT4  a=all_fwd  d=drive_5s_calib  q=start_match"));

    while (true) {
        if (!Serial.available()) continue;
        char c = (char)Serial.read();

        if (c >= '1' && c <= '4') {
            int idx = c - '1';
            Serial.print(F("Testing TT")); Serial.print(idx + 1);
            Serial.println(F(": fwd 1s -> stop -> rev 1s -> stop"));
            driveMotor(idx, true,  150); delay(1000); stopMotor(idx); delay(400);
            driveMotor(idx, false, 150); delay(1000); stopMotor(idx); delay(400);
            Serial.println(F("Done. If direction was wrong, flip 'invert' in the code."));
        }

        if (c == 'a') {
            Serial.println(F("All 4 TT forward 2s — watch for straight-line drive..."));
            for (int i = 0; i < 4; i++) driveMotor(i, true, 150);
            delay(2000);
            stopAll();
            Serial.println(F("Done."));
        }

        if (c == 'd') {
            Serial.println(F("DISTANCE CALIBRATION: driving forward 5 seconds..."));
            Serial.println(F("Mark start position, measure distance to stop position in mm."));
            Serial.println(F("Then set: MS_PER_MM = 5000 / measured_mm"));
            for (int i = 0; i < 4; i++) driveMotor(i, true, DRIVE_SPEED);
            delay(5000);
            stopAll();
            Serial.println(F("Stopped. Measure distance now."));
        }

        if (c == 'q') {
            Serial.println(F("Exiting test mode — starting match."));
            Serial.println(F(""));
            break;
        }
    }
}

// Forward declaration needed because readSerial is used in driveForwardHold
void readSerial();

void setup() {
    Serial.begin(115200);
    Serial.println(F("Board A booting..."));

    for (int i = 0; i < 4; i++) {
        pinMode(tt[i].in1, OUTPUT);
        pinMode(tt[i].in2, OUTPUT);
        pinMode(tt[i].pwm, OUTPUT);
        stopMotor(i);
    }

    pinMode(PIN_TRIGGER, OUTPUT);
    digitalWrite(PIN_TRIGGER, LOW);   // default LOW = Mode 1

    // ── 5-second test mode window ─────────────────────────────────────────────
    Serial.println(F("Send 't' within 5 seconds to enter motor test mode..."));
    Serial.println(F("(nothing sent = match starts automatically)"));
    unsigned long testWindowStart = millis();
    while (millis() - testWindowStart < 5000) {
        if (Serial.available() && (char)Serial.peek() == 't') {
            Serial.read();
            motorTestMode();
            break;
        }
    }

    // ── Gyro calibration ──────────────────────────────────────────────────────
    mpuInit();
    delay(200);
    Serial.println(F("Calibrating gyro — keep robot STILL for 1 second..."));
    mpuCalibrate();
    Serial.println(F("Gyro ready."));
    delay(2000);   // extra settle time before first movement

    Serial.println(F("Startup: nudge forward..."));
    driveForwardHold(STARTUP_FORWARD_MS);
    odomAddForward(STARTUP_FORWARD_MS);

    while (true) {
        runMode1();
        runMode2();
    }
}

void loop() {
    stopAll();   // safety net — never reached under normal operation
}
