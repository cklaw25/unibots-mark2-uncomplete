// UnibotsMarkII — Main Match Firmware v2
//
// Combines:
//   - Confirmed 4-motor pin map + invert flags from HardwareTest.ino (Session 4)
//   - Mode 1 / Mode 2 FSM + odometry from UnibotsMain.ino
//
// Hardware assumptions:
//   - TT4 IN1/IN2 re-wired from GPIO1(TX0)/GPIO3(RX0) → GPIO18/GPIO19
//     (fixes UART0 conflict — Serial works during all drive phases)
//   - STBY on all 3 TB6612FNG chips hardwired to 3.3V — not a GPIO, not touched
//   - No collision sensor
//   - Stepper motor for ball deposit — pins TBD, stubs only (safe no-op until filled)
//
// Android → ESP32: ball-detection CSV over USB serial, 115200 baud
//   format: seq,ts,fw,fh,ntgt,has_ball,bx,by,bw,bh,prob,is_centred,err_x,err_y\n
//
// Board:   ESP32-D0WD-V3 (WROOM-32 classic — NOT S3)
// Flash:   pio run -t upload --upload-port COM7  (close Serial Monitor first)
// Core:    ESP32 Arduino core 3.x  (analogWrite available)

#include <Wire.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR PIN MAP
// Confirmed from hardware bring-up photos (pin codes details/*.jpg), 2026-06-16
// TT4 IN1/IN2 re-routed to GPIO18/19 — see hardware assumption above
// ═══════════════════════════════════════════════════════════════════════════════

struct MotorPins { const char* name; uint8_t pwm; uint8_t in1; uint8_t in2; bool invert; };

MotorPins motors[] = {
    { "TT1 front-right",  14,  13,  23,  false },
    { "TT2 front-left",   16,  17,   5,  true  },  // confirmed reversed on bench
    { "TT3 back-right",    4,  15,   2,  false },
    { "TT4 back-left",    12,  18,  19,  true  },  // confirmed reversed; IN1/IN2 re-routed from GPIO1/3
    { "N20 spinner",      25,  27,  26,  false },
};
#define NUM_TT   4   // indices 0-3 are drive wheels
#define IDX_N20  4   // index 4 is the N20 spinner

// N20 quadrature encoder — best guess from diagram, verify with HardwareTest 'm' command
#define N20_ENC_C1  32
#define N20_ENC_C2  33

// ═══════════════════════════════════════════════════════════════════════════════
// OTHER HARDWARE
// ═══════════════════════════════════════════════════════════════════════════════

// MPU6050 gyro — I2C, confirmed pins
#define PIN_SDA   21
#define PIN_SCL   22
#define MPU_ADDR  0x68

// ⚠️ STEPPER MOTOR — driver type and pins TBD, leave as 0 until confirmed
// When ready: set STEPPER_STEP_PIN / STEPPER_DIR_PIN / STEPPER_EN_PIN to real GPIOs,
// set STEPPER_STEPS_EXTEND to the step count that fully extends the mechanism,
// tune STEPPER_STEP_DELAY_US for your driver's speed limit.
// Wiring assumed: A4988 or DRV8825 style (STEP + DIR + EN-active-LOW).
#define STEPPER_STEP_PIN      0    // TBD
#define STEPPER_DIR_PIN       0    // TBD
#define STEPPER_EN_PIN        0    // TBD — active LOW; set 0 if not using enable pin
#define STEPPER_STEPS_EXTEND  200  // TBD — steps to fully extend for deposit
#define STEPPER_STEP_DELAY_US 800  // TBD — microseconds per half-step pulse

// ═══════════════════════════════════════════════════════════════════════════════
// TUNING CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define DRIVE_SPEED        150    // base forward PWM (0-255)
#define TURN_SPEED         120    // rotation PWM
#define KP                 1.8f   // heading-hold proportional gain
#define TURN_THRESH        3.0f   // degrees — stop turning when within this
#define GYRO_SIGN          1      // flip to -1 if heading corrections go the wrong way

// CALIBRATE after flashing: drive robot forward exactly 500 mm on flat floor,
// measure elapsed ms, set MS_PER_MM = measured_ms / 500.
#define MS_PER_MM          3.5f

#define MODE1_DURATION_MS   60000UL   // 60 s ball collection window
#define STARTUP_FORWARD_MS    700UL   // move forward at boot before Mode 1 starts
#define SCAN_STEP_MS          400UL   // CW rotation burst during scanning
#define SCAN_PAUSE_MS        1500UL   // still pause waiting for Android packet
#define ALIGN_STEP_MS         200UL   // each rotation pulse during alignment
#define ALIGN_PAUSE_MS         80UL   // stop gap between alignment pulses
#define COAST_MS             2000UL   // forward drive after ball leaves frame
#define FINE_APPROACH_MS      400UL   // each fine-approach pulse in Mode 2
#define BACKUP_MS             700UL   // reverse after deposit

#define CENTRE_ERR_PX    100    // pixels — ball centred if abs(err_x) < this
#define BALL_LOST_FRAMES   5    // consecutive no-ball frames → leave APPROACH

// ═══════════════════════════════════════════════════════════════════════════════
// STATE ENUMS
// ═══════════════════════════════════════════════════════════════════════════════

enum Mode   { MODE_1, MODE_2 };
enum State1 { S1_SCAN_STEP, S1_SCAN_PAUSE, S1_ALIGN, S1_APPROACH, S1_COAST };
enum State2 { S2_COMPUTE, S2_NAVIGATE, S2_AT_POSITION, S2_DEPOSIT, S2_RETRACT, S2_BACKUP };

Mode    currentMode   = MODE_1;
State1  currentState1 = S1_SCAN_STEP;
State2  currentState2 = S2_COMPUTE;

unsigned long mode1StartMs    = 0;
unsigned long stateStartMs    = 0;
float         approachHeading = 0.0f;  // heading locked when entering APPROACH
float         coastHeading    = 0.0f;  // heading locked when entering COAST

// ═══════════════════════════════════════════════════════════════════════════════
// N20 ENCODER
// ═══════════════════════════════════════════════════════════════════════════════

volatile long encoderCount = 0;

void IRAM_ATTR onEncoderC1() {
    if (digitalRead(N20_ENC_C2) == HIGH) encoderCount++;
    else                                  encoderCount--;
}

// ═══════════════════════════════════════════════════════════════════════════════
// GYRO (MPU6050 direct I2C — no library needed)
// Keep robot still for ~1s after boot during mpuCalibrate()
// ═══════════════════════════════════════════════════════════════════════════════

float         heading_deg  = 0.0f;
int32_t       gyro_offset  = 0;
unsigned long last_gyro_us = 0;

void mpuInit() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);   // wake up
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x00);   // ±250 °/s full scale
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
    if (fabsf(rate) < 1.0f) rate = 0.0f;  // deadband to suppress drift
    heading_deg += rate * dt;
}

float normalizeAngle(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ODOMETRY
// Dead-reckoning x/y from start (mm). Updated after each timed drive phase.
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
    const MotorPins &m = motors[idx];
    bool fwd = m.invert ? !forward : forward;
    digitalWrite(m.in1, fwd ? HIGH : LOW);
    digitalWrite(m.in2, fwd ? LOW  : HIGH);
    analogWrite(m.pwm, speed);
}

void stopMotor(int idx) {
    digitalWrite(motors[idx].in1, LOW);
    digitalWrite(motors[idx].in2, LOW);
    analogWrite(motors[idx].pwm, 0);
}

void stopAll() {
    for (int i = 0; i < (int)(sizeof(motors) / sizeof(motors[0])); i++) stopMotor(i);
}

// Rotate in place. CW = right wheels back, left wheels forward.
void rotateCW(uint8_t speed) {
    driveMotor(0, false, speed);   // TT1 front-right backward
    driveMotor(1, true,  speed);   // TT2 front-left  forward
    driveMotor(2, false, speed);   // TT3 back-right  backward
    driveMotor(3, true,  speed);   // TT4 back-left   forward
}

void rotateCCW(uint8_t speed) {
    driveMotor(0, true,  speed);
    driveMotor(1, false, speed);
    driveMotor(2, true,  speed);
    driveMotor(3, false, speed);
}

void spinnerOn()  { driveMotor(IDX_N20, true,  DRIVE_SPEED); }
void spinnerOff() { stopMotor(IDX_N20); }

// Drive all 4 TT wheels forward with gyro heading-hold for duration_ms.
// Reads Android serial each iteration. Returns false if Mode 1 timer fires mid-drive.
bool driveForwardHold(unsigned long duration_ms) {
    float target_h = heading_deg;
    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        updateHeading();
        readSerial();
        if (currentMode == MODE_1 && millis() - mode1StartMs >= MODE1_DURATION_MS) {
            stopAll();
            return false;
        }
        float   err  = normalizeAngle(heading_deg - target_h);
        uint8_t lspd = (uint8_t)constrain((int)(DRIVE_SPEED + KP * err), 80, 255);
        uint8_t rspd = (uint8_t)constrain((int)(DRIVE_SPEED - KP * err), 80, 255);
        driveMotor(0, true, rspd);   // right side
        driveMotor(1, true, lspd);   // left side
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

// Turn to an absolute heading (degrees). Gyro-controlled, slows as it approaches.
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
// STEPPER MOTOR — A4988 / DRV8825 style stubs
// Safe no-op while STEPPER_STEP_PIN == 0
// ═══════════════════════════════════════════════════════════════════════════════

void stepperInit() {
    if (STEPPER_STEP_PIN == 0) return;
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN,  OUTPUT);
    if (STEPPER_EN_PIN != 0) {
        pinMode(STEPPER_EN_PIN, OUTPUT);
        digitalWrite(STEPPER_EN_PIN, HIGH);   // disabled at start
    }
}

void stepperStep(bool extend, int steps) {
    if (STEPPER_STEP_PIN == 0) return;
    if (STEPPER_EN_PIN != 0) digitalWrite(STEPPER_EN_PIN, LOW);    // enable driver
    digitalWrite(STEPPER_DIR_PIN, extend ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(STEPPER_STEP_DELAY_US);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(STEPPER_STEP_DELAY_US);
    }
    if (STEPPER_EN_PIN != 0) digitalWrite(STEPPER_EN_PIN, HIGH);   // disable driver
}

void stepperDeposit() { stepperStep(true,  STEPPER_STEPS_EXTEND); }
void stepperRetract()  { stepperStep(false, STEPPER_STEPS_EXTEND); }

// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL PARSER
// Android sends CSV packets over USB serial. Non-blocking accumulation.
// Packet: seq,ts,fw,fh,ntgt,has_ball,bx,by,bw,bh,prob,is_centred,err_x,err_y\n
// ═══════════════════════════════════════════════════════════════════════════════

char    rxBuf[160];
uint8_t rxIdx       = 0;
bool    hasBall     = false;
bool    isCentred   = false;
int16_t errX        = 0;
uint8_t ballLostCnt = 0;

void parsePacket(char* buf) {
    int   field       = 0;
    char* tok         = strtok(buf, ",");
    int   hasBallRaw  = 0;
    int   isCentredRaw = 0;
    int   errXRaw     = 0;
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
// STATE TRANSITION HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void enterState1(State1 s) {
    currentState1 = s;
    stateStartMs  = millis();
    // Capture heading on entry so heading-hold has a stable reference
    if (s == S1_APPROACH) approachHeading = heading_deg;
    if (s == S1_COAST)    coastHeading    = heading_deg;
    const char* names[] = { "SCAN_STEP","SCAN_PAUSE","ALIGN","APPROACH","COAST" };
    Serial.print(F("[M1] ")); Serial.println(names[s]);
}

void enterState2(State2 s) {
    currentState2 = s;
    stateStartMs  = millis();
    const char* names[] = { "COMPUTE","NAVIGATE","AT_POSITION","DEPOSIT","RETRACT","BACKUP" };
    Serial.print(F("[M2] ")); Serial.println(names[s]);
}

unsigned long stateElapsed() { return millis() - stateStartMs; }

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 1 — BALL COLLECTION (60 s)
//
// SCAN_STEP  → rotate CW 400 ms
// SCAN_PAUSE → stop, wait for Android ball packet
// ALIGN      → pulse-rotate to centre ball; spinner ON at transition to APPROACH
// APPROACH   → drive forward gyro heading-hold, spinner ON
// COAST      → drive forward 2 s, spinner ON → log odom → back to SCAN_STEP
// ═══════════════════════════════════════════════════════════════════════════════

void runMode1() {
    Serial.println(F("=== MODE 1 START ==="));
    mode1StartMs = millis();
    enterState1(S1_SCAN_STEP);

    while (millis() - mode1StartMs < MODE1_DURATION_MS) {
        readSerial();
        updateHeading();

        switch (currentState1) {

            // ── SCAN_STEP: burst-rotate CW to sweep the field ─────────────────
            case S1_SCAN_STEP: {
                rotateCW(TURN_SPEED);
                if (stateElapsed() >= SCAN_STEP_MS) {
                    stopAll();
                    enterState1(S1_SCAN_PAUSE);
                }
                break;
            }

            // ── SCAN_PAUSE: stop and wait for an Android ball packet ──────────
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

            // ── ALIGN: pulse-rotate until ball is centred in frame ────────────
            // err_x > 0 = ball right of centre → rotate CCW (verified correct)
            // err_x < 0 = ball left of centre  → rotate CW
            case S1_ALIGN: {
                if (!hasBall) {
                    stopAll();
                    enterState1(S1_SCAN_STEP);
                    break;
                }
                if (isCentred || abs(errX) < CENTRE_ERR_PX) {
                    stopAll();
                    spinnerOn();
                    enterState1(S1_APPROACH);
                    break;
                }
                if (stateElapsed() < ALIGN_STEP_MS) {
                    if (errX > 0) rotateCCW(TURN_SPEED);
                    else          rotateCW(TURN_SPEED);
                } else {
                    stopAll();
                    delay(ALIGN_PAUSE_MS);
                    stateStartMs = millis();
                }
                break;
            }

            // ── APPROACH: drive forward toward ball with heading-hold ──────────
            // Spinner already ON (turned on at end of ALIGN).
            // Transitions to COAST once ball has left frame for BALL_LOST_FRAMES.
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

            // ── COAST: continue forward 2 s to ensure ball is fully collected ─
            // Spinner still ON. Log coast distance into odometry when done.
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
                    spinnerOff();
                    odomAddForward(COAST_MS);
                    enterState1(S1_SCAN_STEP);
                }
                break;
            }
        }
    }

    stopAll();
    spinnerOff();
    Serial.println(F("=== MODE 1 END — switching to MODE 2 ==="));
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 2 — RETURN TO START AND DEPOSIT
//
// COMPUTE     → dead-reckoning bearing + distance to home from odom_x/y
// NAVIGATE    → turnToHeading + driveForwardHold + 4 fine-approach pulses
// AT_POSITION → brief pause on arrival
// DEPOSIT     → stepper extends to dump balls
// RETRACT     → stepper retracts
// BACKUP      → reverse 700 ms → reset odometry → restart Mode 1
// ═══════════════════════════════════════════════════════════════════════════════

void runMode2() {
    Serial.println(F("=== MODE 2 START ==="));

    // ── COMPUTE ───────────────────────────────────────────────────────────────
    enterState2(S2_COMPUTE);
    float dist_home    = sqrtf(odom_x * odom_x + odom_y * odom_y);
    float home_heading = atan2f(-odom_y, -odom_x) * (180.0f / PI);
    Serial.print(F("  pos: (")); Serial.print(odom_x);
    Serial.print(F(", "));       Serial.print(odom_y);
    Serial.println(F(") mm"));
    Serial.print(F("  dist: "));    Serial.print(dist_home);    Serial.println(F(" mm"));
    Serial.print(F("  bearing: ")); Serial.print(home_heading); Serial.println(F(" deg"));
    delay(200);

    // ── NAVIGATE ──────────────────────────────────────────────────────────────
    enterState2(S2_NAVIGATE);
    turnToHeading(home_heading);
    driveForwardHold((unsigned long)(dist_home * MS_PER_MM));

    // Fine-approach pulses to close the last bit of distance
    for (int i = 0; i < 4; i++) {
        turnToHeading(home_heading);
        driveForwardHold(FINE_APPROACH_MS);
    }
    stopAll();

    // ── AT_POSITION ───────────────────────────────────────────────────────────
    enterState2(S2_AT_POSITION);
    Serial.println(F("  Arrived at home position."));
    delay(500);

    // ── DEPOSIT ───────────────────────────────────────────────────────────────
    enterState2(S2_DEPOSIT);
    stepperDeposit();

    // ── RETRACT ───────────────────────────────────────────────────────────────
    enterState2(S2_RETRACT);
    stepperRetract();

    // ── BACKUP ────────────────────────────────────────────────────────────────
    enterState2(S2_BACKUP);
    driveBackward(BACKUP_MS);

    odom_x = 0.0f;
    odom_y = 0.0f;
    Serial.println(F("=== MODE 2 END — restarting MODE 1 ===\n"));
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println(F("UnibotsMarkII v2 booting..."));

    // Initialise all motor pins
    for (int i = 0; i < (int)(sizeof(motors) / sizeof(motors[0])); i++) {
        pinMode(motors[i].in1, OUTPUT);
        pinMode(motors[i].in2, OUTPUT);
        pinMode(motors[i].pwm, OUTPUT);
        stopMotor(i);
    }

    // N20 encoder interrupt
    pinMode(N20_ENC_C1, INPUT_PULLUP);
    pinMode(N20_ENC_C2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(N20_ENC_C1), onEncoderC1, RISING);

    // Stepper (no-op if pins are 0)
    stepperInit();

    // Gyro — keep robot still during calibration (~1 s)
    mpuInit();
    delay(200);
    Serial.println(F("Calibrating gyro — keep still..."));
    mpuCalibrate();
    Serial.println(F("Gyro ready."));
    delay(2000);   // time to place robot on floor before it moves

    // Startup nudge — move forward before match begins, log to odometry
    Serial.println(F("Startup: moving forward..."));
    driveForwardHold(STARTUP_FORWARD_MS);
    odomAddForward(STARTUP_FORWARD_MS);

    // Main match loop — alternates Mode 1 / Mode 2 indefinitely
    while (true) {
        runMode1();
        runMode2();
    }
}

void loop() {
    // All logic runs inside setup()'s while(true).
    // loop() is never reached; safety stop in case it ever is.
    stopAll();
}
