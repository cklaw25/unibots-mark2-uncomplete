// UnibotsMarkII — Main Control Sketch (ESP32)
//
// MODE 1 (60 s): Ball collection
//   SCAN_STEP  → rotate CW 400 ms
//   SCAN_PAUSE → stop 1.5 s, wait for Android ball packet
//   ALIGN      → pulse-rotate to centre ball (200 ms steps)
//   APPROACH   → drive forward, heading-hold; spinner ON
//   COAST      → drive forward 2 s, spinner ON; then → SCAN_STEP
//
// MODE 2: Return to start and deposit
//   COMPUTE    → compute bearing + distance to home from dead-reckoning
//   NAVIGATE   → turn to face home, drive home, fine approach
//   AT_POSITION→ confirm arrival
//   DEPOSIT    → linear actuator UP 1 s
//   RETRACT    → linear actuator DOWN
//   BACKUP     → reverse 0.7 s → restart MODE 1
//
// Serial input: Android sends ball-detection CSV packets (115200 baud)
//   format: seq,ts,fw,fh,ntgt,has_ball,bx,by,bw,bh,prob,is_centred,err_x,err_y\n
//
// Board: ESP32 (Arduino framework, platformio or Arduino IDE with ESP32 core)
// analogWrite() requires ESP32 Arduino core 3.x — if older, swap to ledcWrite()

#include <Wire.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS  — all PLACEHOLDERS, fill in after hardware wiring
// ═══════════════════════════════════════════════════════════════════════════════
// ESP32-S3 forbidden pins (do NOT use):
//   GPIO 19, 20 = USB D-/D+
//   GPIO 35, 36, 37 = PSRAM
//   GPIO 22-25 = do not exist on S3

// ⚠️  Drive motors — give real pin numbers  ⚠️
#define PIN_PWMA   0    // Right speed (PWM)
#define PIN_PWMB   0    // Left  speed (PWM)
#define PIN_AIN    0    // Right direction: HIGH = forward
#define PIN_BIN    0    // Left  direction: HIGH = forward
#define PIN_STBY   0    // Motor enable: HIGH = on

// ⚠️  Other hardware — give real pin numbers  ⚠️
#define PIN_SPINNER_1     0    // Spinner motor IN1
#define PIN_SPINNER_2     0    // Spinner motor IN2
#define PIN_ACTUATOR_UP   0    // Linear actuator extend
#define PIN_ACTUATOR_DOWN 0    // Linear actuator retract
#define PIN_COLLISION     0    // Collision sensor (INPUT_PULLUP)

// MPU6050 I2C pins — ESP32 default is 21/22, change if wired differently
#define PIN_SDA   21
#define PIN_SCL   22

// ═══════════════════════════════════════════════════════════════════════════════
// TUNING CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define SPEED           150   // Base forward speed (0-255)
#define TURN_SPEED      120   // Base turn speed
#define KP              1.8f  // Heading-hold P-gain
#define TURN_THRESH     3.0f  // Degrees — stop turning within this
#define GYRO_SIGN       1     // Flip to -1 if turns go the wrong way
#define MPU_ADDR        0x68

// ── CALIBRATE MS_PER_MM after flashing ──
// Measure: run robot forward 500 mm, time it in ms → MS_PER_MM = ms / 500
#define MS_PER_MM       3.5f

#define MODE1_DURATION_MS    60000UL
#define STARTUP_FORWARD_MS     700UL
#define SCAN_STEP_MS           400UL
#define SCAN_PAUSE_MS         1500UL
#define ALIGN_STEP_MS          200UL
#define COAST_MS              2000UL
#define ACTUATOR_HOLD_MS      1000UL
#define ACTUATOR_RETRACT_MS    600UL
#define BACKUP_MS              700UL

// Ball packet: err_x within this many pixels = "centred"
#define CENTRE_ERR_PX          100
// Consecutive frames with no ball before APPROACH → COAST
#define BALL_LOST_FRAMES         5

// ═══════════════════════════════════════════════════════════════════════════════
// STATE ENUMS
// ═══════════════════════════════════════════════════════════════════════════════

enum Mode   { MODE_1, MODE_2 };

enum State1 {
    S1_SCAN_STEP,
    S1_SCAN_PAUSE,
    S1_ALIGN,
    S1_APPROACH,
    S1_COAST
};

enum State2 {
    S2_COMPUTE,
    S2_NAVIGATE,
    S2_AT_POSITION,
    S2_DEPOSIT,
    S2_RETRACT,
    S2_BACKUP
};

Mode    currentMode   = MODE_1;
State1  currentState1 = S1_SCAN_STEP;
State2  currentState2 = S2_COMPUTE;

unsigned long mode1StartMs = 0;
unsigned long stateStartMs = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// GYRO (MPU6050 direct I2C — no library)
// ═══════════════════════════════════════════════════════════════════════════════

float         heading_deg  = 0.0f;
int32_t       gyro_offset  = 0;
unsigned long last_gyro_us = 0;

void mpuInit() {
    Wire.begin(PIN_SDA, PIN_SCL);   // ESP32 requires explicit SDA/SCL
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);   // wake up
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
    gyro_offset = sum / 200;
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
    if (fabsf(rate) < 1.0f) rate = 0.0f;
    heading_deg += rate * dt;
}

float normalizeAngle(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ODOMETRY
// ═══════════════════════════════════════════════════════════════════════════════

float odom_x = 0.0f;   // mm from start
float odom_y = 0.0f;

void odomAddForward(unsigned long duration_ms) {
    float dist_mm = (float)duration_ms / MS_PER_MM;
    float rad = heading_deg * (PI / 180.0f);
    odom_x += dist_mm * cosf(rad);
    odom_y += dist_mm * sinf(rad);
}

// ═══════════════════════════════════════════════════════════════════════════════
// MOTOR HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void motorStop() {
    digitalWrite(PIN_STBY, LOW);
    analogWrite(PIN_PWMA, 0);
    analogWrite(PIN_PWMB, 0);
}

void setMotors(uint8_t rSpd, uint8_t lSpd, bool rFwd, bool lFwd) {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_AIN, rFwd ? HIGH : LOW);
    digitalWrite(PIN_BIN, lFwd ? HIGH : LOW);
    analogWrite(PIN_PWMA, rSpd);
    analogWrite(PIN_PWMB, lSpd);
}

// Rotate CW in place until target heading reached (gyro-controlled).
void turnToHeading(float target_deg) {
    float delta;
    do {
        updateHeading();
        delta = normalizeAngle(target_deg - heading_deg);
        uint8_t spd = (uint8_t)constrain((int)(fabsf(delta) * 2.5f), 70, TURN_SPEED);
        if      (delta >  TURN_THRESH) setMotors(spd, spd, false, true);   // CW
        else if (delta < -TURN_THRESH) setMotors(spd, spd, true,  false);  // CCW
    } while (fabsf(delta) > TURN_THRESH);
    motorStop();
    delay(120);
}

// Drive forward with gyro heading-hold; also reads serial & checks mode timer.
// Returns early if mode timer fires.
bool driveForwardHold(unsigned long duration_ms) {
    float target_h = heading_deg;
    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        updateHeading();
        readSerial();
        if (millis() - mode1StartMs >= MODE1_DURATION_MS && currentMode == MODE_1) {
            motorStop();
            return false;  // mode timer fired — caller switches mode
        }
        float err  = normalizeAngle(heading_deg - target_h);
        int   lspd = constrain((int)(SPEED + KP * err), 80, 255);
        int   rspd = constrain((int)(SPEED - KP * err), 80, 255);
        setMotors((uint8_t)rspd, (uint8_t)lspd, true, true);
    }
    motorStop();
    delay(80);
    return true;
}

// Drive backward (no heading hold needed for short backup).
void driveBackward(unsigned long duration_ms) {
    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        updateHeading();
        setMotors(SPEED, SPEED, false, false);
    }
    motorStop();
    delay(80);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SPINNER
// ═══════════════════════════════════════════════════════════════════════════════

void spinnerOn() {
    // ⚠️ Placeholder — replace PIN_SPINNER_1/2 with actual pins
    if (PIN_SPINNER_1 == 0) return;
    digitalWrite(PIN_SPINNER_1, HIGH);
    digitalWrite(PIN_SPINNER_2, LOW);
}

void spinnerOff() {
    if (PIN_SPINNER_1 == 0) return;
    digitalWrite(PIN_SPINNER_1, LOW);
    digitalWrite(PIN_SPINNER_2, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════════
// LINEAR ACTUATOR
// ═══════════════════════════════════════════════════════════════════════════════

void actuatorUp() {
    if (PIN_ACTUATOR_UP == 0) return;   // ⚠️ placeholder
    digitalWrite(PIN_ACTUATOR_UP, HIGH);
    digitalWrite(PIN_ACTUATOR_DOWN, LOW);
}

void actuatorDown() {
    if (PIN_ACTUATOR_UP == 0) return;
    digitalWrite(PIN_ACTUATOR_UP, LOW);
    digitalWrite(PIN_ACTUATOR_DOWN, HIGH);
}

void actuatorStop() {
    if (PIN_ACTUATOR_UP == 0) return;
    digitalWrite(PIN_ACTUATOR_UP, LOW);
    digitalWrite(PIN_ACTUATOR_DOWN, LOW);
}

// ═══════════════════════════════════════════════════════════════════════════════
// SERIAL PARSER (non-blocking, accumulates chars until '\n')
// Packet: seq,ts,fw,fh,ntgt,has_ball,bx,by,bw,bh,prob,is_centred,err_x,err_y\n
// ═══════════════════════════════════════════════════════════════════════════════

char    rxBuf[160];
uint8_t rxIdx   = 0;
bool    hasBall     = false;
bool    isCentred   = false;
int16_t errX        = 0;
uint8_t ballLostCnt = 0;

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

void parsePacket(char* buf) {
    // Fields: 0=seq 1=ts 2=fw 3=fh 4=ntgt 5=has_ball 6=bx 7=by 8=bw 9=bh
    //         10=prob 11=is_centred 12=err_x 13=err_y
    int field = 0;
    char* tok = strtok(buf, ",");
    int   hasBallRaw = 0, isCentredRaw = 0, errXRaw = 0;
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

// ═══════════════════════════════════════════════════════════════════════════════
// STATE TRANSITION HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

void enterState1(State1 s) {
    currentState1 = s;
    stateStartMs  = millis();
    const char* names[] = {"SCAN_STEP","SCAN_PAUSE","ALIGN","APPROACH","COAST"};
    Serial.print(F("[MODE1] ")); Serial.println(names[s]);
}

void enterState2(State2 s) {
    currentState2 = s;
    stateStartMs  = millis();
    const char* names[] = {"COMPUTE","NAVIGATE","AT_POSITION","DEPOSIT","RETRACT","BACKUP"};
    Serial.print(F("[MODE2] ")); Serial.println(names[s]);
}

unsigned long stateElapsed() { return millis() - stateStartMs; }

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 1 — BALL COLLECTION
// ═══════════════════════════════════════════════════════════════════════════════

void runMode1() {
    Serial.println(F("=== MODE 1 START ==="));
    mode1StartMs = millis();
    enterState1(S1_SCAN_STEP);

    while (millis() - mode1StartMs < MODE1_DURATION_MS) {
        readSerial();
        updateHeading();

        switch (currentState1) {

            // ── SCAN_STEP: rotate CW for SCAN_STEP_MS ─────────────────────────
            case S1_SCAN_STEP: {
                setMotors(TURN_SPEED, TURN_SPEED, false, true);  // CW rotation
                if (stateElapsed() >= SCAN_STEP_MS) {
                    motorStop();
                    enterState1(S1_SCAN_PAUSE);
                }
                break;
            }

            // ── SCAN_PAUSE: stand still, wait for ball packet ──────────────────
            case S1_SCAN_PAUSE: {
                motorStop();
                if (hasBall) {
                    ballLostCnt = 0;
                    enterState1(S1_ALIGN);
                } else if (stateElapsed() >= SCAN_PAUSE_MS) {
                    enterState1(S1_SCAN_STEP);
                }
                break;
            }

            // ── ALIGN: pulse-rotate to centre the ball ─────────────────────────
            case S1_ALIGN: {
                if (!hasBall) {
                    motorStop();
                    enterState1(S1_SCAN_STEP);
                    break;
                }
                if (isCentred || abs(errX) < CENTRE_ERR_PX) {
                    motorStop();
                    spinnerOn();
                    enterState1(S1_APPROACH);
                    break;
                }
                // Pulse-rotate toward ball
                if (stateElapsed() < ALIGN_STEP_MS) {
                    // err_x > 0 = ball right of centre → rotate CCW
                    if (errX > 0) setMotors(TURN_SPEED, TURN_SPEED, true,  false);  // CCW
                    else          setMotors(TURN_SPEED, TURN_SPEED, false, true);   // CW
                } else {
                    motorStop();
                    delay(80);
                    stateStartMs = millis();  // reset pulse timer
                }
                break;
            }

            // ── APPROACH: drive toward ball until it leaves frame ──────────────
            case S1_APPROACH: {
                if (hasBall) {
                    ballLostCnt = 0;
                } else {
                    ballLostCnt++;
                    if (ballLostCnt >= BALL_LOST_FRAMES) {
                        motorStop();
                        enterState1(S1_COAST);
                        break;
                    }
                }
                // Heading-hold forward
                float err  = normalizeAngle(heading_deg - heading_deg);  // locked at entry
                int   lspd = constrain((int)(SPEED + KP * err), 80, 255);
                int   rspd = constrain((int)(SPEED - KP * err), 80, 255);
                setMotors((uint8_t)rspd, (uint8_t)lspd, true, true);
                break;
            }

            // ── COAST: continue forward 2 s then back to scanning ─────────────
            case S1_COAST: {
                float target_h = heading_deg;
                float err  = normalizeAngle(heading_deg - target_h);
                int   lspd = constrain((int)(SPEED + KP * err), 80, 255);
                int   rspd = constrain((int)(SPEED - KP * err), 80, 255);
                setMotors((uint8_t)rspd, (uint8_t)lspd, true, true);

                if (stateElapsed() >= COAST_MS) {
                    motorStop();
                    spinnerOff();
                    // Record coast distance to odometry
                    odomAddForward(COAST_MS);
                    enterState1(S1_SCAN_STEP);
                }
                break;
            }
        }
    }

    motorStop();
    spinnerOff();
    Serial.println(F("=== MODE 1 END — switching to MODE 2 ==="));
}

// ═══════════════════════════════════════════════════════════════════════════════
// MODE 2 — RETURN TO START AND DEPOSIT
// ═══════════════════════════════════════════════════════════════════════════════

void runMode2() {
    Serial.println(F("=== MODE 2 START ==="));

    // ── COMPUTE: bearing + distance to home ───────────────────────────────────
    enterState2(S2_COMPUTE);
    float dist_home = sqrtf(odom_x * odom_x + odom_y * odom_y);
    float home_heading = atan2f(-odom_y, -odom_x) * (180.0f / PI);
    Serial.print(F("  pos: (")); Serial.print(odom_x); Serial.print(F(", ")); Serial.print(odom_y); Serial.println(F(")"));
    Serial.print(F("  dist: ")); Serial.print(dist_home); Serial.println(F(" mm"));
    Serial.print(F("  bearing: ")); Serial.print(home_heading); Serial.println(F(" deg"));
    delay(200);

    // ── NAVIGATE: turn to face home, drive home ────────────────────────────────
    enterState2(S2_NAVIGATE);
    turnToHeading(home_heading);

    bool ok = driveForwardHold((unsigned long)(dist_home * MS_PER_MM));
    if (!ok) {
        Serial.println(F("  Mode timer fired mid-return — continuing anyway"));
    }

    // Fine approach: a few short pulses to reach wall
    for (int i = 0; i < 4; i++) {
        float bearing_now = atan2f(-odom_y - (float)i*50.0f, -odom_x) * (180.0f / PI);
        turnToHeading(bearing_now);
        driveForwardHold(400);
    }

    motorStop();

    // ── AT_POSITION ───────────────────────────────────────────────────────────
    enterState2(S2_AT_POSITION);
    Serial.println(F("  Arrived at home position"));
    delay(500);

    // ── DEPOSIT: linear actuator up ───────────────────────────────────────────
    enterState2(S2_DEPOSIT);
    actuatorUp();
    delay(ACTUATOR_HOLD_MS);

    // ── RETRACT ───────────────────────────────────────────────────────────────
    enterState2(S2_RETRACT);
    actuatorDown();
    delay(ACTUATOR_RETRACT_MS);
    actuatorStop();

    // ── BACKUP ────────────────────────────────────────────────────────────────
    enterState2(S2_BACKUP);
    driveBackward(BACKUP_MS);

    // Reset odometry for next Mode 1 cycle
    odom_x = 0.0f;
    odom_y = 0.0f;

    Serial.println(F("=== MODE 2 END — restarting MODE 1 ===\n"));
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP + MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    Serial.println(F("UnibotsMarkII booting…"));

    // Motor pins
    pinMode(PIN_PWMA,  OUTPUT);
    pinMode(PIN_PWMB,  OUTPUT);
    pinMode(PIN_AIN,   OUTPUT);
    pinMode(PIN_BIN,   OUTPUT);
    pinMode(PIN_STBY,  OUTPUT);
    motorStop();

    // Spinner + actuator placeholder pins
    if (PIN_SPINNER_1   != 0) { pinMode(PIN_SPINNER_1,   OUTPUT); }
    if (PIN_SPINNER_2   != 0) { pinMode(PIN_SPINNER_2,   OUTPUT); }
    if (PIN_ACTUATOR_UP != 0) { pinMode(PIN_ACTUATOR_UP, OUTPUT); pinMode(PIN_ACTUATOR_DOWN, OUTPUT); actuatorStop(); }

    // Collision sensor
    if (PIN_COLLISION   != 0) { pinMode(PIN_COLLISION, INPUT_PULLUP); }

    // Gyro calibrate (keep robot still ~1 s)
    mpuInit();
    delay(200);
    Serial.println(F("Calibrating gyro — keep still…"));
    mpuCalibrate();
    Serial.println(F("Gyro ready."));
    delay(2000);    // time to place robot on floor

    // Startup: move forward 0.7 s, record in odometry
    Serial.println(F("Startup: moving forward 0.7 s"));
    driveForwardHold(STARTUP_FORWARD_MS);
    odomAddForward(STARTUP_FORWARD_MS);

    // Main match loop
    while (true) {
        runMode1();
        runMode2();
    }
}

void loop() {
    // All logic runs inside setup() while(true).
    // loop() is never reached; safety stop in case it is.
    motorStop();
}
