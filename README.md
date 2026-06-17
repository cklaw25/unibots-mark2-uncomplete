# UnibotsMarkII

Autonomous competition robot for Unibots UK 2025-2026.

The robot collects orange ping-pong balls scattered on a 2×2m arena, then autonomously returns to its starting net and deposits them. It uses a YOLO11 Android app for ball detection and dead-reckoning odometry (gyro + timed drive) for the return path.

---

## How it works

**Mode 1 — Ball collection (60 s)**
Robot scans by rotating in bursts, locks onto a ball detected by the Android camera, drives toward it with the spinner wheel collecting, then resumes scanning. Repeats until 60 s is up.

**Mode 2 — Return and deposit**
Uses dead-reckoning odometry (accumulated x/y position from gyro heading + timed drive distance) to compute a bearing and distance back to the start, drives home, then triggers the stepper motor to deposit the balls. Resets and restarts Mode 1.

---

## Hardware

| Component | Details |
|-----------|---------|
| ESP32-D0WD-V3 (WROOM-32) | Brain — NOT an S3, classic dual-core |
| Android phone | YOLO11 ball detection, sends packets to ESP32 via USB OTG |
| 4× TT DC gear motors | Drive wheels (front-right, front-left, back-right, back-left) |
| 3× TB6612FNG | Motor drivers — STBY hardwired to 3.3V, always enabled, not a GPIO |
| N20 DC motor w/ encoder | Spinner wheel for ball collection |
| MPU6050 | Gyroscope/IMU — I2C on SDA=21, SCL=22 |
| Stepper motor | Ball deposit mechanism |

## Pin map (confirmed 2026-06-16)

| Motor | PWM | IN1 | IN2 | Inverted |
|-------|-----|-----|-----|----------|
| TT1 front-right | GPIO14 | GPIO13 | GPIO23 | no |
| TT2 front-left | GPIO16 | GPIO17 | GPIO5 | yes |
| TT3 back-right | GPIO4 | GPIO15 | GPIO2 | no |
| TT4 back-left | GPIO12 | GPIO18 | GPIO19 | yes |
| N20 spinner | GPIO25 | GPIO27 | GPIO26 | no |
| MPU6050 SDA | — | GPIO21 | — | — |
| MPU6050 SCL | — | GPIO22 | — | — |

N20 encoder: C1=GPIO32, C2=GPIO33 (best guess from wiring diagram — unconfirmed).

Stepper motor pins: TBD — fill into `arduino/UnibotsMainV2/UnibotsMainV2.ino` before use.

---

## Code

| Path | Purpose |
|------|---------|
| `arduino/UnibotsMainV2/UnibotsMainV2.ino` | **Main match firmware — flash this to ESP32** |
| `arduino/UnibotsMainV2/platformio.ini` | PlatformIO config (board=esp32dev, COM7) |
| `arduino/HardwareTest/HardwareTest.ino` | Motor bench-test tool (serial menu, not for match use) |
| `android/` | Android YOLO11 ball-detection app — build and install on phone |
| `simulation/` | Python dead-reckoning simulator (no hardware needed) |

---

## How to use the code

### UnibotsMainV2 — match firmware

This is fully autonomous once flashed. There are no commands to send — the robot runs itself.

**What happens at power-on:**
1. Serial Monitor prints `UnibotsMarkII v2 booting...`
2. Gyro calibrates for ~1 s — keep the robot completely still during this
3. `Gyro ready.` is printed — you have 2 s to place the robot on the floor
4. Robot nudges forward 0.7 s (startup move), then Mode 1 begins automatically

**Reading the Serial Monitor output (115200 baud):**

During Mode 1 you will see state transitions as the robot runs:
```
=== MODE 1 START ===
[M1] SCAN_STEP
[M1] SCAN_PAUSE
[M1] ALIGN
[M1] APPROACH
[M1] COAST
[M1] SCAN_STEP
...
=== MODE 1 END — switching to MODE 2 ===
```

During Mode 2:
```
=== MODE 2 START ===
[M2] COMPUTE
  pos: (312.4, -88.1) mm
  dist: 324.6 mm
  bearing: 15.7 deg
[M2] NAVIGATE
[M2] AT_POSITION
  Arrived at home position.
[M2] DEPOSIT
[M2] RETRACT
[M2] BACKUP
=== MODE 2 END — restarting MODE 1 ===
```

**Tuning constants in the file (top of `UnibotsMainV2.ino`):**

| Constant | What it controls | When to change |
|----------|-----------------|----------------|
| `MS_PER_MM` | Drive distance calibration | After first flash — measure 500mm run |
| `DRIVE_SPEED` | Forward PWM (0-255) | If robot is too fast/slow |
| `TURN_SPEED` | Rotation PWM | If turns are too aggressive |
| `KP` | Heading-hold strength | If robot drifts or oscillates while driving |
| `GYRO_SIGN` | Gyro direction | Flip to -1 if heading corrections go the wrong way |
| `COAST_MS` | How long to drive after ball leaves frame | If balls are being missed |
| `BALL_LOST_FRAMES` | Frames without ball before APPROACH → COAST | If robot aborts approach too early/late |
| `STEPPER_STEPS_EXTEND` | How far stepper extends for deposit | Tune to your mechanism travel distance |

---

### HardwareTest — bench test tool

Flash this **instead of** UnibotsMainV2 when you want to test individual motors on the bench. Do not use it during a match.

Flash it from `arduino/HardwareTest/`:
```
cd C:\unibotsmark2\arduino\HardwareTest
pio run -t upload --upload-port COM7
```

Open Serial Monitor at **115200 baud**. You will see a menu. Send a single character (no newline needed):

| Key | What it does |
|-----|-------------|
| `1` | TT1 front-right — forward 600ms, stop, reverse 600ms, stop |
| `2` | TT2 front-left — same sequence |
| `3` | TT3 back-right — same sequence |
| `4` | TT4 back-left — same sequence (**Serial goes quiet** during this test — normal, TX/RX pins conflict) |
| `5` | N20 spinner — same sequence, also prints encoder pulse delta |
| `a` | All 4 TT wheels forward together 1.2 s — use this to confirm straight-line drive |
| `m` | Encoder monitor — spin N20 shaft by hand for 8 s, watch pulse count to confirm GPIO32/33 are correct |
| `p` | Pin probe — holds each TT1/TT2 input pin HIGH 3 s so you can confirm continuity with a multimeter |
| `o` | Output terminal test — drives TT1 then TT2 for 6 s each so you can measure voltage at screw terminals |
| `s` | Stop everything immediately |

**Recommended test order when bringing up the hardware for the first time:**
1. `1` through `4` individually — confirm each wheel spins the right direction (forward = away from starting wall)
2. `a` — confirm all 4 go forward together and robot rolls straight
3. `5` — confirm N20 spinner runs
4. `m` — confirm N20 encoder pulses (if zero, GPIO32/33 are wrong — update `N20_ENC_C1/C2` defines)

---

## REMINDERS — read before flashing

These are open hardware/config items. Check each one before proceeding.

**1. TT4 wiring — MUST CHECK**
TT4's direction pins were originally on GPIO1 (TX) and GPIO3 (RX), which conflict with USB serial (Android packets). They must be re-routed:
- TT4 IN1: move wire from GPIO1 → GPIO18
- TT4 IN2: move wire from GPIO3 → GPIO19

The firmware already expects GPIO18/19. If this re-wire has not been done, the robot will lose Android ball packets every time TT4 drives.

**2. Stepper motor pins — MUST FILL IN**
Open `arduino/UnibotsMainV2/UnibotsMainV2.ino` and fill in:
```cpp
#define STEPPER_STEP_PIN     0   // ← real GPIO
#define STEPPER_DIR_PIN      0   // ← real GPIO
#define STEPPER_EN_PIN       0   // ← real GPIO (or leave 0 if no enable pin)
#define STEPPER_STEPS_EXTEND 200 // ← steps to fully extend mechanism
```
Driver assumed: A4988 or DRV8825 (STEP + DIR + EN-active-LOW).
If using a different driver type, the `stepperStep()` function will need updating.

**3. MS_PER_MM calibration — do after first flash**
The return-path distance is entirely based on this constant. Wrong value = robot overshoots or stops short.
```cpp
#define MS_PER_MM  3.5f   // ← replace with: drive robot 500mm, measure ms, set = ms/500
```

**4. N20 encoder pins — unconfirmed**
GPIO32/33 are a best guess from the hand-drawn wiring diagram. Use `HardwareTest.ino` and the `m` command (spin shaft by hand, watch pulse count) to verify. If zero pulses, the pins are wrong — only `N20_ENC_C1` and `N20_ENC_C2` in the firmware need changing.

---

## Instructions for Claude

You are being given this README to help bring up this robot. Here is the workflow:

**Step 1 — Check the reminders above.**
Go through reminders 1-4 with the user. Ask which ones have been done and which are still outstanding. Do not proceed to flashing until reminder 1 (TT4 re-wire) and reminder 2 (stepper pins) are either fixed or the user explicitly says to ignore them.

**Step 2 — Flash the ESP32.**
Once reminders are addressed:
```
cd C:\unibotsmark2\arduino\UnibotsMainV2
pio run -t upload --upload-port COM7
```
Close any Serial Monitor or `pio device monitor` before running — COM7 busy will cause upload failure.

**Step 3 — Build and install the Android app.**
```
cd C:\unibotsmark2\android
.\gradlew assembleDebug
adb install app\build\outputs\apk\debug\app-debug.apk
```
Phone must be connected via USB with USB debugging enabled. If `adb` is not found, it is in the Android SDK platform-tools folder — ask the user for the path or add it to PATH.

**Step 4 — Run.**
- Open the installed app on the phone (`yolo11ncnn`)
- Connect phone to ESP32 via USB OTG cable
- Power the ESP32
- Keep robot still for ~1 s while gyro calibrates (Serial Monitor will show "Gyro ready.")
- Robot starts automatically after a 2 s delay
