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
