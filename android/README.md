# Unibots Mark 2 — Ping Pong Robot

An autonomous robot for the Unibots UK 2025-2026 competition.

The robot uses an Android phone as its "brain" (computer vision) and an ESP32 as its "body" (motor control). It finds ping-pong balls, launches them, navigates to a scoring net using wall markers, and deposits the balls — all on its own.

---

## If you are completely new to this project, read this first

**What is this repo?**
This is the Android app code + ESP32 firmware for a competition robot. The phone runs AI vision (YOLO11 for ball detection, AprilTag library for navigation). It sends commands to an ESP32 over a USB cable. The ESP32 drives the motors.

**Two modes:**
- **Part 1 (90 seconds):** Robot scans for orange ping-pong balls. When it finds one, it drives toward it and launches it with a spinning wheel.
- **Part 2 (90 seconds):** Robot looks for AprilTag markers on the arena walls, navigates to the scoring net (Tag ID 1), backs up to it, lifts a mechanism to drop the balls in, then returns to Part 1.

The two modes cycle automatically, every 90 seconds.

**Where everything is:**

| Thing | Location |
|-------|----------|
| Android app source | `app/src/main/jni/yolo11ncnn.cpp` (C++ vision logic) |
| Android Java UI | `app/src/main/java/.../MainActivity.java` |
| ESP32 firmware | `esp32_main.cpp` in this repo |
| Technical deep-dive | `TECHNICAL_OVERVIEW.txt` |
| What changed vs old version | `CHANGES_FROM_OLD_PROJECT.md` |
| Full session history & handoff | `CONVERSATION_LOG.md` |
| Detailed dev notes | `NOTES.md` |

---

## Current Progress (as of 2026-06-15)

| Feature | Status |
|---------|--------|
| Ball detection (YOLO11 on phone) | Working |
| USB cable link phone ↔ ESP32 | Working |
| LED flashes x3 when ball is centred | Working |
| Robot drives forward to ball | Working |
| Robot coasts 2.5s after losing ball | Working |
| Step-scan rotation (45° → pause → look) | Working |
| Alignment toward ball | Working |
| Find AprilTag and drive toward it | Working |
| Auto-switch Part 1 ↔ Part 2 every 90s | Working |
| Spinner after ball launch cycle | **NOT YET TESTED** |
| Full tag-to-tag navigation (Part 2) | **NOT YET TESTED** |
| Full delivery sequence | **NOT YET TESTED** |

**Next session priorities:**
1. Test spinner timing (tune `SPINNER_DEG120_MS`)
2. Test full Part 2 tag-to-tag path end-to-end
3. Test delivery sequence (turn 180° → reverse → lift → forward → switch to Part 1)

---

## Setup — How to get this running on a new machine

### Step 1 — Clone this repo
```bash
git clone https://github.com/<your-username>/unibots-mark2-uncomplete.git
```

### Step 2 — Download large binary dependencies (not in git)

These are too big to store in git. Download and extract them into `app/src/main/jni/`:

- **ncnn library:** https://github.com/Tencent/ncnn/releases
  - File: `ncnn-20260113-android-vulkan.zip`
  - Extract so the folder is: `app/src/main/jni/ncnn-20260113-android-vulkan/`

- **OpenCV mobile:** https://github.com/nihui/opencv-mobile
  - File: `opencv-mobile-4.13.0-android.zip`
  - Extract so the folder is: `app/src/main/jni/opencv-mobile-4.13.0-android/`

### Step 3 — Open in Android Studio
- Install Android Studio (with NDK support)
- Open the project root folder
- Let Gradle sync — it will auto-detect everything
- `local.properties` will be generated automatically with your SDK path

### Step 4 — Build and install the app
Plug your phone into your PC via USB with ADB authorized (not the OTG cable — the regular data cable).
```bash
./gradlew installDebug
```

### Step 5 — Set up the ESP32 firmware
- Install VS Code + PlatformIO extension
- Create a new PlatformIO project somewhere (e.g. `C:\esp32\`)
  - Board: `esp32dev`, Framework: `arduino`
- Copy `esp32_main.cpp` from this repo into `C:\esp32\src\main.cpp`
- Connect ESP32 via USB and flash:
```bash
cd C:\esp32
pio run --target upload
```

### Step 6 — Run the robot
1. Disconnect ADB cable from phone
2. Connect phone to ESP32 via **USB OTG cable** (USB-C to USB-C)
3. Open the app on the phone
4. Select **Task 5** (ball tracking / Part 1)
5. Point **front camera** at arena — robot starts scanning
6. After 90 seconds, app auto-switches to Task 6 (Part 2)
7. For Part 2, **manually switch to back camera** in the app (back camera detects AprilTags better)

---

## Hardware Overview

```
Phone ──[USB OTG]──> ESP32 ──> L298N #1 ──> Back Left + Front Left motors
                         └──> L298N #2 ──> Back Right + Front Right motors
                         └──> L298N #3 ──> Lift UP / Lift DOWN motor
                         └──> Spinner motor (GPIO 15, 21)
```

**Power:** Two battery packs in series (~24V) → LM2596 buck converter → ESP32 VIN

**Motor pin mapping:**

| GPIO | Board   | Motor |
|------|---------|-------|
| 26   | #1 IN1  | Back Left |
| 25   | #1 IN2  | Back Left |
| 33   | #1 IN3  | Front Left |
| 32   | #1 IN4  | Front Left |
| 14   | #2 IN1  | Back Right |
| 27   | #2 IN2  | Back Right |
| 23   | #2 IN3  | Front Right |
| 22   | #2 IN4  | Front Right |
| 19   | #3 IN1  | Lift UP |
| 18   | #3 IN2  | Lift DOWN |
| 15   | IN3     | Spinner |
| 21   | IN4     | Spinner |

---

## Important Reminders (read before touching the code)

**1. Camera is switched manually.**
Front camera for Part 1 (ball tracking). Back camera for Part 2 (AprilTags). You must tap to switch in the app. There is no auto-switch yet.

**2. Rotation direction is intentionally "backwards".**
`err_x > 0` means the ball/tag is to the RIGHT of the screen → robot rotates CCW.
`err_x < 0` means the ball/tag is to the LEFT → robot rotates CW.
This is physically correct and was verified on real hardware. Do not "fix" it.

**3. Do NOT use LEDC/PWM for motors.**
It broke rotation. All motor control uses `digitalWrite` + software delays. Don't add analogWrite or LEDC.

**4. Do NOT resize the camera frame in draw_apriltag_mode().**
A previous attempt to downscale to save processing time corrupted the display. The camera runs at 1280×720 for both tasks. Leave it alone.

**5. WiFi UDP still works as fallback.**
ESP32 AP: `PingPong_ESP32` / password: `12345678`, IP `192.168.4.1`, port 4210.
But USB serial is faster and more reliable — use that.

---

## Tuning Constants (things to adjust on physical hardware)

All of these need real-world testing to dial in:

| Constant | File | Current Value | What it controls |
|----------|------|---------------|-----------------|
| `ROTATE_STEP_MS` | `main.cpp` | 400ms | One rotation step (~45°). Increase if undershoot. |
| `SCAN_PAUSE_MS` | `main.cpp` | 1500ms | Pause between rotation steps. |
| `ALIGN_STEP_MS` | `main.cpp` | 200ms | One alignment pulse toward ball. |
| `SPINNER_DEG120_MS` | `main.cpp` | 300ms | Spinner 120° rotation time. |
| `SCAN_STEP_MS` | `yolo11ncnn.cpp` | 400ms | Part 2 scan step duration. |
| `TURN_180_MS` | `yolo11ncnn.cpp` | 1500ms | 180° turn before delivery. |
| `CLOSE_ENOUGH` | `yolo11ncnn.cpp` | 150px | Stop distance for non-target tags. |
| `TARGET_RANGE_PX` | `yolo11ncnn.cpp` | 70px | Stop distance for delivery (Tag 1). |

---

## WiFi Credentials (ESP32 Access Point)
- SSID: `PingPong_ESP32`
- Password: `12345678`
- IP: `192.168.4.1`
- UDP Port: `4210`

---

## Old repo / git history reference
Old repo (before this clean start): `DannyTWizard/android_dylan_rough`
Last stable checkpoint tag: `checkpoint-21-march` (2026-03-21)
