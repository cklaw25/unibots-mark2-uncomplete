# Session Conversation Log & Project Handoff
Last saved: 2026-06-15

This file is a human-readable summary of the full development session so far.
It is intended to allow someone picking this up on a new machine to understand
exactly where we left off, what decisions were made and why, and what to do next.

---

## What this project is

A robot for the Unibots UK 2025-2026 competition. The robot autonomously:
1. Finds and launches ping-pong balls using YOLO11 ball detection (Part 1)
2. Navigates to the scoring net using AprilTag wall markers (Part 2)
3. Lifts and deposits the balls, then repeats

Full technical details: see TECHNICAL_OVERVIEW.txt
All code changes vs the old version: see CHANGES_FROM_OLD_PROJECT.md

---

## Where the code lives

| Component        | Path |
|-----------------|------|
| Android app     | `C:\mark 4 ping pong\android_dylan_rough\` (this repo) |
| ESP32 firmware  | `C:\esp32\src\main.cpp` (PlatformIO project) |
| ESP32 backup    | `esp32_main.cpp` in this repo (keep in sync!) |

---

## Development session summary

### Starting point
- Base project was `android_dylan_rough` (nihui ncnn-android-yolo11 fork)
- Had basic ball detection + WiFi UDP to ESP32
- ESP32 had simple rotating FSM

### Session 1 — USB serial + Part 1 overhaul
- Switched primary transport from WiFi UDP to USB OTG serial (115200 baud)
- Replaced continuous duty-cycle rotation with step-scan (45° rotate → 1500ms pause → look)
- Added alignment pulse (200ms rotate toward ball, then re-pause)
- Added spinner motor (GPIO 15, 21): fires CW→stop→CCW after each coast cycle
- Fixed rotation direction bug: err_x > 0 (ball on RIGHT) must rotate CCW, not CW
- Added on-screen FSM mirror and countdown timer

### Session 2 — Part 2 AprilTag navigation
- Integrated apriltag C library via CMake FetchContent
- Switched Part 2 to back camera (front camera can't reliably detect AprilTags)
- Built full 11-state FSM on Android side (scan → align → approach → deliver → exit)
- Added delivery sequence (180° turn, reverse, lift up, lift down, forward exit)
- Added g_request_part1_switch flag + JNI poll so Java switches to Part 1 immediately on delivery
- Added auto-cycle Java timers (90s each phase)

### Session 3 — Integration + bug fixes
- Fixed downscale bug in draw_apriltag_mode() (resizing corrupted display — reverted)
- Added step-scan to Part 2 scan state (was previously continuous rotation)
- Verified and documented rotation direction for both parts
- All motor pins confirmed and documented

---

## Current progress checkpoints

| What                              | Status |
|-----------------------------------|--------|
| Ball detection (YOLO11)           | WORKING ✓ |
| USB serial link                   | WORKING ✓ |
| LED flash x3 on ball centred      | WORKING ✓ |
| Forward drive when ball centred   | WORKING ✓ |
| Coast 2500ms after ball leaves    | WORKING ✓ |
| Step-scan rotation (Part 1)       | WORKING ✓ |
| Alignment direction fix           | WORKING ✓ (both parts) |
| Part 2: find tag + drive to it    | WORKING ✓ |
| Auto-switch Part 1 → Part 2 timer | WORKING ✓ |
| Spinner after ball tracking cycle | NOT YET TESTED |
| Full tag-to-tag navigation        | NOT YET TESTED |
| Full delivery sequence            | NOT YET TESTED |
|   (Tag 1 → 180° → reverse →      |                |
|    lift → forward → Part 1)       |                |

---

## What to do when you pick this up on a new machine

### 1. Clone this repo
```
git clone https://github.com/<your-username>/unibots-mark2-uncomplete.git
cd unibots-mark2-uncomplete
```

### 2. Download binary dependencies (NOT in git — too large)
- ncnn library: https://github.com/Tencent/ncnn/releases
  → download `ncnn-20260113-android-vulkan.zip`
  → extract into `app/src/main/jni/`
- OpenCV mobile: https://github.com/nihui/opencv-mobile
  → download `opencv-mobile-4.13.0-android.zip`
  → extract into `app/src/main/jni/`

### 3. Set up Android Studio
- Open the project root in Android Studio
- SDK/NDK will be detected automatically
- `local.properties` will be auto-generated with your SDK path

### 4. Build and install Android app
```bash
cd "path/to/this/repo"
./gradlew installDebug
```
Phone must be plugged in via USB with ADB authorized.
Disconnect OTG cable before building (ADB and OTG share the USB port).

### 5. Set up ESP32 (PlatformIO)
- Install VS Code + PlatformIO extension
- Create a PlatformIO project at `C:\esp32\` (or wherever)
- Copy `esp32_main.cpp` from this repo to `C:\esp32\src\main.cpp`
- Flash:
```bash
cd C:\esp32
pio run --target upload
```

### 6. Run the robot
- Connect phone to ESP32 via USB OTG cable (USB-C to USB-C)
- Open the app → select Task 5 (ball tracking)
- Point front camera at the arena
- Robot will auto-cycle between Part 1 and Part 2 every 90 seconds
- For Part 2, switch to back camera manually

---

## Critical reminders for next session

1. **Camera is manual** — front for Part 1 (ball), back for Part 2 (AprilTags). This must be switched manually in the app. No auto-switch exists yet.

2. **Rotation direction is INVERTED** — err_x > 0 (object on RIGHT of screen) → rotate CCW. This is physically correct and verified. Do NOT change it.

3. **No LEDC/PWM on motors** — LEDC broke rotation. All motor control is digitalWrite + software delay. Do not reintroduce PWM.

4. **Do not resize frame in draw_apriltag_mode()** — corrupts display. Camera is 1280×720 for both tasks.

5. **First things to test on next session:**
   - Spinner sequence (fires after coast — check CW/CCW direction and timing)
   - Full Part 2 tag-to-tag path (scan → align → approach non-target → find Tag 1 → approach → deliver)
   - Full delivery sequence (180° turn, reverse, lift, forward exit, return to Part 1)

6. **Constants to tune physically:**
   - `ROTATE_STEP_MS = 400` — adjust until each step is ~45 degrees
   - `SCAN_PAUSE_MS = 1500` — can reduce if robot is scanning too slowly
   - `TURN_180_MS = 1500` — adjust until robot actually turns 180°
   - `CLOSE_ENOUGH = 150px` — adjust based on actual tag size at stop distance
   - `TARGET_RANGE_PX = 70px` — adjust for correct delivery distance
   - `SPINNER_DEG120_MS = 300` — adjust for actual 120° spinner rotation

---

## Git checkpoints in old repo (DannyTWizard/android_dylan_rough)

| Tag | Description |
|-----|-------------|
| checkpoint-night-before | Stable pre-Part2 state |
| checkpoint-mark4 | Mid-development snapshot |
| checkpoint-21-march | 2026-03-21 — last stable state before this new repo |

ESP32 backups:
- `C:\esp32\main_checkpoint_night_before.cpp.bak`
- `C:\esp32\main_checkpoint_21_march.cpp.bak`
