# Changes vs Old Project (mark 4 progress / android_dylan_rough original)

## What the old project had
- Basic YOLO11 ball detection (ncnn on Android)
- WiFi UDP as the only transport (phone connects to ESP32 AP)
- Continuous rotation (duty-cycle) to search for ball
- Simple FSM: ROTATING → MOVING_FORWARD → COASTING
- No AprilTag support
- No delivery mechanism

---

## New additions in this version

### Transport layer
- **USB serial added as primary path** (OTG USB-C to USB-C cable, 115200 baud)
  - WiFi UDP kept as fallback, both listened to simultaneously
  - USB is far more stable and doesn't require phone to join WiFi network

### Part 1 — Ball Tracking overhaul
- **Step-scan rotation replaces continuous rotation**
  - Old: spin continuously on duty cycle, hard to tune
  - New: rotate ~45° → pause 1500ms → look for ball → repeat
  - Much more reliable detection during pause (no motion blur)
- **Alignment pulses**: short 200ms rotation toward ball if off-centre, then re-pause
- **On-screen FSM mirror**: phone screen shows ROTATING / MOVING FORWARD / COASTING + LED flash indicator
- **Part 1 countdown timer** on screen (P1 TIME: Xs remaining)
- **Spinner motor** integrated: fires CW 300ms → stop → CCW 300ms after each coast cycle

### Part 2 — AprilTag navigation (entirely new)
- **AprilTag C library** integrated via CMake FetchContent
- **Back camera** used for AprilTag (sharper, more reliable than front camera)
- **Full 11-state FSM** on Android side (see TECHNICAL_OVERVIEW.txt)
- Tag-to-tag navigation: approach non-target tags → reorient toward Tag 1 → approach → deliver
- **Delivery sequence**: 180° turn → reverse 2s → lift up 3s → lift down 1s → forward 2s → done
- **Immediate Part 1 revert** after delivery (g_request_part1_switch flag + JNI poll)

### Auto-cycling (entirely new)
- Java timer: Part 1 runs 90s → auto-switch to Part 2
- Part 2 runs 90s → auto-switch to Part 1 (or immediately on delivery)
- `pollDeliveryComplete` Runnable polls every 500ms for early exit from Part 2

### ESP32 firmware additions
- `g_part2_active` flag: distinguishes CMD packets from ball packets
- `execute_p2_command()`: handles all 7 command types
- `stop_all_motors()` now also zeros spinner pins
- `move_backward()`, `set_rotate_ccw()` added
- Lift motor pins (GPIO 19, 18) wired and controlled
- Spinner motor (GPIO 15, 21) with full sequence
- Step-scan FSM (STATE_STEP_ROTATE, STATE_SCAN_PAUSE, STATE_ALIGN_ROTATE) replacing continuous rotation

### Bug fixes
- **Rotation direction was inverted** — alignment logic fixed so err_x > 0 (ball right) → rotate CCW (physically correct). Applies to both Part 1 and Part 2.
- **LEDC PWM removed** — caused broken rotation, replaced with digitalWrite + software duty cycle
- **Task 6 frame downscale removed** — corrupted display, reverted to full 1280×720

### Build system changes
- `gradle.properties`: `android.useAndroidX=true` added
- `app/build.gradle`: `usb-serial-for-android:3.7.0` dependency added
- `settings.gradle`: JitPack repository added
- `CMakeLists.txt`: FetchContent for apriltag library, cmake_minimum bumped to 3.11

---

## What is NOT changed / still the same as original
- YOLO11 ncnn model and inference pipeline
- Camera capture via NDK (ndkcamera.cpp) — still 1280×720
- Basic bounding box drawing and UI layout
- Motor wiring polarity assumptions (move_forward uses `false`)
