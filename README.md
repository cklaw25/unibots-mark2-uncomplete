# UnibotsMarkII

Autonomous competition robot — Unibots UK 2025-2026.

The robot collects orange ping-pong balls from a 2×2m arena, then returns to its starting position and deposits them. A YOLO11 Android app detects balls through the camera and sends coordinates to the ESP32 over USB. The ESP32 drives the wheels using gyro heading-hold and returns home using dead-reckoning odometry.

---

# HOW TO FLASH EVERYTHING FROM SCRATCH

> Read this whole section before starting. It takes about 30-40 minutes the first time.

---

## What you need

### Hardware
- 2× ESP32-WROOM-32 boards (**NOT ESP32-S3** — must be classic WROOM-32)
- 1× Android phone (must support USB OTG)
- 1× USB OTG adapter or cable (phone → ESP32)
- 2× USB-A to micro-USB (or USB-C, matching your ESP32) cables for flashing
- The assembled robot with all wiring done

### Software — install these on your computer first

| Software | What it's for | Where to get it |
|----------|--------------|-----------------|
| VS Code | Code editor | https://code.visualstudio.com |
| PlatformIO (VS Code extension) | Flashes code to ESP32 | Install inside VS Code |
| Android Studio | Builds and installs the Android app | https://developer.android.com/studio |
| Git (optional) | Downloads the code | https://git-scm.com |

**After installing VS Code:**
1. Open VS Code
2. Click the Extensions icon on the left sidebar (looks like 4 squares)
3. Search for `PlatformIO IDE`
4. Click Install
5. Restart VS Code when prompted

**ESP32 USB driver (Windows only — do this if ESP32 doesn't show up as a COM port):**
- Download and install the CH340 driver: search "CH340 driver Windows" and install it
- Restart your computer after installing

---

## Step 1 — Get the code

**Option A — Git (recommended):**
Open a terminal and run:
```
git clone https://github.com/cklaw25/unibots-mark2-uncomplete.git
cd unibots-mark2-uncomplete
```

**Option B — Download ZIP:**
Go to https://github.com/cklaw25/unibots-mark2-uncomplete → click the green "Code" button → "Download ZIP" → extract it anywhere

---

## Step 2 — Flash Board A (the wheels ESP32)

Board A controls: 4 drive wheels + MPU6050 gyro + talks to the Android phone.

**2a. Plug in Board A**
- Connect Board A (wheels ESP32) to your computer with a USB cable
- Open Windows Device Manager (right-click Start → Device Manager → Ports)
- Look for something like **COM7** or **COM3** under "Ports (COM & LPT)"
- Note the COM number — you will need it

**2b. Check the COM port in the config file**
- Open the file `arduino/BoardA/platformio.ini`
- Find the line: `upload_port = COM7`
- Change `COM7` to whatever COM port you saw in Device Manager

**2c. Open the BoardA folder in VS Code**
- In VS Code: File → Open Folder → select `arduino/BoardA/`
- Wait for PlatformIO to finish loading (progress bar at the bottom)

**2d. Upload the code**
- At the very bottom of VS Code, find the toolbar with these icons:
  - **✓** (build) and **→** (upload/flash)
- Click the **→ Upload** button (right arrow)
- The terminal at the bottom will show the upload progress
- Wait until you see: `SUCCESS` or `Hard resetting via RTS pin...`
- Done — Board A is flashed!

> If you see `upload_port not found` — wrong COM port. Check Device Manager again.
> If you see `port busy` — close any Serial Monitor windows first, then try again.

---

## Step 3 — Test Board A motors and calibrate

> Do this before a real match. Wheels in wrong direction = robot drives backward.

**3a. Open Serial Monitor**
- In VS Code bottom toolbar, click the **plug icon** (Serial Monitor)
- Set baud rate to **115200**
- Power on Board A

**3b. Enter test mode**
- Serial Monitor will show: `Send 't' within 5 seconds to enter motor test mode...`
- Quickly type `t` and press Enter in the Serial Monitor input box
- You are now in test mode

**3c. Test each wheel (lift wheels off ground first!)**

Send these single letters one at a time:

| Send | What happens | What to check |
|------|-------------|---------------|
| `1` | TT1 front-right: fwd 1s → rev 1s | Wheel should spin away from robot (forward), then toward robot (reverse) |
| `2` | TT2 front-left: fwd 1s → rev 1s | Same check |
| `3` | TT3 back-right: fwd 1s → rev 1s | Same check |
| `4` | TT4 back-left: fwd 1s → rev 1s | Same check |
| `a` | All 4 forward 2s | All wheels should turn the same direction |

**If a wheel goes the wrong way:**
1. Open `arduino/BoardA/BoardA.ino`
2. Find the line for that wheel (e.g. `{ "TT2 front-left", 17, 18, 19, true }`)
3. Change `true` to `false` (or `false` to `true`) at the end
4. Save, re-upload (click → again), re-test

**3d. Calibrate distance (MS_PER_MM)**

This is critical — it controls how accurately the robot navigates home in Mode 2.

1. Place robot on flat floor, put a piece of tape at the front wheel as a start mark
2. In test mode, send `d`
3. Robot drives forward for exactly 5 seconds, then stops
4. Measure the distance from the tape mark to where the front wheel stopped (in mm)
5. Calculate: `MS_PER_MM = 5000 ÷ distance_in_mm`
   - Example: robot traveled 420mm → `MS_PER_MM = 5000 / 420 = 11.9`
6. Open `arduino/BoardA/BoardA.ino`, find `#define MS_PER_MM` near the top, update the number
7. Save and re-upload Board A

**3e. Start the match from test mode**
- Send `q` to exit test mode and start the normal match sequence

---

## Step 4 — Flash Board B (the spinner + lift ESP32)

Board B controls: N20 spinner wheel + N20 lift motor + listens for signal from Board A.

**4a. Plug in Board B**
- Unplug Board A, plug in Board B
- Check Device Manager for its COM port

**4b. Update the COM port**
- Open `arduino/BoardB/platformio.ini`
- Change `upload_port = COMX` to the COM port you found (e.g. `COM8`)

**4c. Open the BoardB folder and upload**
- In VS Code: File → Open Folder → select `arduino/BoardB/`
- Click **→ Upload**
- Wait for `SUCCESS`

---

## Step 5 — Test Board B and calibrate

**5a. Open Serial Monitor (115200 baud) with Board B powered on**

**5b. Calibrate spinner encoder (SPINNER_PULSES_REV)**

This tells the robot how many encoder pulses = one full spinner revolution. The robot uses this to rotate the spinner exactly 90° as a ball gate during deposit.

1. In Serial Monitor, send `m`
2. Serial Monitor shows: `Encoder monitor — spin N20 shaft by hand for 8s...`
3. Slowly rotate the N20 #1 spinner output shaft by hand (not the motor, the shaft coming out)
4. Make exactly **one full 360° revolution**, watch the "delta" number on screen
5. Note the delta number (e.g. `delta: 186`)
6. Open `arduino/BoardB/BoardB.ino`, find `#define SPINNER_PULSES_REV`, set it to that number
7. Re-upload Board B

> If delta stays at 0 while you spin the shaft: the encoder wires are on wrong pins.
> Check C1=GPIO32 and C2=GPIO33 on Board B.

**5c. Calibrate lift timing (LIFT_UP_MS / LIFT_DOWN_MS)**

1. In Serial Monitor, send `l`
2. Lift motor runs up, holds 1 second, comes back down
3. Watch carefully:
   - If the lift **doesn't fully extend**: increase `LIFT_UP_MS` (e.g. 2000 → 2500)
   - If the lift **stalls/jams before fully up**: decrease `LIFT_UP_MS`
   - Same logic for `LIFT_DOWN_MS` on the way down
4. Open `arduino/BoardB/BoardB.ino`, update the values, re-upload, test again

**5d. Check spinner direction**

Power on Board B while GPIO4 is LOW (Board A not connected yet, or Board B powered alone — GPIO4 will float LOW through internal pulldown). Spinner should spin in the direction that pulls balls INTO the robot.

If spinner pushes balls out instead:
- Open `arduino/BoardB/BoardB.ino`
- Find `#define SPINNER_INVERT false`
- Change to `#define SPINNER_INVERT true`
- Re-upload

---

## Step 6 — Install the Android app

**6a. Set up your phone**
1. On your phone: Settings → About Phone → tap **Build Number** 7 times rapidly
2. You will see "You are now a developer!"
3. Go back: Settings → Developer Options → turn on **USB Debugging**

**6b. Connect phone to computer**
- Use a regular USB cable (phone to computer, not OTG)
- A popup on the phone asks "Allow USB Debugging?" → tap **Always allow** → OK

**6c. Open Android Studio**
1. Open Android Studio
2. Click **Open** → navigate to the `android/` folder in this repo → click OK
3. Wait for Gradle sync to finish (progress bar at the bottom, can take 2-5 minutes)
4. If it asks to update Gradle or SDK: click **OK / Update**

**6d. Run the app on your phone**
1. At the top of Android Studio, find the device selector (dropdown next to the ▶ button)
2. Select your phone from the list
3. Click the green **▶ Run** button
4. App builds and installs automatically — it will open on the phone
5. Approve the camera permission when asked

**What you should see in the app:**
- Camera preview fills the screen
- Bounding boxes appear around orange balls when detected
- Top bar shows: **ESP32: Not connected** (red) until phone is plugged into Board A via OTG

---

## Step 7 — First full test

1. Connect phone to Board A via USB OTG cable (the short cable between phone and ESP32)
2. App status bar should change to **ESP32: Connected** (green) within 2 seconds
   - If it stays red: press the USB button in the app → approve any permission dialog → press again
3. Power on both Board A and Board B
4. Keep the robot completely still — gyro calibrates for 1 second at boot
5. Robot nudges forward, then Mode 1 begins
6. Point the phone camera at an orange ball to test detection

---

## What you will see in Serial Monitor during a match

**Board A — Mode 1:**
```
Board A booting...
Send 't' within 5 seconds to enter motor test mode...
Calibrating gyro — keep robot STILL for 1 second...
Gyro ready.
Startup: nudge forward...
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

**Board A — Mode 2:**
```
=== MODE 2 START ===
[M2] COMPUTE
  pos mm: (312.4, -88.1)
  dist:    324.6 mm
  bearing: 15.7 deg
[M2] NAVIGATE
[M2] AT_POSITION
  Arrived at home.
[M2] DEPOSIT
  Pulsing GPIO4 → Board B lift sequence...
  Waiting for Board B lift cycle...
[M2] BACKUP
=== MODE 2 END — restarting MODE 1 ===
```

**Board B — deposit sequence:**
```
Board B booting...
[B] Mode 1 at boot — starting spinner.
[B] Mode 2: closing gate...
[B] Gate closed. Holding.
[B] Deposit pulse — lifting.
[B] Lifting...
[B] Lowering...
[B] Deposit done.
[B] Mode 1 resuming: opening gate...
[B] Gate open. Resuming spinner.
```

---

## Tuning constants (what to change and when)

**Board A (`arduino/BoardA/BoardA.ino`):**

| Constant | What it does | When to change |
|----------|-------------|----------------|
| `MS_PER_MM` | Drive distance per millisecond | **Must calibrate** — see Step 3d |
| `DRIVE_SPEED` | Forward speed (0-255) | Robot too fast/slow |
| `TURN_SPEED` | Rotation speed | Turns too aggressive/slow |
| `KP` | Heading-hold strength | Robot drifts while driving straight |
| `GYRO_SIGN` | Gyro direction | Set to -1 if robot corrects the wrong way |
| `COAST_MS` | How long to drive after ball disappears from frame | Ball being missed on collection |
| `BALL_LOST_FRAMES` | How many no-ball frames before switching to coast | Approach aborts too early/late |

**Board B (`arduino/BoardB/BoardB.ino`):**

| Constant | What it does | When to change |
|----------|-------------|----------------|
| `SPINNER_PULSES_REV` | Encoder pulses per spinner revolution | **Must calibrate** — see Step 5b |
| `SPINNER_SPEED` | Spinner intake speed (0-255) | Balls not being pulled in |
| `LIFT_UP_MS` | Time for lift to fully extend | **Must calibrate** — see Step 5c |
| `LIFT_DOWN_MS` | Time for lift to fully retract | **Must calibrate** — see Step 5c |
| `SPINNER_INVERT` | Reverses spinner direction | Spinner pushes balls out instead of in |

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---------|-------------|-----|
| `upload_port not found` | Wrong COM port | Check Device Manager, update `platformio.ini` |
| `port busy` during upload | Serial Monitor is open | Close Serial Monitor, try again |
| ESP32 not found in Device Manager | Missing USB driver | Install CH340 driver, restart PC |
| Robot rotates in only one direction | Left or right side invert flags wrong | Test each wheel with `1`-`4` in test mode, fix invert flags |
| Robot misses home in Mode 2 | `MS_PER_MM` wrong | Recalibrate with `d` command |
| Spinner doesn't spin at boot | Board B not powered, or STBY not HIGH | Check 5V to Board B, check TB6612 #1 STBY pin wired to 3V3 |
| Spinner spins wrong way | `SPINNER_INVERT` wrong | Flip it in `BoardB.ino` |
| Lift doesn't fully extend | `LIFT_UP_MS` too small | Increase it and re-flash |
| Lift jams | `LIFT_UP_MS` too large (motor stalls) | Decrease it |
| Gate angle is wrong (90° off) | `SPINNER_PULSES_REV` wrong | Recalibrate with `m` command in Serial Monitor |
| App shows "ESP32: Not connected" | OTG not connected or permission denied | Replug OTG cable, press USB button in app, approve permission |
| No balls detected | Wrong task selected or camera covered | Check Task 5 is selected in app dropdown |
| All wheels dead | TB6612 STBY pins not wired HIGH | Check wiring — all STBY must be tied to 3.3V |
| Gyro drifts constantly | Robot moved during calibration | Power off, place on flat surface, power on and keep still for 1s |
| Mode 2 overshoots home | `MS_PER_MM` too low | Recalibrate — the number should be higher |
| Mode 2 stops short of home | `MS_PER_MM` too high | Recalibrate — the number should be lower |

---

## Hardware reference

### Board A pin map

| Motor | PWM | IN1 | IN2 | Invert | Driver chip |
|-------|-----|-----|-----|--------|-------------|
| TT1 front-right | GPIO13 | GPIO14 | GPIO16 | check on bench | TB6612 #2 Ch A |
| TT2 front-left  | GPIO17 | GPIO18 | GPIO19 | check on bench | TB6612 #2 Ch B |
| TT3 back-right  | GPIO23 | GPIO25 | GPIO26 | check on bench | TB6612 #3 Ch A |
| TT4 back-left   | GPIO27 | GPIO32 | GPIO33 | check on bench | TB6612 #3 Ch B |
| MPU6050 SDA | GPIO21 | | | | direct I2C |
| MPU6050 SCL | GPIO22 | | | | direct I2C |
| Trigger OUT to Board B | GPIO4 | | | | single wire |

### Board B pin map

| Component | PWM | IN1 | IN2 | Encoder C1 | Encoder C2 | Driver chip |
|-----------|-----|-----|-----|-----------|-----------|-------------|
| N20 #1 spinner | GPIO13 | GPIO14 | GPIO27 | GPIO32 | GPIO33 | TB6612 #1 Ch A |
| N20 #2 lift    | GPIO16 | GPIO17 | GPIO18 | (not used) | (not used) | TB6612 #1 Ch B |
| Trigger IN from Board A | GPIO4 | | | | | |

### GPIO4 signal between boards

| State | Meaning | Board B response |
|-------|---------|-----------------|
| LOW (sustained) | Mode 1 active | Spinner runs continuously |
| LOW → HIGH | Mode 2 started | Stop spinner, rotate 90° gate, hold |
| HIGH → LOW (200ms) → HIGH | Deposit command | Lift up, hold, lower |
| HIGH → LOW (sustained) | Mode 1 resuming | Rotate gate back, resume spinner |

---

## Code reference

| File | What it is |
|------|-----------|
| `arduino/BoardA/BoardA.ino` | Board A match firmware (wheels + gyro + phone comms) |
| `arduino/BoardA/platformio.ini` | PlatformIO config for Board A — update COM port here |
| `arduino/BoardB/BoardB.ino` | Board B match firmware (spinner + lift + encoder) |
| `arduino/BoardB/platformio.ini` | PlatformIO config for Board B — update COM port here |
| `android/` | Android YOLO11 ball-detection app |
| `simulation/` | Python dead-reckoning simulator (no hardware needed, for testing nav logic) |
| `CONVERSATION_LOG.md` | Full development session history — read this for background context |
