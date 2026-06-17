# ESP32 Implementation Plan — Unibots Robot Controller

## Overview

You are programming an ESP32-WROOM-32 using PlatformIO in Cursor. The ESP32 is the central controller of an autonomous robot for the Unibots UK 2025-2026 competition. It receives detection data from two Android phones over WiFi UDP and drives motors accordingly.

## What Already Exists (Android Side — Done)

Phone 1 runs a YOLO11 ball detection app. When the user selects "ball" task and taps "Stream", it sends a UDP packet every frame to `192.168.4.1:4210` containing detection data. The phone connects to the ESP32's WiFi access point manually (user joins the network in Android settings).

## Competition Context

- **Arena:** 2000mm x 2000mm, white floor, black walls ~150mm tall
- **Match duration:** 180 seconds
- **Balls:** 16 orange ping-pong balls (40mm) + 24 magnetic steel ball bearings (20mm)
- **Scoring:** 4 pts per ping-pong ball in own net, 2 pts per bearing in net, 1 pt per bearing held by robot, 3 pts for parking at scoring wall
- **Scoring nets:** Outside arena walls, 800mm wide, 200mm deep, positioned -100mm below wall top
- **Navigation aids:** AprilTags on walls (36h11, IDs 0-23), black line tracks on floor, colored scoring zones (Yellow/Orange/Purple/Green for N/E/S/W)
- **Robot must be fully autonomous** — no remote control during matches
- **Robot fits in 200mm cube initially, can extend to 300mm**
- **Physical start button required**

## Robot Mechanisms

- **Waterwheel pickup:** Always spinning, collects balls on contact — no trigger needed
- **Magnets:** Passively collect ball bearings as robot drives
- **Elevator:** Raises/lowers to dump balls into scoring net — ESP32 controls this
- **Differential drive:** Two motors for tank-style steering

---

## What the ESP32 Must Do

### 1. Create a WiFi Access Point

```
SSID: "RobotAP"
Password: "robot1234"
IP: 192.168.4.1 (ESP32 AP default)
```

Both phones connect to this AP manually.

### 2. Listen for UDP Packets

| Source | Port | Data |
|--------|------|------|
| Phone 1 (ball detection) | 4210 | Detection data every frame (see packet format below) |
| Phone 2 (bay counter) | 4211 | Future — ball count in the collection bay |

### 3. Send UDP Commands to Phone 1

| Destination | Port | Purpose |
|-------------|------|---------|
| Phone 1 | 4212 | Mode commands: "collect" or "return" (future — for switching Phone 1 between ball tracking and AprilTag navigation) |

The phone's IP can be captured from the source address of incoming packets on port 4210.

### 4. Parse Phone 1 Packets (Port 4210)

CSV format, newline-terminated:

```
seq,ts,fw,fh,n,has_primary,px,py,pw,ph,pprob,centered,err_x,err_y,ox,oy,ow,oh,oprob,...\n
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `seq` | uint32 | Frame sequence number (monotonic) |
| `ts` | uint32 | Timestamp in milliseconds |
| `fw` | int | Frame width in pixels |
| `fh` | int | Frame height in pixels |
| `n` | int | Number of detected target objects this frame |
| `has_primary` | 0/1 | Whether a primary (tracked) object exists |
| `px` | int | Primary object bounding box X (pixels) |
| `py` | int | Primary object bounding box Y (pixels) |
| `pw` | int | Primary object bounding box width |
| `ph` | int | Primary object bounding box height |
| `pprob` | int | Primary object confidence (0-100) |
| `centered` | 0/1 | Whether primary object center is within the center square |
| `err_x` | int | Horizontal error: primary center X minus frame center X. **Negative = ball is LEFT, Positive = ball is RIGHT.** This is the direct PID input for steering. |
| `err_y` | int | Vertical error: primary center Y minus frame center Y. **Negative = ball is ABOVE, Positive = ball is BELOW.** Proxy for distance (ball below center = closer). |
| Then repeated `n` times: | | |
| `ox,oy,ow,oh,oprob` | int | Each detected object's bounding box and confidence |

**Minimal parsing example (just the important fields):**

```cpp
int seq, ts, fw, fh, n, has_primary;
int px, py, pw, ph, pprob, centered, err_x, err_y;
sscanf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
       &seq, &ts, &fw, &fh, &n, &has_primary,
       &px, &py, &pw, &ph, &pprob, &centered, &err_x, &err_y);
```

**If `n == 0`:** No balls visible. Robot should spin in place to search.
**If `has_primary == 1`:** Use `err_x` for steering PID. Use `ph` (primary height in pixels) as a rough distance proxy — larger = closer.
**If `centered == 1`:** Ball is aligned. Drive forward.

---

## Robot State Machine

The ESP32 manages three routines. It is the central coordinator.

### Routine 1: COLLECT (default state)

```
Start → Spin to search (n==0)
       → Lock onto primary (has_primary==1)
       → PID steer using err_x to center the ball
       → When centered==1, drive forward
       → Waterwheel collects ball on contact
       → Repeat
```

**PID control for steering:**
- Input: `err_x` from Phone 1 (pixels, range roughly -320 to +320 for 640px wide frame)
- Output: differential motor speed (left_speed, right_speed)
- Negative err_x → ball is left → turn left (slow left motor, speed right motor)
- Positive err_x → ball is right → turn right (speed left motor, slow right motor)
- Start with P-only control (proportional), tune gains on the actual robot
- Placeholder: `Kp = 0.5` (adjust when you have the robot)

**Search behavior (n==0):**
- Spin in place (one motor forward, one reverse) until a ball appears

### Routine 2: DUMP (triggered by Phone 2 — future)

- Phone 2 reports bay has N balls → ESP32 switches to return mode
- ESP32 sends "return" command to Phone 1 (UDP port 4212) → Phone 1 switches to AprilTag navigation (future feature)
- Robot navigates to scoring net, aligns, raises elevator, dumps, lowers elevator
- Phone 2 reports bay is empty → ESP32 sends "collect" to Phone 1 → back to Routine 1
- **This cycle repeats throughout the match**

### Routine 3: FINAL RETURN (triggered by ESP32 timer)

- ESP32 starts a 180-second countdown when the physical start button is pressed
- At ~30 seconds remaining, ESP32 forces Routine 2 regardless of bay ball count
- After dumping, robot parks touching the scoring wall (3 bonus points)
- Robot stops completely

**The ESP32 owns the match timer because it cannot depend on phone network reliability for the most critical transition.**

---

## PlatformIO Project Setup

### `platformio.ini`

```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

### Required Libraries

None beyond the built-in ESP32 Arduino core (WiFi.h, WiFiUdp.h are included).

---

## Implementation Priority

### Phase 1: Communication (do this first — testable immediately)

1. Create WiFi AP
2. Listen on UDP port 4210
3. Parse incoming packets
4. Print parsed fields to Serial monitor
5. **Test:** Connect phone to AP, start streaming, verify data arrives on Serial monitor

### Phase 2: Motor Control (needs motors wired up)

1. Set up motor driver pins (PWM)
2. Implement `setMotorSpeeds(int left, int right)` function
3. Wire up PID steering using `err_x`
4. Implement search behavior (spin when n==0)
5. Implement forward drive when centered==1

### Phase 3: Match Timer + Start Button

1. Wire physical start button to a GPIO pin
2. On button press: start 180s countdown + begin Routine 1
3. At ~30s remaining: trigger Routine 3

### Phase 4: Elevator Control (needs elevator mechanism)

1. Control elevator servo/motor for raise/lower
2. Integrate into dump sequence

### Phase 5: Phone 2 Integration + Reverse UDP (future)

1. Listen on port 4211 for Phone 2 bay count
2. Send mode commands to Phone 1 on port 4212
3. Implement Routine 2 (dump cycle)

---

## Hardware Pin Assignments (TBD)

Fill these in once you have the wiring:

```cpp
// Motor driver pins
#define MOTOR_LEFT_PWM    __
#define MOTOR_LEFT_DIR    __
#define MOTOR_RIGHT_PWM   __
#define MOTOR_RIGHT_DIR   __

// Start button
#define START_BUTTON_PIN  __

// Elevator
#define ELEVATOR_PIN      __
```

---

## Key Constants

```cpp
const char* WIFI_SSID = "RobotAP";
const char* WIFI_PASS = "robot1234";
const int UDP_PORT_PHONE1 = 4210;
const int UDP_PORT_PHONE2 = 4211;
const int UDP_PORT_CMD = 4212;
const int MATCH_DURATION_MS = 180000;
const int FINAL_RETURN_MS = 30000; // trigger return with 30s remaining
```

---

## Notes

- UDP is fire-and-forget. Packets may be lost. The robot should always have a safe fallback (e.g., stop motors if no packet received for 500ms).
- The phone sends data every frame (~15-30 fps depending on model). You don't need to process every packet — just use the latest one.
- `err_x` and `err_y` are in pixels. Normalize by dividing by `fw/2` and `fh/2` if you want -1.0 to 1.0 range.
- The `seq` field lets you detect packet loss (gaps in sequence numbers) and stale data.
- All YOLO processing happens on the phone. The ESP32 just receives results and drives motors.
