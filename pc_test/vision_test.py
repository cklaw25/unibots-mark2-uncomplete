"""
UnibotsMarkII — PC Vision Test
Simulates the full Mode 1 / Mode 2 FSM using a webcam + YOLOv8.

No real robot needed — motor commands shown as on-screen overlays.
Odometry is simulated based on time spent in each state.

Controls:
    Q  — quit
    R  — reset (restart Mode 1 timer)
    +  — increase target class ID
    -  — decrease target class ID (try 0=person, 32=sports ball, 39=bottle)
"""

import cv2
import time
import math
import sys
import os
import numpy as np
from ultralytics import YOLO

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "simulation"))
from odometry import Odometry, FORWARD_SPEED_MM_S, ROTATE_SPEED_DEG_S

# ─── Config ───────────────────────────────────────────────────────────────────
MODEL_PATH      = r"C:\Users\lawch\yolov8n.pt"
# Same filter as Mark 4: sports ball=32, apple=47, orange=49
TARGET_CLASSES  = [32, 47, 49]
CONF_THRESHOLD  = 0.30
CAM_INDEX       = 0

MODE1_DURATION_S = 60.0
STARTUP_FWD_S    = 0.7
COAST_S          = 2.0
SCAN_STEP_S      = 0.4
SCAN_PAUSE_S     = 1.5
BACKUP_S         = 0.7
ACTUATOR_HOLD_S  = 1.0

# Centre rectangle size (fraction of frame)
CRECT_W_FRAC = 0.20   # 20% of frame width — narrow column down full height

# How close ball cx must be to frame centre to count as "centred" (fraction)
ALIGN_THRESH_FRAC = 0.06

# Frames with no detection before switching APPROACH → COAST
BALL_LOST_FRAMES = 4

# Colours
COL_GREEN   = (50,  230,  80)
COL_ORANGE  = (0,   165, 255)
COL_YELLOW  = (0,   220, 220)
COL_RED     = (60,   60, 220)
COL_GREY    = (160, 160, 160)
COL_WHITE   = (240, 240, 240)
COL_CYAN    = (220, 220,   0)

COCO_NAMES = {
    0:"person", 32:"sports ball", 39:"bottle", 46:"banana",
    47:"apple",  49:"orange",     63:"laptop",  67:"cell phone"
}

# ─── FSM ──────────────────────────────────────────────────────────────────────
class RobotFSM:
    """
    Full Mode 1 / Mode 2 FSM with simulated odometry.
    Does NOT drive real motors — shows what the robot would do.
    """

    def __init__(self, frame_w: int, frame_h: int):
        self.fw = frame_w
        self.fh = frame_h
        self._reset()

    # ── centre rectangle (stays fixed) ────────────────────────────────────────
    def _crect(self):
        cx = self.fw // 2
        rw = int(self.fw * CRECT_W_FRAC)
        return cx - rw // 2, 0, cx + rw // 2, self.fh   # full window height

    def _reset(self):
        self.mode        = 1
        self.state       = "STARTUP"
        self.state_t     = time.time()
        self.mode1_t     = time.time()
        self.odom        = Odometry(add_noise=False)
        self.last_odom_t = time.time()
        self.path_wander : list[tuple[float,float]] = [(0.0, 0.0)]
        self.path_return : list[tuple[float,float]] = []

        self.target_box    = None   # (x1,y1,x2,y2) of locked target
        self.no_ball_count = 0

        # Mode 2 nav plan (list of (cmd, dur_s))
        self.nav_plan   : list[tuple[str,float]] = []
        self.nav_idx    = 0
        self.nav_step_t = time.time()

        self.log : list[str] = []
        self._set_state("SCAN_STEP")

    # ── helpers ───────────────────────────────────────────────────────────────
    def _elapsed(self) -> float:
        return time.time() - self.state_t

    def _set_state(self, s: str):
        self.state   = s
        self.state_t = time.time()
        entry = f"M{self.mode} | {s}"
        self.log.append(entry)
        if len(self.log) > 6:
            self.log.pop(0)

    def _odom_tick(self, cmd: str):
        now = time.time()
        dt  = now - self.last_odom_t
        self.last_odom_t = now
        if cmd != "STOP":
            self.odom.update(cmd, dt)
        x, y, _ = self.odom.pose()
        if self.mode == 1:
            self.path_wander.append((x, y))
        else:
            self.path_return.append((x, y))

    def _err_x(self, box) -> float:
        return ((box[0] + box[2]) / 2) - (self.fw / 2)

    def _is_centred(self, box) -> bool:
        return abs(self._err_x(box)) < self.fw * ALIGN_THRESH_FRAC

    # ── main update called every frame ────────────────────────────────────────
    def update(self, detections: list):
        self.last_odom_t = self.last_odom_t or time.time()

        if self.mode == 1:
            # Mode 1 → Mode 2 timer
            if time.time() - self.mode1_t >= MODE1_DURATION_S:
                self.mode = 2
                self._set_state("COMPUTE_PATH")
                return
            self._run_mode1(detections)
        else:
            self._run_mode2()

    # ── MODE 1 ────────────────────────────────────────────────────────────────
    def _run_mode1(self, detections: list):
        s = self.state

        if s == "SCAN_STEP":
            self._odom_tick("ROTATE_CW")
            if self._elapsed() >= SCAN_STEP_S:
                self._set_state("SCAN_PAUSE")

        elif s == "SCAN_PAUSE":
            self._odom_tick("STOP")
            target = self._pick_target(detections)
            if target:
                self.target_box = target[0]
                self.no_ball_count = 0
                self._set_state("ALIGN")
            elif self._elapsed() >= SCAN_PAUSE_S:
                self._set_state("SCAN_STEP")

        elif s == "ALIGN":
            target = self._pick_target(detections)
            if not target:
                self.target_box = None
                self._set_state("SCAN_STEP")
                return
            self.target_box = target[0]
            if self._is_centred(self.target_box):
                self._set_state("APPROACH")
            else:
                cmd = "ROTATE_CCW" if self._err_x(self.target_box) > 0 else "ROTATE_CW"
                self._odom_tick(cmd)

        elif s == "APPROACH":
            self._odom_tick("FORWARD")
            target = self._pick_target(detections)
            if target:
                self.target_box = target[0]
                self.no_ball_count = 0
            else:
                self.no_ball_count += 1
                if self.no_ball_count >= BALL_LOST_FRAMES:
                    self.target_box = None
                    self._set_state("COAST")

        elif s == "COAST":
            self._odom_tick("FORWARD")
            if self._elapsed() >= COAST_S:
                self.target_box = None
                self._set_state("SCAN_STEP")

    # ── MODE 2 ────────────────────────────────────────────────────────────────
    def _run_mode2(self):
        s = self.state

        if s == "COMPUTE_PATH":
            self.nav_plan  = self._compute_return()
            self.nav_idx   = 0
            self.nav_step_t = time.time()
            self.path_return = [self.odom.pose()[:2]]
            self._set_state("NAVIGATE")

        elif s == "NAVIGATE":
            if self.nav_idx >= len(self.nav_plan):
                self._set_state("AT_POSITION")
                return
            cmd, dur = self.nav_plan[self.nav_idx]
            self._odom_tick(cmd)
            if time.time() - self.nav_step_t >= dur:
                self.nav_idx   += 1
                self.nav_step_t = time.time()

        elif s == "AT_POSITION":
            self._odom_tick("STOP")
            if self._elapsed() >= 0.6:
                self._set_state("DEPOSIT")

        elif s == "DEPOSIT":
            self._odom_tick("STOP")
            if self._elapsed() >= ACTUATOR_HOLD_S:
                self._set_state("RETRACT")

        elif s == "RETRACT":
            self._odom_tick("STOP")
            if self._elapsed() >= 0.6:
                self._set_state("BACK_UP")

        elif s == "BACK_UP":
            self._odom_tick("BACKWARD")
            if self._elapsed() >= BACKUP_S:
                # Reset for new Mode 1 cycle
                self.odom       = Odometry(add_noise=False)
                self.path_wander = [(0.0, 0.0)]
                self.path_return = []
                self.target_box  = None
                self.mode        = 1
                self.mode1_t     = time.time()
                self._set_state("SCAN_STEP")

    # ── Return path planner ───────────────────────────────────────────────────
    def _compute_return(self) -> list[tuple[str, float]]:
        """Simple bearing+distance return — same strategy as elegoo_gyro_return."""
        ox, oy, oa = self.odom.pose()
        dist = math.hypot(ox, oy)
        if dist < 10:
            return [("STOP", 0.2)]

        # Bearing to origin in the robot-relative frame
        target_deg = math.degrees(math.atan2(-oy, -ox)) % 360
        turn_deg   = (target_deg - oa + 180) % 360 - 180     # signed, ±180

        plan = []
        # 1. Turn to face home
        if abs(turn_deg) > 5.0:
            turn_s = abs(turn_deg) / ROTATE_SPEED_DEG_S
            plan.append(("ROTATE_CW" if turn_deg > 0 else "ROTATE_CCW", turn_s))
        # 2. Drive toward home
        drive_s = dist / FORWARD_SPEED_MM_S
        plan.append(("FORWARD", drive_s))
        # 3. Small overshoot buffer
        plan.append(("FORWARD", 0.4))
        plan.append(("STOP", 0.2))
        return plan

    # ── Target selection ──────────────────────────────────────────────────────
    def _pick_target(self, detections: list):
        if not detections:
            return None
        # Prefer the detection closest to horizontal centre
        return min(detections, key=lambda d: abs(self._err_x(d[0])))

    # ── Draw everything onto frame ────────────────────────────────────────────
    def draw(self, frame: np.ndarray, detections: list) -> np.ndarray:
        h, w = frame.shape[:2]

        # ── bounding boxes ──
        for box, conf, cls in detections:
            x1, y1, x2, y2 = [int(v) for v in box]
            is_tgt = (self.target_box is not None and
                      abs(box[0] - self.target_box[0]) < 8)
            col  = COL_GREEN if is_tgt else COL_GREY
            thick = 3 if is_tgt else 1
            cv2.rectangle(frame, (x1, y1), (x2, y2), col, thick)
            lbl = f"{COCO_NAMES.get(cls, str(cls))} {conf:.2f}"
            if is_tgt:
                lbl += " [TARGET]"
            _text_shadow(frame, lbl, (x1, y1 - 6), 0.48, col)

        # ── centre rectangle ──
        rx1, ry1, rx2, ry2 = self._crect()
        rect_col = COL_YELLOW if self.state == "APPROACH" else (0, 180, 255)
        cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), rect_col, 2)
        _text_shadow(frame, "TARGET ZONE", (rx1 + 4, ry1 - 6), 0.42, rect_col)

        # ── mode / state header ──
        m1_elapsed = time.time() - self.mode1_t
        timer_s    = max(0.0, MODE1_DURATION_S - m1_elapsed) if self.mode == 1 else 0.0
        hdr_col    = COL_GREEN if self.mode == 1 else COL_ORANGE
        hdr_txt    = (f"MODE {self.mode}  |  {self.state}" +
                      (f"  [{timer_s:.0f}s]" if self.mode == 1 else ""))
        _text_shadow(frame, hdr_txt, (10, 36), 0.90, hdr_col, thickness=2)

        # ── motor command hint ──
        hint = self._motor_hint()
        if hint:
            _text_shadow(frame, hint, (w // 2 - len(hint) * 7, h - 22), 0.70, COL_YELLOW)

        # ── spinner indicator ──
        spinner_on = self.state in ("ALIGN", "APPROACH", "COAST")
        sp_col  = COL_CYAN if spinner_on else COL_GREY
        sp_txt  = "SPINNER: ON" if spinner_on else "SPINNER: OFF"
        _text_shadow(frame, sp_txt, (w - 210, 36), 0.60, sp_col)

        # ── Mode 2 state details ──
        if self.mode == 2 and self.state == "NAVIGATE":
            if self.nav_idx < len(self.nav_plan):
                cmd, dur = self.nav_plan[self.nav_idx]
                prog = min(1.0, (time.time() - self.nav_step_t) / max(dur, 0.01))
                _text_shadow(frame, f"Step {self.nav_idx+1}/{len(self.nav_plan)}: {cmd} ({dur:.1f}s)",
                             (10, 68), 0.60, COL_WHITE)
                # progress bar
                bar_x, bar_y, bar_w = 10, 78, 200
                cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + 8), (60,60,60), -1)
                cv2.rectangle(frame, (bar_x, bar_y),
                              (bar_x + int(bar_w * prog), bar_y + 8), COL_ORANGE, -1)

        # ── log lines ──
        for i, line in enumerate(self.log[-5:]):
            _text_shadow(frame, line, (10, h - 110 + i * 22), 0.45, COL_GREY)

        # ── mini-map ──
        self._draw_minimap(frame)

        # ── target info ──
        if self.target_box and self.state in ("ALIGN", "APPROACH"):
            err = self._err_x(self.target_box)
            _text_shadow(frame, f"err_x: {int(err):+d}px", (10, 68), 0.60, COL_WHITE)

        return frame

    def _motor_hint(self) -> str:
        s = self.state
        if s == "SCAN_STEP":   return ">>> ROTATING CW"
        if s == "SCAN_PAUSE":  return "[ PAUSED — SCANNING ]"
        if s == "ALIGN":
            if self.target_box:
                return "ALIGN: <<< CCW" if self._err_x(self.target_box) > 0 else "ALIGN: CW >>>"
        if s == "APPROACH":    return "^^  FORWARD  ^^"
        if s == "COAST":
            remaining = max(0.0, COAST_S - self._elapsed())
            return f"~~ COASTING {remaining:.1f}s ~~"
        if s == "NAVIGATE":
            if self.nav_idx < len(self.nav_plan):
                return f"RETURN: {self.nav_plan[self.nav_idx][0]}"
        if s == "AT_POSITION": return "ARRIVED AT HOME"
        if s == "DEPOSIT":     return "ACTUATOR UP ^"
        if s == "RETRACT":     return "ACTUATOR DOWN v"
        if s == "BACK_UP":     return "vv  BACKING UP  vv"
        return ""

    # ── mini map ──────────────────────────────────────────────────────────────
    def _draw_minimap(self, frame: np.ndarray):
        MAP = 190
        PAD = 10
        h, w = frame.shape[:2]
        mx, my = w - MAP - PAD, PAD

        # background
        cv2.rectangle(frame, (mx, my), (mx + MAP, my + MAP), (25, 25, 25), -1)
        cv2.rectangle(frame, (mx, my), (mx + MAP, my + MAP), (80, 80, 80), 1)
        _text_shadow(frame, "PATH MAP", (mx + 5, my + 14), 0.38, (120, 120, 120))

        ox, oy, _ = self.odom.pose()
        max_r = max(abs(ox), abs(oy), 600.0)
        scale = (MAP - 36) / (2 * max_r)
        cx_m, cy_m = mx + MAP // 2, my + MAP // 2

        def to_px(rx, ry):
            return int(cx_m + rx * scale), int(cy_m + ry * scale)

        # Home marker
        hx, hy = to_px(0, 0)
        cv2.drawMarker(frame, (hx, hy), COL_GREEN, cv2.MARKER_STAR, 12, 2)

        # Wander path (blue)
        pts = self.path_wander
        if len(pts) > 1:
            for i in range(1, len(pts)):
                cv2.line(frame, to_px(*pts[i-1]), to_px(*pts[i]), (180, 120, 60), 1)

        # Return path (orange)
        rpts = self.path_return
        if len(rpts) > 1:
            for i in range(1, len(rpts)):
                cv2.line(frame, to_px(*rpts[i-1]), to_px(*rpts[i]), COL_ORANGE, 1)

        # Current position
        px, py = to_px(ox, oy)
        cv2.circle(frame, (px, py), 5, COL_RED, -1)


# ─── Drawing helper ───────────────────────────────────────────────────────────
def _text_shadow(img, text, pos, scale, colour, thickness=1):
    x, y = pos
    cv2.putText(img, text, (x+1, y+1), cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thickness+2)
    cv2.putText(img, text, (x,   y  ), cv2.FONT_HERSHEY_SIMPLEX, scale, colour,  thickness)


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    print("Loading YOLOv8 model …")
    if not os.path.exists(MODEL_PATH):
        print(f"ERROR: model not found at {MODEL_PATH}")
        return
    model = YOLO(MODEL_PATH)
    print("Model loaded.")

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera index {CAM_INDEX}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    ret, frame = cap.read()
    if not ret:
        print("ERROR: Cannot read from camera.")
        return

    h, w = frame.shape[:2]
    print(f"Camera: {w}x{h}")
    print(f"Target classes: {TARGET_CLASSES} (sports ball / apple / orange)")
    print("Q=quit  R=reset  +/-=change class\n")

    fsm = RobotFSM(w, h)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Filter for sports ball (32), apple (47), orange (49) — same as Mark 4
        results = model(frame, classes=TARGET_CLASSES, conf=CONF_THRESHOLD, verbose=False)
        dets = []
        for box in results[0].boxes:
            b    = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls  = int(box.cls[0])
            dets.append((b, conf, cls))

        fsm.update(dets)
        frame = fsm.draw(frame, dets)

        _text_shadow(frame, "detecting: sports ball / apple / orange  |  Q=quit  R=reset",
                     (10, h - 10), 0.38, (130, 130, 130))

        cv2.imshow("UnibotsMarkII Vision Test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            fsm = RobotFSM(w, h)
            print("Reset.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
