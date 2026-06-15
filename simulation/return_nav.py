"""
Return-to-start navigator.

Strategy:
  1. Compute bearing and distance to start ONCE using dead-reckoning.
  2. Execute a single rotation to face start, then a single long drive.
     (Many tiny rotation steps accumulate angle noise and cause spinning loops.)
  3. Opportunistic tag correction mid-return — if a tag is seen, correct the
     estimate and recompute bearing/distance. Never pauses to look for tags.
  4. Wall-seek final step — rotate to face wall, drive until physical contact.
     Works even with lateral error because the wall is a hard boundary.

Usage:
    nav = ReturnNavigator(odometry, localizer, start_wall_angle_deg)
    for cmd, duration in nav.commands():
        robot.apply(cmd, duration)
        nav.notify_tags(robot.visible_tags())
"""

import math
from odometry import Odometry, FORWARD_SPEED_MM_S, ROTATE_SPEED_DEG_S
from localizer import Localizer

ALIGN_THRESHOLD_DEG  = 5.0    # accept alignment within this (degrees)
ARRIVAL_THRESHOLD_MM = 150    # mm — switch to wall-seek phase when this close
WALL_SEEK_DRIVE_S    = 0.5    # s per drive pulse in wall-seek phase
WALL_SEEK_STEPS      = 8      # max drive pulses (8 × 100mm = 800mm clearance)


class ReturnNavigator:
    def __init__(self,
                 odom: Odometry,
                 localizer: Localizer,
                 start_wall_angle_deg: float = None):
        self.odom = odom
        self.loc  = localizer
        self.start_wall_angle = start_wall_angle_deg
        self._pending_tags: list[tuple[int, float, int]] = []
        self.correction_count = 0

    def notify_tags(self, tag_sightings: list[tuple[int, float, int]]):
        self._pending_tags = tag_sightings

    def commands(self):
        """
        Generator yielding (command_str, duration_s).
        Phases: align → drive → (tag correction if available) → wall-seek.
        """
        # ── Phase 1: rotate to face start ────────────────────────────────────
        bearing = self.odom.bearing_to_start()
        if abs(bearing) > ALIGN_THRESHOLD_DEG:
            rotation_s = abs(bearing) / ROTATE_SPEED_DEG_S
            cmd = "ROTATE_CW" if bearing > 0 else "ROTATE_CCW"
            yield (cmd, rotation_s)

        # ── Phase 2: drive toward start ───────────────────────────────────────
        # Re-apply any tag correction that arrived during the rotation
        self._try_tag_correction()

        dist = self.odom.distance_to_start()
        if dist > ARRIVAL_THRESHOLD_MM:
            # Drive most of the way, leaving ARRIVAL_THRESHOLD_MM as buffer
            drive_dist = dist - ARRIVAL_THRESHOLD_MM
            drive_s    = drive_dist / FORWARD_SPEED_MM_S
            yield ("FORWARD", drive_s)

        # ── Phase 3: tag correction mid-approach (if available) ───────────────
        self._try_tag_correction()

        # Fine-approach: small iterative steps for the last ~150mm
        # Only a few steps so angle noise stays minimal
        for _ in range(6):
            dist = self.odom.distance_to_start()
            if dist <= ARRIVAL_THRESHOLD_MM / 3:
                break
            bearing = self.odom.bearing_to_start()
            if abs(bearing) > ALIGN_THRESHOLD_DEG:
                rot_s = abs(bearing) / ROTATE_SPEED_DEG_S
                yield ("ROTATE_CW" if bearing > 0 else "ROTATE_CCW", rot_s)
            yield ("FORWARD", 0.3)

        # ── Phase 4: wall-seek ────────────────────────────────────────────────
        if self.start_wall_angle is not None:
            # Rotate to face the wall (= 180° in relative frame = directly behind start)
            _, _, cur_rel = self.odom.pose()
            target_rel   = 180.0
            bearing      = (target_rel - cur_rel + 180) % 360 - 180

            if abs(bearing) > ALIGN_THRESHOLD_DEG:
                rot_s = abs(bearing) / ROTATE_SPEED_DEG_S
                yield ("ROTATE_CW" if bearing > 0 else "ROTATE_CCW", rot_s)

            # Drive into wall — robot will physically stop at boundary
            for _ in range(WALL_SEEK_STEPS):
                yield ("FORWARD", WALL_SEEK_DRIVE_S)

        yield ("STOP", 0.1)

    def _try_tag_correction(self):
        if not self._pending_tags:
            return
        rel_x, rel_y, rel_angle = self.odom.pose()
        corrected = self.loc.correct_odometry(rel_x, rel_y, rel_angle, self._pending_tags)
        if corrected is not None:
            cx, cy, ca = corrected
            drift = math.hypot(cx - rel_x, cy - rel_y)
            if drift > 50:
                self.odom.apply_absolute_correction(cx, cy, ca)
                self.correction_count += 1
                print(f"[ReturnNav] Tag correction #{self.correction_count}: "
                      f"drift={drift:.0f}mm ({rel_x:.0f},{rel_y:.0f})→({cx:.0f},{cy:.0f})")
        self._pending_tags = []
