"""
Absolute localizer using AprilTag sightings.

How it works:
  1. At startup, the robot does a brief scan. If it sees any tag, it records
     its absolute arena position (abs_start_x, abs_start_y, abs_start_angle).
     Wall assignment is NOT needed — the tag ID tells us everything.

  2. During the match, if a tag is seen, it computes the robot's current
     absolute position and converts that back to a corrected relative pose
     (relative to the recorded start).

  3. If no tag was ever seen, this module returns None for any correction
     and the robot runs on pure dead-reckoning.

Tag observation input:
  tag_id   — AprilTag ID (0-23)
  distance — mm from robot to tag centre
  err_x    — pixel offset from screen centre (+ve = tag to the right)
             used to compute the bearing to the tag

Camera parameters (adjust to match your lens):
  IMAGE_W   — camera frame width in pixels
  H_FOV_DEG — horizontal field of view in degrees
"""

import math
from env_map import TAG_POSITIONS

IMAGE_W   = 1280
H_FOV_DEG = 60.0


def bearing_from_err_x(err_x: int) -> float:
    """
    Convert err_x pixel offset to bearing angle (degrees).
    Positive bearing = tag is clockwise from robot heading.
    """
    return (err_x / (IMAGE_W / 2)) * (H_FOV_DEG / 2)


class Localizer:
    """
    Manages absolute position fixes from AprilTag sightings.
    All arena coordinates in mm (origin = arena NW corner, y-down).
    """

    def __init__(self):
        self.calibrated = False
        self.abs_start_x: float = 0.0
        self.abs_start_y: float = 0.0
        self.abs_start_angle: float = 0.0  # degrees, arena frame

    def try_calibrate(self,
                      robot_abs_angle: float,
                      tag_sightings: list[tuple[int, float, int]]) -> bool:
        """
        Called at startup (before moving) with all visible tags.
        robot_abs_angle: the robot's heading in arena frame (degrees).
                         Cheaply estimated from which wall the robot is touching,
                         OR from the first tag seen.
        Returns True if calibration succeeded.
        """
        if not tag_sightings:
            return False

        # Use the closest tag for calibration
        tag_id, distance, err_x = min(tag_sightings, key=lambda t: t[1])
        fix = self._compute_abs_position(tag_id, distance, err_x, robot_abs_angle)
        if fix is None:
            return False

        self.abs_start_x, self.abs_start_y = fix
        self.abs_start_angle = robot_abs_angle
        self.calibrated = True
        return True

    def correct_odometry(self,
                          odom_rel_x: float,
                          odom_rel_y: float,
                          odom_rel_angle: float,
                          tag_sightings: list[tuple[int, float, int]]) -> tuple[float, float, float] | None:
        """
        Given current dead-reckoning relative pose and a list of tag sightings,
        returns a corrected (rel_x, rel_y, rel_angle) or None if no correction
        is possible (not calibrated or no tags visible).

        The corrected pose is in the same relative frame as the odometry
        (relative to start = (0,0,0)), so it can be fed directly back into
        Odometry.apply_absolute_correction().
        """
        if not self.calibrated or not tag_sightings:
            return None

        # Reconstruct current abs angle from start + relative rotation
        abs_angle = (self.abs_start_angle + odom_rel_angle) % 360

        # Use the closest visible tag
        tag_id, distance, err_x = min(tag_sightings, key=lambda t: t[1])
        fix = self._compute_abs_position(tag_id, distance, err_x, abs_angle)
        if fix is None:
            return None

        # Convert abs fix back to relative frame
        abs_x, abs_y = fix
        dx_abs = abs_x - self.abs_start_x
        dy_abs = abs_y - self.abs_start_y

        # Rotate abs displacement into robot-relative start frame
        a = math.radians(self.abs_start_angle)
        corrected_rel_x = dx_abs * math.cos(a) + dy_abs * math.sin(a)
        corrected_rel_y = -dx_abs * math.sin(a) + dy_abs * math.cos(a)

        return corrected_rel_x, corrected_rel_y, odom_rel_angle

    def _compute_abs_position(self,
                               tag_id: int,
                               distance_mm: float,
                               err_x: int,
                               robot_abs_angle: float) -> tuple[float, float] | None:
        """
        Compute robot's absolute arena position from a single tag observation.
        Returns (abs_x, abs_y) in mm from arena NW corner, or None on failure.
        """
        if tag_id not in TAG_POSITIONS:
            return None

        tag_abs_x, tag_abs_y, _ = TAG_POSITIONS[tag_id]

        # Bearing to tag in arena frame
        tag_bearing_deg = robot_abs_angle + bearing_from_err_x(err_x)
        tag_bearing_rad = math.radians(tag_bearing_deg)

        # Robot is at (tag_pos - vector_to_tag)
        robot_abs_x = tag_abs_x - distance_mm * math.cos(tag_bearing_rad)
        robot_abs_y = tag_abs_y - distance_mm * math.sin(tag_bearing_rad)

        return robot_abs_x, robot_abs_y
