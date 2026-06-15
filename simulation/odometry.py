"""
Dead-reckoning odometry.

Tracks robot pose in a local frame where the starting position is always (0, 0, 0).
No external data required — purely integrates movement commands.

Angle convention: degrees, 0=East, 90=South, 180=West, 270=North (y-down).
"""

import math

# Tune these to match real robot behaviour
FORWARD_SPEED_MM_S = 200.0
ROTATE_SPEED_DEG_S = 90.0
SIM_DT = 0.05

# Simulated noise — 2% is realistic for PWM motors without encoders
# Set to 0.0 for ideal dead-reckoning, raise to stress-test robustness
FORWARD_NOISE_FRAC = 0.02
ROTATE_NOISE_FRAC  = 0.02


class Odometry:
    """
    Integrates movement commands into a pose estimate relative to start.
    start is always (0, 0, angle=0 in relative frame).
    """

    def __init__(self, add_noise: bool = False):
        self.rel_x = 0.0    # mm from start, in robot-relative East direction
        self.rel_y = 0.0    # mm from start, in robot-relative South direction
        self.rel_angle = 0.0  # degrees from start heading
        self.add_noise = add_noise
        self._history: list[tuple[str, float]] = []

    def update(self, cmd: str, duration_s: float):
        """Integrate one command into the pose estimate."""
        self._history.append((cmd, duration_s))
        steps = max(1, int(duration_s / SIM_DT))

        import random
        for _ in range(steps):
            noise_f = (1.0 + random.uniform(-FORWARD_NOISE_FRAC, FORWARD_NOISE_FRAC)) if self.add_noise else 1.0
            noise_r = (1.0 + random.uniform(-ROTATE_NOISE_FRAC,  ROTATE_NOISE_FRAC))  if self.add_noise else 1.0

            rad = math.radians(self.rel_angle)
            if cmd == "FORWARD":
                d = FORWARD_SPEED_MM_S * SIM_DT * noise_f
                self.rel_x += d * math.cos(rad)
                self.rel_y += d * math.sin(rad)
            elif cmd == "BACKWARD":
                d = FORWARD_SPEED_MM_S * SIM_DT * noise_f
                self.rel_x -= d * math.cos(rad)
                self.rel_y -= d * math.sin(rad)
            elif cmd == "ROTATE_CW":
                self.rel_angle = (self.rel_angle + ROTATE_SPEED_DEG_S * SIM_DT * noise_r) % 360
            elif cmd == "ROTATE_CCW":
                self.rel_angle = (self.rel_angle - ROTATE_SPEED_DEG_S * SIM_DT * noise_r) % 360

    def pose(self) -> tuple[float, float, float]:
        """Current estimated pose (rel_x, rel_y, rel_angle) relative to start."""
        return self.rel_x, self.rel_y, self.rel_angle

    def distance_to_start(self) -> float:
        return math.hypot(self.rel_x, self.rel_y)

    def heading_to_start(self) -> float:
        """
        Angle (degrees) the robot needs to face in order to point directly at start.
        Returns the world-frame angle to target, NOT the turn amount.
        Use bearing_to_start() for how much to turn.
        """
        return math.degrees(math.atan2(-self.rel_y, -self.rel_x)) % 360

    def bearing_to_start(self) -> float:
        """
        Signed degrees the robot must rotate to face start.
        Positive = CW, negative = CCW.
        """
        target = self.heading_to_start()
        diff = (target - self.rel_angle + 180) % 360 - 180
        return diff

    def apply_absolute_correction(self,
                                   true_rel_x: float,
                                   true_rel_y: float,
                                   true_rel_angle: float):
        """
        Override the dead-reckoning estimate with a corrected pose.
        Called when a tag fix gives us a better position estimate.
        """
        self.rel_x = true_rel_x
        self.rel_y = true_rel_y
        self.rel_angle = true_rel_angle
