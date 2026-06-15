"""
2D robot simulator for UnibotsMarkII.

Shows three things:
  - True path (ground truth, blue)
  - Dead-reckoning estimate path (orange dashed) — drifts over time due to noise
  - Return path after return-to-start is triggered (red)

Run:
    pip install matplotlib numpy
    python robot_sim.py
"""

import math
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from env_map import (
    ARENA_W, ARENA_H, NET_DEPTH, NET_START, NET_END,
    TAG_POSITIONS, SCORING_NETS, get_start_pose
)
from odometry import Odometry, FORWARD_SPEED_MM_S, ROTATE_SPEED_DEG_S, SIM_DT
from localizer import Localizer
from return_nav import ReturnNavigator

# Which wall your team starts on this round — only used for ground-truth sim
# In the real robot this is unknown; the navigator doesn't rely on it
START_WALL = "South"


# ── Ground-truth robot (perfect physics, no noise) ───────────────────────────
class TrueRobot:
    def __init__(self, wall=START_WALL):
        self.x, self.y, self.angle = get_start_pose(wall)
        self.start = (self.x, self.y, self.angle)
        self.true_path  = [(self.x, self.y)]
        self.odom = Odometry(add_noise=True)   # noisy estimate alongside true
        self.odom_path  = [(0.0, 0.0)]         # relative to start

    def apply(self, cmd: str, duration_s: float) -> bool:
        """
        Apply command to both ground-truth and odometry.
        Returns False if the robot has hit its start wall (wall-contact).
        """
        steps = max(1, int(duration_s / SIM_DT))
        wall_hit = False
        for _ in range(steps):
            rad = math.radians(self.angle)
            if cmd == "FORWARD":
                self.x += FORWARD_SPEED_MM_S * SIM_DT * math.cos(rad)
                self.y += FORWARD_SPEED_MM_S * SIM_DT * math.sin(rad)
            elif cmd == "BACKWARD":
                self.x -= FORWARD_SPEED_MM_S * SIM_DT * math.cos(rad)
                self.y -= FORWARD_SPEED_MM_S * SIM_DT * math.sin(rad)
            elif cmd == "ROTATE_CW":
                self.angle = (self.angle + ROTATE_SPEED_DEG_S * SIM_DT) % 360
            elif cmd == "ROTATE_CCW":
                self.angle = (self.angle - ROTATE_SPEED_DEG_S * SIM_DT) % 360

            # Wall collision — clamp to arena bounds
            self.x = max(0, min(ARENA_W, self.x))
            self.y = max(0, min(ARENA_H, self.y))

            # Detect if robot has reached its start wall
            sx, sy, sa = self.start
            if sa == 270 and self.y >= ARENA_H:   wall_hit = True
            elif sa == 90  and self.y <= 0:        wall_hit = True
            elif sa == 0   and self.x >= ARENA_W:  wall_hit = True
            elif sa == 180 and self.x <= 0:        wall_hit = True

            self.true_path.append((self.x, self.y))
            if wall_hit:
                break

        self.odom.update(cmd, duration_s)
        ox, oy, _ = self.odom.pose()
        self.odom_path.append((ox, oy))
        return not wall_hit

    def visible_tags(self, fov_deg=60, max_range_mm=2000):
        """Simulate camera seeing tags — based on TRUE position."""
        visible = []
        for tag_id, (tx, ty, _) in TAG_POSITIONS.items():
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist > max_range_mm or dist < 1:
                continue
            tag_angle = math.degrees(math.atan2(dy, dx))
            rel_angle = (tag_angle - self.angle + 180) % 360 - 180
            if abs(rel_angle) <= fov_deg / 2:
                err_x = int(rel_angle / (fov_deg / 2) * 640)
                visible.append((tag_id, dist, err_x))
        return sorted(visible, key=lambda v: v[1])

    def pose(self):
        return self.x, self.y, self.angle

    def abs_to_rel(self, abs_x, abs_y):
        """Convert absolute arena coords to relative-to-start coords."""
        sx, sy, sa = self.start
        dx, dy = abs_x - sx, abs_y - sy
        a = math.radians(sa)
        rel_x =  dx * math.cos(a) + dy * math.sin(a)
        rel_y = -dx * math.sin(a) + dy * math.cos(a)
        return rel_x, rel_y

    def rel_to_abs(self, rel_x, rel_y):
        """Convert relative-to-start coords back to absolute arena coords."""
        sx, sy, sa = self.start
        a = math.radians(sa)
        abs_x = sx + rel_x * math.cos(a) - rel_y * math.sin(a)
        abs_y = sy + rel_x * math.sin(a) + rel_y * math.cos(a)
        return abs_x, abs_y


# ── Random wander generator ───────────────────────────────────────────────────
def _random_wander(num_moves: int = None) -> list[tuple[str, float]]:
    """
    Generate a randomised wander sequence of forward drives and rotations.
    num_moves: total number of move segments (default: random 6-12)
    Forward durations: 1.0 – 5.0 s
    Rotation durations: 0.3 – 2.0 s (mapped to ~27°–180°)
    Direction (CW / CCW) chosen randomly each time.
    """
    if num_moves is None:
        num_moves = random.randint(6, 12)

    seq = []
    for i in range(num_moves):
        # Alternate loosely between forward and rotate, but randomise which
        if i == 0 or random.random() < 0.55:
            seq.append(("FORWARD", round(random.uniform(1.0, 5.0), 1)))
        else:
            direction = random.choice(["ROTATE_CW", "ROTATE_CCW"])
            seq.append((direction, round(random.uniform(0.3, 2.0), 1)))

    seq.append(("STOP", 0.2))
    return seq


# ── Demo simulation ───────────────────────────────────────────────────────────
def run_demo():
    robot = TrueRobot(wall=START_WALL)
    loc   = Localizer()

    print("=== UnibotsMarkII Return-to-Start Simulation ===\n")
    print(f"True start: {robot.start}")

    # --- Startup calibration scan (optional, 0 cost) ---
    startup_tags = robot.visible_tags()
    sx, sy, sa = robot.start
    calibrated = loc.try_calibrate(sa, startup_tags)
    print(f"Startup calibration: {'OK — absolute position locked' if calibrated else 'SKIPPED — no tags visible at start'}")
    print(f"Visible at start: {[t[0] for t in startup_tags]}\n")

    # --- Wander phase (randomised — simulates collecting balls) ---
    print("--- Wander phase ---")
    wander = _random_wander()
    print("Wander sequence:")
    for cmd, dur in wander:
        print(f"  {cmd:<12} {dur:.1f}s")
        robot.apply(cmd, dur)

    ox, oy, oa = robot.odom.pose()
    tx, ty, ta = robot.pose()
    print(f"True position:        ({tx:.0f}, {ty:.0f})  angle={ta:.1f}°")
    print(f"Dead-reckoning est.:  ({ox:.0f}, {oy:.0f})  angle={oa:.1f}°  (relative)")
    rx, ry = robot.rel_to_abs(ox, oy)
    print(f"DR estimate (abs):    ({rx:.0f}, {ry:.0f})")
    print(f"Drift from truth:     {math.hypot(tx - rx, ty - ry):.0f} mm\n")

    # --- Return phase ---
    print("--- Return phase ---")
    _, _, start_angle = robot.start
    nav = ReturnNavigator(robot.odom, loc, start_wall_angle_deg=start_angle)
    return_true_path  = [(robot.x, robot.y)]
    return_odom_path  = []

    for cmd, dur in nav.commands():
        # Feed current visible tags into the navigator (opportunistic correction)
        tags = robot.visible_tags()
        nav.notify_tags(tags)

        robot.apply(cmd, dur)
        return_true_path.append((robot.x, robot.y))
        ox, oy, _ = robot.odom.pose()
        return_odom_path.append(robot.rel_to_abs(ox, oy))

    fx, fy, _ = robot.pose()
    sx, sy, _ = robot.start
    final_error = math.hypot(fx - sx, fy - sy)
    print(f"\nFinal position:   ({fx:.0f}, {fy:.0f})")
    print(f"Start position:   ({sx:.0f}, {sy:.0f})")
    print(f"Error from start: {final_error:.0f} mm")
    if nav.correction_count > 0:
        print("Tag correction was applied during return — improved accuracy.")
    else:
        print("No tag correction — pure dead-reckoning return.")

    render(robot, return_true_path, return_odom_path, final_error)


# ── Renderer ──────────────────────────────────────────────────────────────────
def render(robot: TrueRobot,
           return_true_path: list,
           return_odom_path: list,
           final_error: float):

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-NET_DEPTH - 30, ARENA_W + NET_DEPTH + 30)
    ax.set_ylim(ARENA_H + NET_DEPTH + 30, -NET_DEPTH - 30)
    ax.set_aspect('equal')
    ax.set_title("UnibotsMarkII — Return-to-Start Simulation", fontsize=13, fontweight='bold')
    ax.set_xlabel("x (mm)  →  East")
    ax.set_ylabel("y (mm)  ↓  South")

    # Arena floor
    ax.add_patch(mpatches.Rectangle(
        (0, 0), ARENA_W, ARENA_H,
        linewidth=3, edgecolor='black', facecolor='#f9f9f9', zorder=1))

    # Scoring nets
    for side, info in SCORING_NETS.items():
        c = info["color"]
        lo, hi = info["net_min"], info["net_max"]
        wc = info["wall_coord"]
        if info["axis"] == "x":
            nx, ny = lo, wc - NET_DEPTH if side == "North" else wc
            ax.add_patch(mpatches.Rectangle(
                (nx, ny), hi - lo, NET_DEPTH,
                linewidth=2, edgecolor=c, facecolor=c, alpha=0.3, zorder=2))
            ax.text((lo + hi) / 2, ny + NET_DEPTH / 2, side,
                    ha='center', va='center', fontsize=9, color=c, fontweight='bold')
        else:
            nx, ny = (wc if side == "East" else wc - NET_DEPTH), lo
            ax.add_patch(mpatches.Rectangle(
                (nx, ny), NET_DEPTH, hi - lo,
                linewidth=2, edgecolor=c, facecolor=c, alpha=0.3, zorder=2))
            ax.text(nx + NET_DEPTH / 2, (lo + hi) / 2, side,
                    ha='center', va='center', fontsize=9, color=c, fontweight='bold')

    # AprilTags
    TAG_SIZE = 100
    for tag_id, (tx, ty, facing) in TAG_POSITIONS.items():
        wall = _tag_wall(tag_id)
        c = SCORING_NETS[wall]["color"]
        half = TAG_SIZE / 2
        offsets = {90: (tx-half, ty-TAG_SIZE), 270: (tx-half, ty),
                   180: (tx-TAG_SIZE, ty-half), 0: (tx, ty-half)}
        rx_p, ry_p = offsets[facing]
        ax.add_patch(mpatches.Rectangle(
            (rx_p, ry_p), TAG_SIZE, TAG_SIZE,
            linewidth=1, edgecolor='black', facecolor=c, alpha=0.75, zorder=3))
        ax.text(tx, ty, str(tag_id), ha='center', va='center',
                fontsize=5.5, fontweight='bold', color='black', zorder=4)

    # Wander path (true)
    if len(robot.true_path) > 1:
        xs, ys = zip(*robot.true_path)
        ax.plot(xs, ys, '-', color='steelblue', linewidth=1.5, alpha=0.7,
                label='True wander path', zorder=5)

    # Wander path (dead-reckoning estimate, converted to abs)
    odom_abs = [robot.rel_to_abs(x, y) for x, y in robot.odom_path]
    if len(odom_abs) > 1:
        xs, ys = zip(*odom_abs)
        ax.plot(xs, ys, '--', color='darkorange', linewidth=1.2, alpha=0.65,
                label='Dead-reckoning estimate', zorder=5)

    # Return path (true)
    if len(return_true_path) > 1:
        xs, ys = zip(*return_true_path)
        ax.plot(xs, ys, '-', color='crimson', linewidth=2.0, alpha=0.85,
                label='True return path', zorder=6)

    # Start marker
    sx, sy, sa = robot.start
    ax.plot(sx, sy, '*', markersize=18, color='limegreen', zorder=8, label='Start')

    # Final position
    fx, fy, _ = robot.pose()
    ax.plot(fx, fy, 'D', markersize=10, color='black', zorder=9, label='Final position')

    # Arrow: final heading
    length = 80
    dx = length * math.cos(math.radians(robot.pose()[2]))
    dy = length * math.sin(math.radians(robot.pose()[2]))
    ax.annotate('', xy=(fx+dx, fy+dy), xytext=(fx, fy),
                arrowprops=dict(arrowstyle='->', color='black', lw=2))

    # Info box
    info = (f"Final error from start: {final_error:.0f} mm\n"
            f"Tag correction used: {'Yes' if any('correction' in str(l) for l in []) else 'see console'}")
    ax.text(15, ARENA_H - 15, f"Final error from start: {final_error:.0f} mm",
            va='bottom', fontsize=10, fontweight='bold',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))

    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, linestyle=':', alpha=0.25)
    plt.tight_layout()
    plt.show()


def _tag_wall(tag_id):
    if tag_id <= 5:  return "North"
    if tag_id <= 11: return "East"
    if tag_id <= 17: return "South"
    return "West"


if __name__ == "__main__":
    run_demo()
