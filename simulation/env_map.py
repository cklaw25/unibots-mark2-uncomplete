"""
Competition arena map — exact dimensions from rulebook.

Coordinate system (all mm):
  Origin = top-left (NW) corner of arena floor
  x → East (right)
  y → South (down)
  angle → degrees, 0 = facing East, 90 = South, 180 = West, 270 = North

Arena: 2000 x 2000 mm
Walls: ~150mm tall, ~18-20mm thick

Tag layout per wall (center-to-center from left corner of that wall):
  150 | 300 | 300 | [net 800mm] | 300 | 300 | 150  = 2000mm total
  → tag positions at 150, 450, 750, 1250, 1550, 1850 mm from each wall's left corner
"""

ARENA_W = 2000   # mm, East–West
ARENA_H = 2000   # mm, North–South

# Net dimensions
NET_LENGTH  = 800    # mm along wall
NET_DEPTH   = 200    # mm outward from wall
NET_START   = 600    # mm from wall corner to start of net (each side)
NET_END     = 1400   # NET_START + NET_LENGTH

WALL_THICKNESS = 19  # mm (~18-20mm)

# ── AprilTag positions on walls ───────────────────────────────────────────────
# Tag centres are ON the wall surface (x or y = 0 / 2000).
# Offsets along each wall (from that wall's left/top corner): 150, 450, 750, 1250, 1550, 1850
_OFFSETS = [150, 450, 750, 1250, 1550, 1850]

# (x, y, facing_into_arena_degrees)
TAG_POSITIONS = {
    # North wall (y=0), tags 0-5, left→right, robot faces South (90°)
    0:  (_OFFSETS[0], 0,    90),
    1:  (_OFFSETS[1], 0,    90),
    2:  (_OFFSETS[2], 0,    90),
    3:  (_OFFSETS[3], 0,    90),
    4:  (_OFFSETS[4], 0,    90),
    5:  (_OFFSETS[5], 0,    90),

    # East wall (x=2000), tags 6-11, top→bottom, robot faces West (180°)
    6:  (2000, _OFFSETS[0], 180),
    7:  (2000, _OFFSETS[1], 180),
    8:  (2000, _OFFSETS[2], 180),
    9:  (2000, _OFFSETS[3], 180),
    10: (2000, _OFFSETS[4], 180),
    11: (2000, _OFFSETS[5], 180),

    # South wall (y=2000), tags 12-17, right→left, robot faces North (270°)
    12: (2000 - _OFFSETS[0], 2000, 270),
    13: (2000 - _OFFSETS[1], 2000, 270),
    14: (2000 - _OFFSETS[2], 2000, 270),
    15: (2000 - _OFFSETS[3], 2000, 270),
    16: (2000 - _OFFSETS[4], 2000, 270),
    17: (2000 - _OFFSETS[5], 2000, 270),

    # West wall (x=0), tags 18-23, bottom→top, robot faces East (0°)
    18: (0, 2000 - _OFFSETS[0], 0),
    19: (0, 2000 - _OFFSETS[1], 0),
    20: (0, 2000 - _OFFSETS[2], 0),
    21: (0, 2000 - _OFFSETS[3], 0),
    22: (0, 2000 - _OFFSETS[4], 0),
    23: (0, 2000 - _OFFSETS[5], 0),
}

# ── Scoring nets (one per wall) ───────────────────────────────────────────────
# Each entry: (wall_side, color, net_start_x_or_y, net_end_x_or_y)
# Net is outside the arena wall (y < 0 for North, x > 2000 for East, etc.)
SCORING_NETS = {
    "North":  {"color": "gold",       "tags": list(range(0,  6)),  "axis": "x", "wall_coord": 0,    "net_min": NET_START, "net_max": NET_END},
    "East":   {"color": "orange",     "tags": list(range(6,  12)), "axis": "y", "wall_coord": 2000, "net_min": NET_START, "net_max": NET_END},
    "South":  {"color": "mediumpurple","tags": list(range(12, 18)), "axis": "x", "wall_coord": 2000, "net_min": NET_START, "net_max": NET_END},
    "West":   {"color": "limegreen",  "tags": list(range(18, 24)), "axis": "y", "wall_coord": 0,    "net_min": NET_START, "net_max": NET_END},
}

# ── Team assignment ───────────────────────────────────────────────────────────
# Set this to whichever wall your team's net is on.
# The robot starts centered on that net, touching the wall.
MY_WALL = "South"   # ← change to "North", "East", "West" as needed

def get_start_pose(wall=MY_WALL):
    """Return (x_mm, y_mm, angle_deg) for robot starting against its net wall."""
    net_centre = (NET_START + NET_END) / 2   # 1000mm — centre of net along wall
    if wall == "North":
        return (net_centre, 0, 90)      # facing South into arena
    elif wall == "East":
        return (2000, net_centre, 180)  # facing West
    elif wall == "South":
        return (net_centre, 2000, 270)  # facing North into arena
    elif wall == "West":
        return (0, net_centre, 0)       # facing East
