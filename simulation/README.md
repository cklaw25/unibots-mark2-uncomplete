# UnibotsMarkII — Simulation

## Files
- `env_map.py` — arena dimensions + AprilTag positions (edit these to match your real field)
- `robot_sim.py` — 2D robot simulator + visualiser

## Setup
```
pip install matplotlib numpy
```

## Run demo
```
cd C:\unibotsmark2\simulation
python robot_sim.py
```

## How to use it for return-to-start navigation

1. Measure the real tag positions in cm from the arena corner — edit `TAG_POSITIONS` in `env_map.py`
2. Set `ROBOT_START` to where the robot physically starts each match
3. Run `robot_sim.py` with your command sequence and watch the path on screen
4. Tune `FORWARD_SPEED_CM_S` and `ROTATE_SPEED_DEG_S` until the sim matches the real robot's movement
5. Use `sim.visible_tags()` to simulate what tags the camera would see at any pose
