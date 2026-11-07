# Routines

Each `.py` file here is listed under **Routines** in the dashboard and run on the drone
with the arguments typed there. Routines sequence `Drone` calls from
`../drone_controller.py`; control, sensor checks and watchdogs stay in that class.

## Template

```python
rospy.init_node('my_routine', disable_signals=True)
drone = Drone()
drone.ready()                    # disarmed, sensors fresh, battery OK
try:
    drone.takeoff(hold_xy=True)  # waits for the dashboard start, climbs, holds X/Y
    drone.hover(3.0)
finally:
    drone.finish(hold_xy=True)   # always land/disarm, even after an error
```

Copy `hover_routine.py` for argument parsing; it requires `--fly` before flying.

## Drone methods

Units are meters, seconds and degrees. Body axes: x right, y forward, z up.
Room frame: `map` (UWB), yaw counterclockwise, 0 = facing +Y, 180 = facing -Y.

| Call | Does |
|---|---|
| `ready()` | Wait for flight nodes, sensors and battery while disarmed on the ground. |
| `takeoff(hold_xy=False)` | Wait for the dashboard's synchronized start, arm, climb to `takeoff_height` (0.25 m). |
| `hover(seconds, command=None)` | Hover; holds the `lock_xy()` anchor, or uses `command()` each cycle. |
| `lock_xy()` / `unlock_xy()` | Hold / release the current position (local odometry). |
| `move_to_altitude(height)` | Climb or descend to a downward-range height. |
| `move(x, y, z)` | Move relative to the body (local odometry). Unlock X/Y first. |
| `move_forward(m)` / `move_back(m)` | `move(y=±m)`. |
| `move_to(x, y, 'map')` | Fly to a room waypoint (UWB pose), keeping height. Unlock X/Y first. |
| `turn_to(yaw_deg)` | Turn in place to a room yaw; holds the `lock_xy()` anchor. Call while stopped. |
| `land(hold_xy=False)` / `finish(hold_xy=False)` | Land and disarm; `finish` is the safe `finally` exit. |

Movement calls block until done and raise `RuntimeError` on timeout, no progress,
stale sensors, low battery or a mode change; `finally: drone.finish()` then lands.
`move`, `move_to` and `turn_to` accept `check=` (called each cycle, raise to abort).

## AprilTags

`../tag_hold.py`: `tag = TagHold(drone)`, `tag.wait_ready()` before `ready()`, then in flight
`tag.acquire(tag_id=None)`, `drone.hover(seconds, tag.command)` and `tag.finish()`.
See `apriltag_lockon.py`.

## Limitations

- Yaw drifts after initialization; UWB does not correct heading.
- `turn_to` direction and yaw gain are not yet flight-verified.
- No path planning: moves are straight lines toward the target.
