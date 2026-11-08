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
| `takeoff_at(height, start, end, check)` | Timed takeoff used by the paired `takeoff` action (see below). |
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

## Paired routines (`../coordination.py`)

drone1 leads and drone2 follows (from `ROS_MASTER_URI`). Run the leader's routine on
drone1 and a follow routine on drone2; the laptop dashboard server relays plans and
status between them. The leader proposes each step; both start it at the same
wall-clock time (clocks must be NTP-synced).

```python
leader = Leader(drone)                 # drone1
leader.run(name, params, lead_s, hold_s)
leader.finish()                        # follower stops waiting; each drone then lands itself

Follower(drone).run()                  # drone2: executes plans until the leader finishes
```

- `lead_s` (2-60 s): the start is this far in the future. The follower accepts only
  with at least 1 s left; the leader runs only if it has the acceptance 0.5 s before.
- `hold_s` (0-300 s): after the action, both hold X/Y. If no next plan starts by
  `end + hold_s`, both land. Propose the next plan right away; use `lead_s` to wait.
- Plans are proposed and accepted only while held: disarmed on the ground, or flying
  with `lock_xy()`. Each drone must see the other report `executing` within 0.5 s of
  the start, with peer status never older than 0.5 s.
- Any failure (rejection, late acceptance, lost or failed peer, overrun) raises
  `RuntimeError`, tells the peer, and `finally: drone.finish()` lands.

Adding an action: subclass `Action`, decorate with `@register`, and define
`duration(params)` (identical on both drones), `validate(drone, params)` (raise
`ValueError`), `run(drone, start, end, params, check)` (call `check()` every cycle;
end held by `end`) and optionally `prepare(drone, params)` (leader only, builds the
plan params). Keep flight control in `Drone`; actions sequence it.

| Action | Routine params | Does |
|---|---|---|
| `takeoff` | `[height]` (0.25-1.0 m) | Arm at the start; at +1 s lock X/Y and fly; ramp to the height at `takeoff_rate` (0.10 m/s); be within 5 cm for the last 1 s before the end. The leader sets the action time to arm + climb from its ground height + 3 s. Requires `ready()`, battery >= 15.2 V and fresh EKF local and room estimates. |

Paired routines do not use the dashboard's synchronized start: launch both, and the
leader proposes once the follower's status appears.

## Limitations

- Yaw drifts after initialization; UWB does not correct heading.
- `turn_to` direction and yaw gain are not yet flight-verified.
- No path planning: moves are straight lines toward the target.
- Paired handshake: start alignment depends on clock sync (not yet measured); landing
  at the end of a paired routine is not synchronized. Not yet flight-tested.
