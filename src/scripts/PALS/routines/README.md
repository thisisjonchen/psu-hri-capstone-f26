# Routines

Each `.py` file here is listed under **Routines** in the dashboard and run on the
selected drones with the arguments typed there. Routines sequence `Drone` calls from
`../drone_controller.py`; control, sensor checks and watchdogs stay in that class.

## Solo and paired routines

| | Solo (`hover_routine.py`, …) | Paired (`paired_*.py`) |
|---|---|---|
| Drones | One. Refused while both drones are connected to the dashboard. | Both, launched together with one Run. |
| Takeoff | `drone.takeoff()` starts when the routine runs. | `pair.takeoff(h)`: the leader's plan sets the shared start. |
| Calls | `Drone` methods | `Leader` block methods on drone1, `Follower(drone).run()` on drone2 |
| Name | anything | `paired_` prefix, so operators know to select both |

Never mix them: a solo routine cannot fly two drones in sync (the dashboard has no
synchronized start), and paired blocks wait for a follower.

## Template

```python
rospy.init_node('my_routine', disable_signals=True)
drone = Drone()
drone.ready()                    # disarmed, sensors fresh, battery OK
try:
    drone.takeoff(hold_xy=True)  # refused if the other drone is connected; climbs, holds X/Y
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
| `takeoff(hold_xy=False)` | Solo only: refuse if peer telemetry arrived within 1 s, then arm and climb to `takeoff_height` (0.25 m). |
| `takeoff_at(height, start, end, check)` | Timed takeoff used by the paired `takeoff` action (see below). |
| `hover(seconds, command=None)` | Hover; holds the `lock_xy()` anchor, or uses `command()` each cycle. |
| `lock_xy()` / `unlock_xy()` | Hold / release the current position (local odometry). |
| `move_to_altitude(height)` | Climb or descend to a downward-range height. |
| `move(x, y, z)` | Move relative to the body (local odometry). Unlock X/Y first. |
| `move_forward(m)` / `move_back(m)` | `move(y=±m)`. |
| `move_to(x, y, 'map')` | Fly to a room waypoint (UWB pose), keeping height. Unlock X/Y first. |
| `turn_to(yaw_deg)` | Turn in place to a room yaw; holds the `lock_xy()` anchor. Call while stopped. |
| `land(hold_xy=False, check=None, deadline=None)` | Descend and disarm at ground + 5 cm, or after 1 s stalled below ground + 10 cm. Default timeout: at least 10 s. |
| `finish(hold_xy=False)` | Land if flying, else disarm; the safe `finally` exit. |
| `takeoff_seconds(height)` / `landing_seconds()` | Arm + climb time from the ground; descent time from the current height. Used by paired blocks. |

Movement calls block until done and raise `RuntimeError` on timeout, no progress,
stale sensors, low battery or a mode change; `finally: drone.finish()` then lands.
`move`, `move_to` and `turn_to` accept `check=` (called each cycle, raise to abort).

## AprilTags

`../tag_hold.py`: `tag = TagHold(drone)`, `tag.wait_ready()` before `ready()`, then in flight
`tag.acquire(tag_id=None)`, `drone.hover(seconds, tag.command)` and `tag.finish()`.
See `apriltag_lockon.py`.

## Paired routines (`../coordination.py`, blocks in `../actions.py`)

Select **both** drones and run a `paired_*.py` routine once. Each drone picks its role
from `ROS_MASTER_URI` (`coordination.identity()`): drone1 leads, drone2 follows. The
dashboard server's peer relay forwards plans and status between them, so both must
stay connected. The leader proposes each step; both start it at the same wall-clock
time (clocks must be NTP-synced). `paired_hover.py` is the reference routine:

```python
import actions                         # registers the blocks; required on both drones
from coordination import LEADER, Follower, Leader, identity

drone.ready()
try:
    if identity() == LEADER:
        pair = Leader(drone, lead_s=10, hold_s=30)   # defaults for every block
        pair.takeoff(0.5)
        pair.land(hold_s=5)            # per-call override; any block: pair.<name>(*params)
        pair.finish()                  # follower's run() returns
    else:
        Follower(drone).run()          # executes the leader's plans until it finishes
finally:
    drone.finish(hold_xy=True)         # lands unsynchronized if a block failed
```

- `lead_s` (2-60 s): the start is this far in the future. The follower accepts only
  with at least 1 s left; the leader runs only if it has the acceptance 0.5 s before.
- `hold_s` (0-300 s): after the action, both hold X/Y. If no next plan starts by
  `end + hold_s`, both land. Propose the next plan right away; use `lead_s` to wait.
  Don't call `drone.hover()` or other `Drone` motion between blocks.
- Plans are proposed and accepted only while held: disarmed on the ground, or flying
  with `lock_xy()`. Each drone must see the other report `executing` within 0.5 s of
  the start, with peer status never older than 0.5 s.
- Any failure (rejection, late acceptance, lost or failed peer, overrun) raises
  `RuntimeError`, tells the peer, and `finally: drone.finish()` lands each drone.
- Both routines must start within 60 s of each other (`PEER_WAIT_S`).

Adding a block: follow the architecture, contract and checklist in `../README.md`.

| Action | Routine params | Does |
|---|---|---|
| `takeoff` | `[height]` (0.25-1.0 m) | Arm at the start; at +1 s lock X/Y and fly; ramp to the height at `takeoff_rate` (0.10 m/s); be within 5 cm for the last 1 s before the end. The leader sets the action time to arm + climb from its ground height + 3 s. Requires `ready()`, battery >= 15.2 V and fresh EKF local and room estimates. |
| `land` | none | Descend holding X/Y; disarm at ground + 5 cm, or after 1 s stalled below ground + 10 cm, by the end. The leader sets the action time to its own descent time (`LAND_RATE`, then `LAND_FINAL_RATE` below 0.15 m) + 1 s stall window + 3 s; each drone rejects a plan shorter than its own descent. Requires flight with the X/Y lock. |

### Paired routines available

| Routine | Leader calls |
|---|---|
| `paired_hover.py --fly [--height 0.5] [--seconds 10]` | `pair.takeoff(h, hold_s=N + 5)`, `pair.land(lead_s=N, hold_s=5)`; the drones hold X/Y during the land plan's lead time |

Planned blocks for the payload mission (not built): `move_to`, `lower`/`lift`, `sweep`,
`center_on_payload`.

## Troubleshooting a routine that won't start or stops early

Read the routine's terminal output (and the PID/FC windows for refusals). Common causes:

| Message | Cause and fix |
|---|---|
| `Both drones are connected; solo routines fly one drone` | A solo routine saw the peer relay's `/pidrone/coordination/peer` within 1 s. Use a `paired_*` routine, or disconnect the other drone in the dashboard. |
| `Cannot identify drone1/drone2 from ROS_MASTER_URI` | A paired routine runs on a host not named `drone1`/`drone2`. |
| `drone2 cannot run the leader handshake` | `Leader` was created on drone2; branch on `identity()` as in `paired_hover.py`. |
| `No actions registered` | The routine did not `import actions`. |
| `Peer routine did not appear within 60 s` | The other drone's routine is not running, or the relay is down (dashboard server stopped, a drone disconnected). |
| `Peer plan status is stale` / `missing` | Relay or Wi-Fi dropped status for more than 0.5 s; each drone lands itself. |
| `Plan N rejected: start is X s away` | Clocks differ by about a second or more; check NTP on both drones. |
| `Plan N rejected: …` (other) | The follower failed the block's `validate` (not held, battery, stale EKF, plan shorter than its own descent); the reason text says which. |
| `Plan N cancelled: no acceptance 0.5 s before start` | The follower's acceptance arrived late; increase `lead_s` or check the relay. |
| `No new plan started within the hold` | The leader did something other than propose the next block before `end + hold_s`. |
| `Takeoff was not settled …` / `Landing was not confirmed` | The block's physical timing did not fit the plan; the drone lands itself. Compare the logged heights with `takeoff_seconds` / `landing_seconds`. |
| FC `Refusing ARMED: preflight_unavailable` / `local_estimate_unavailable` | The PID's preflight readiness (`/pidrone/coordination/preflight`) is missing or the EKF local estimate is not ready. |

The relay status line in the dashboard server log (`Relaying peer telemetry.` or
`Peer relay paused: …`) tells whether the drones see each other. The relay publishes
peer poses only while both drones have valid, fresh localization on the same anchor map.

## Limitations

- Yaw drifts after initialization; UWB does not correct heading.
- `turn_to` direction and yaw gain are not yet flight-verified.
- No path planning: moves are straight lines toward the target.
- Paired handshake: start alignment depends on clock sync (not yet measured). Only
  `pair.land()` lands together; failure landings are per drone. No separation check
  yet: start the drones about 1 m apart. Not yet flight-tested.
- The solo refusal relies on the relay: two solo routines launched from terminals with
  the dashboard closed, or before both drones have localization, are not detected.
