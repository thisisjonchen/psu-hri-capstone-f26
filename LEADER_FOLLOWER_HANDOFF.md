# Leader–follower coordination: handoff

Status as of 2026-11-07, branch `Routines` (forked from `coordination-stability` at
`e955473`). **Nothing in this document has been committed, deployed or flown.** All
verification so far is hardware-free unit tests.

Read `AGENTS.md` first. It holds the repository rules: ROS Kinetic, Python 2.7 onboard,
no arming or flying in automated checks, preserve disarm paths, and so on. Operational
reference for routine authors lives in `src/scripts/PALS/routines/README.md`
(section **Paired routines**).

---

## 1. Goal

PALS carries one payload between two Duckiedrone DD24s. The coordination layer must
make both drones act **in unison**:

1. A **standard timed handshake** for every routine step. The leader (drone1) proposes
   an action that starts at a wall-clock time `lead_s` in the future. The follower
   (drone2) accepts it. Both start at that timestamp, and the action takes the same
   time on both drones.
2. Actions are added through an expandable registry: takeoff (done), then move,
   land, payload pickup, and so on.
3. Formation flight (planned): the drones place their midpoint at a commanded (X, Y),
   aligned on the room X axis at `(X − D/2, Y)` and `(X + D/2, Y)`. The follower keeps
   its Y aligned with the leader and holds spacing D along X.

The old distance guard, a velocity filter that only limited the gap, was deleted to
rebuild this from scratch.

## 2. Decisions already made (do not re-litigate without the owner)

| Topic | Decision |
|---|---|
| Roles | drone1 = leader, drone2 = follower, from the `ROS_MASTER_URI` hostname. |
| Routine split | Only the leader's routine lists the steps. The follower runs a generic loop that validates and executes whatever plans arrive. |
| Commit rule | The follower acks only with **≥ 1.0 s** before the start. The leader executes only if it holds the ack **≥ 0.5 s** before the start; otherwise it cancels. |
| Leader cancels after the follower acked | The leader publishes its status continuously. If the follower doesn't see the leader `executing` within **0.5 s** after the start, it fails. The leader watches the follower the same way. |
| Any failure | Both drones **land immediately** (or stay disarmed if on the ground) via the routine's `finally: drone.finish()`. No new automatic disarm trigger was added. |
| When handshakes happen | Only while held: DISARMED on the ground, or FLYING with the `lock_xy()` hold. |
| Hold between plans | Every plan carries `hold_s`, set by the routine. After the action, both drones hold X/Y until `end + hold_s`. If no next plan has started by then, both land. Both compute that deadline from the same shared times. |
| Dashboard start | Removed. Solo `takeoff()` starts when the routine runs and is **refused while both drones are connected** (peer telemetry on `/pidrone/coordination/peer` within 1 s). Paired routines (`paired_*.py`) run on both drones with one Run; the leader proposes once the follower's status appears. |
| Takeoff height | Given in the plan (0.25–1.0 m), climbed directly on the existing 0.10 m/s takeoff ramp. |
| Takeoff end | A shared end time = arm (1 s) + climb from the leader's ground height + 3 s settle pad, computed by the leader and sent in the plan. Must be within ±5 cm for the last 1 s. |
| Takeoff X/Y | `lock_xy()` on the existing EKF odometry (rate-limited EKF position). Requires the EKF local and room estimates to be fresh before acceptance. |
| Code layout | Layers and conventions are in `src/scripts/PALS/README.md`: flight behaviour and its timing in `Drone`, the protocol in `coordination.py`, thin blocks in `actions.py`. |
| Formation (agreed, not built) | Move targets `(X ± D/2, Y)`, D = `nominal_separation_m` from `src/params/payload_geometry.yaml` (1.0 m). The follower couples Y to the leader and keeps D along X. |

Owner workflow preference: implement in **small, reviewed steps**, explain each one
before building it, delete obsolete code instead of leaving it dormant, and keep
code minimal with sparse comments.

## 3. Architecture and data flow

```
 drone1 (leader routine)                laptop: src/web/server.py               drone2 (follower routine)
 Leader.run() ── /pidrone/coordination/plan ──► tools/peer_relay.py ──► /pidrone/coordination/peer_plan ──► Follower
 status ───── /pidrone/coordination/plan_status ─►   (forwards on      ─► /pidrone/coordination/peer_plan_status ─► peer watch
 peer watch ◄─ /pidrone/coordination/peer_plan_status ◄─ receipt) ◄──── /pidrone/coordination/plan_status ◄─ status
```

- Each drone runs its **own ROS master**. They only talk through the laptop relay,
  which the dashboard server starts as a thread (`server.py`, `peer_relay.run`). It uses
  rosbridge on `ws://droneN.local:9090`. If the dashboard server isn't running, there
  is no handshake.
- The relay forwards plan and status messages **immediately on receipt**, independently
  of its 10 Hz pose loop and its validity checks. A delivery that stalls for more than
  0.1 s is dropped, because senders re-publish their status at 10 Hz and the leader
  re-publishes the plan every 50 ms until it is accepted.
- The relay also sends `PeerState` pose telemetry on `/pidrone/coordination/peer` at 10 Hz
  whenever both drones have valid localization on the same anchor map (any flight mode).
  `Drone.takeoff()` uses it to refuse solo flight; the planned separation check will use it.

### Messages (`src/msg/`, registered in `src/CMakeLists.txt`)

`Plan.msg`
```
std_msgs/Header header
uint32 plan_id        # increments per leader session, starting at 1
string action         # key in coordination.ACTIONS
float64[] params      # action-specific (after the leader's prepare())
time start_time       # shared wall-clock start
float64 hold_s        # hold after the action; land if no next plan starts by end + hold_s
```

`PlanStatus.msg`
```
std_msgs/Header header
string drone_id
uint32 plan_id
string state   # idle, proposed, accepted, rejected, executing, cancelled, done, failed, finished
string reason
```

## 4. The handshake (`src/scripts/PALS/coordination.py`)

### Public API
```python
from coordination import Leader, Follower

pair = Leader(drone, lead_s=10, hold_s=30)     # on drone1 only; raises on drone2
pair.takeoff(0.5)                              # = pair.run('takeoff', [0.5], 10, 30)
pair.land(hold_s=5)                            # synchronized landing; per-call override
pair.finish()                                  # follower's run() returns

Follower(drone).run()                          # on drone2: executes plans until the leader finishes
```

### Constants
| Name | Value | Meaning |
|---|---|---|
| `ACCEPT_MARGIN_S` | 1.0 | follower accepts only with this much time before the start |
| `COMMIT_MARGIN_S` | 0.5 | leader needs the acceptance this long before the start |
| `MIN_LEAD_S` / `MAX_LEAD_S` | 2 / 60 | allowed `lead_s`; the follower also rejects starts more than 60 s away |
| `MAX_HOLD_S` | 300 | longest `hold_s` |
| `START_CONFIRM_S` | 0.5 | peer must report `executing` this soon after the start |
| `PEER_TIMEOUT_S` | 0.5 | peer status older than this counts as lost (local receipt time) |
| `PEER_WAIT_S` | 60 | how long to wait for the peer's routine to first appear |
| `OVERRUN_S` | 0.5 | an action may finish this late before it fails |
| `STEP_S` | 0.05 | hold/poll granularity |

### Sequence for one plan
1. **Leader:** check the action name, `lead_s` and `hold_s`, then
   `params = action.prepare(drone, params)` and `action.validate(drone, params)`.
   Wait for the peer, check that the peer isn't failed, check held,
   `start = now + lead_s`, and check `start ≤ deadline` if a hold window is active.
   Set status `proposed` and publish the plan every 50 ms while holding.
2. **Follower:** on a plan with a new `plan_id` it rejects if:
   - the action is unknown;
   - the start is < 1 s or > 60 s away;
   - `hold_s` is invalid;
   - the start is after the current hold deadline;
   - `validate` fails, or the drone isn't held.

   Otherwise it sets status `accepted`.
3. **Leader:** if `now > start − 0.5` before an acceptance arrives, it publishes `cancelled`
   and raises. A follower `rejected` also leads to `cancelled`. On `accepted`, it proceeds.
4. **Both** (`_execute`):
   1. Hold until `start`, running the peer check each cycle.
   2. Set status `executing` and log the actual start offset.
   3. Call `action.run(drone, start, end, params, check)`, where `end = start + duration(params)`.
   4. Fail if the action overran by more than 0.5 s; otherwise hold until `end`.
   5. Set `deadline = end + hold_s` and status `done`.
5. **Peer check** (`_check_peer`) fails if any of these hold:
   - the peer status is missing or stale (> 0.5 s);
   - the peer reports a failure state for this plan;
   - more than 0.5 s after the start, the peer is neither `executing`, `done` nor
     `finished` for this plan. A higher `plan_id` from the peer counts as done, because
     the peer may finish a few milliseconds earlier and move on.
6. **Between plans:** the follower holds, and fails when the hold deadline passes or the peer
   reports a failure. The leader enforces the deadline when its routine next calls
   `run()`.
7. **Any exception** in `Leader.run` / `Follower.run` publishes a failure state, if not
   already in one, and re-raises. The routine's `finally: drone.finish()` lands.

### Action contract
```python
from coordination import Action, register

@register
class MyAction(Action):
    name = 'my_action'
    def prepare(self, drone, params):        # leader only; turns routine args into plan params
        return params
    def validate(self, drone, params):       # both; raise ValueError (RuntimeError also rejects)
        ...
    def duration(self, params):              # MUST be identical on both drones: use params only
        return params[-1]
    def run(self, drone, start, end, params, check):
        # call check() every cycle; leave the drone held (FLYING + lock_xy, or DISARMED) by end
        ...
```
Keep flight control in `Drone`, and have actions call it.

### Known caveats in the handshake
- **Clock sync is assumed.** Start times are wall-clock (`time.time()`) on NTP-synced
  clocks, and the offset has **not been measured**. Freshness uses local receipt time,
  so it is immune to offset, but start alignment is not.
- **Leader-side hold enforcement is lazy.** If the leader routine does other things
  between plans (for example `drone.hover()` outside the handshake), the follower lands at
  the deadline while the leader keeps flying until its next `run()` fails. Routines
  should propose the next plan immediately and use `lead_s` to wait, or call `finish()`.
- **`plan_id` restarts at 1 for each leader session.** The follower ignores ids at or
  below its last one, so restart both routines together.
- **Landing on failure, or by `finish()` without a `land` plan, is not synchronized.** Each
  drone lands itself. Use the `land` block for a synchronized landing.

## 5. Takeoff action (done)

### `Takeoff` in `actions.py`
- The routine passes `[height]`. `prepare` returns `[height, drone.takeoff_seconds(height) + 3.0]`, where
  `Drone.takeoff_seconds` = `ARM_SECONDS + (height − ground) / takeoff_rate`.
  Example: 0.5 m from a 0.05 m ground gives 1 + 4.5 + 3 = 8.5 s.
- `validate` rejects if:
  - the params aren't 2 finite values;
  - the height is outside 0.25–1.0 m;
  - the drone isn't DISARMED;
  - the action time is shorter than this drone's own `climb + 1 s settle`, or longer than 60 s;
  - the battery is stale or below 15.2 V;
  - `drone._hold_pose()` or `drone._hold_pose(local=True)` raises because the EKF room or
    local estimate isn't fresh.
- `duration = params[1]`; `run` calls `drone.takeoff_at(height, start, end, check)`.

### `Drone.takeoff_at(height, start, end, check)` in `drone_controller.py`
| Time | Behaviour |
|---|---|
| `start` | `_check_ground_start()`: ready() called, DISARMED, sensors and battery fresh, within 2 cm of the measured ground, ≥ 15.2 V. EKF poses fresh. Request ARMED, which must be confirmed before `start + 1`. |
| → `start + 1` | Motors idle; `check()` and sensor/battery checks every 50 ms. |
| `start + 1` | `lock_xy()`, then FLYING. The PID starts its takeoff ramp from the current height. |
| `start + 1.5` | Send the plan height (`TAKEOFF_RETARGET_SECONDS`). It's delayed so it lands after the PID's FLYING mode reset, which sets the target back to 0.25 m. The ramp is still below 0.25 m at that point, so the climb is unchanged. |
| climb → `end` | Each cycle: `check()`, `_check_flight`, publish the X/Y hold command, and track whether the height is within ±5 cm. |
| `end` | Must have been within ±5 cm for ≥ 1 s (`ALTITUDE_SETTLE_SECONDS`), otherwise it raises. Returns FLYING with X/Y locked. |

`Drone.takeoff()` (solo) is unchanged in behaviour. It now shares `_check_ground_start()`
and `ARM_SECONDS`. `_set_mode(name, timeout=5.0)` gained the timeout argument.
`Drone.takeoff_rate` is read from `/pidrone/altitude/controller` (`takeoff_rate`, default 0.10).

### PID change (`pid_controller.py`, `desired_pose_callback`)
A height target that arrives during the takeoff ramp no longer calls
`altitude.cancel_takeoff()`. The ramp logic in `AltitudeController.step` uses
`min(desired, ramp)` and ends itself when it reaches the target:
- a **higher** target extends the 0.10 m/s climb;
- a **lower** one snaps the target down and ends the ramp.

Landing, mode changes and reset still cancel the ramp. One side effect: the dashboard's
±5 cm keys during a takeoff now retarget the ramp instead of aborting it.

### What is and isn't identical between drones
- **Identical:** the arm time, the FLYING time, the ramp target trajectory and the end time.
- **Not identical:** the physical liftoff moment, which depends on each drone's hover throttle,
  and X/Y drift in the first centimetres before the EKF's `local_ready`, when X/Y runs on the
  velocity integral's trim alone.

## 6. Background: how stabilization works (needed for the move and formation work)

1. **Betaflight angle mode** holds the commanded lean angle (500 µs of stick = 60°).
2. **Velocity → lean** (`velocity_controller.py`, from `pid_controller.step`):
   - the error is the desired XY velocity minus the EKF velocity (`/pidrone/state`, heading-level body frame);
   - PI control: P = 1.2, I = 0.4, lean capped at 5°, integral capped at 3°;
   - XY is zeroed when the EKF local estimate isn't ready.
3. **Position → velocity:**
   - **The routine's X/Y lock** (`Drone.lock_xy` / `hover`): `v = 0.30·(anchor − pos)`, capped at
     0.10 m/s, published as `/pidrone/desired/twist` every 0.1 s. It disables the PID's own
     position mode while active.
   - **Room targets** use `target_velocity` on the fused pose.
   - Twist commands expire after 0.1 m of travel.
4. **Estimate:** the EKF (`src/native/ekf_node.cpp`) runs in room `map` coordinates. The MSP IMU
   predicts; camera tracking and the ToF correct; UWB (delayed about 0.3 s) corrects horizontal
   position only. The fusion node (`src/scripts/PALS/fusion_localization_node`) publishes:
   - `/pidrone/state`;
   - `/pidrone/localization/odometry`: the EKF position after a rate limiter that follows EKF
     velocity and closes jumps at ≤ 5 cm/s (`max_correction_speed`). `lock_xy` holds this one;
   - `/pidrone/localization/fused`: the same pose once room readiness is confirmed.

## 7. File map

| File | Role | Status |
|---|---|---|
| `src/scripts/PALS/README.md` | architecture and conventions (follow this) | new |
| `src/scripts/PALS/coordination.py` | handshake protocol and action registry | new |
| `src/scripts/PALS/actions.py` | paired blocks (`Takeoff`, `Land`) | new |
| `src/scripts/PALS/routines/paired_hover.py` | reference paired routine (both drones, one Run) | new |
| `src/scripts/PALS/drone_controller.py` | `takeoff_at`, `_check_ground_start`, `_set_mode(timeout)` | modified |
| `src/scripts/pid_controller.py` | guard removed; ramp retarget | modified |
| `src/scripts/localization/state_checks.py` | `fresh`, `finite`, `uncertainty`, timeouts (moved from the deleted guard) | new |
| `src/msg/Plan.msg`, `src/msg/PlanStatus.msg` | handshake messages | new |
| `src/CMakeLists.txt` | registers the new messages | modified |
| `tools/peer_relay.py` | forwards plan and status; imports from `state_checks` | modified |
| `src/scripts/flight_controller_node.py` | import moved to `state_checks` | modified |
| `src/params/payload_geometry.yaml` | comments only (D = `nominal_separation_m`) | modified |
| `src/scripts/PALS/routines/README.md` | Paired routines section | modified |
| `src/scripts/localization/test_coordination.py` | handshake protocol tests | new |
| `src/scripts/localization/test_actions.py` | block tests (`Takeoff`, `Land`) | new |
| `src/scripts/localization/test_peer_relay.py` | relay parser tests (moved) plus a forwarding test | new |
| `src/scripts/localization/test_fusion.py` | + ramp retarget test | modified |
| `src/scripts/localization/test_routine_control.py` | + timed takeoff test | modified |
| `distance_guard.py`, `test_distance_guard.py`, `src/params/coordination.yaml` | old guard | deleted (staged) |

## 8. How to verify (hardware-free)

From the repository root:
```sh
PYTHONPATH=src/scripts .venv/bin/python -m unittest discover -s src/scripts/localization -p 'test_*.py'
.venv/bin/python -m unittest discover -s src/web/tests -p 'test_*.py'
node --test src/web/tests/dashboard.test.js
git diff --check
```
Current result: 124 localization tests, **3 known failures that already exist on a clean
HEAD** (`test_tag_hold` ×2, `test_velocity_controller`). Web tests and dashboard tests
(37) pass. There is no Python 2 on the development laptop, so 2.7 compatibility was
checked by reading the code only.

## 9. Cleanup still needed

1. **The working tree is not committed.** There are also unrelated user edits (EKF startup,
   boot and dashboard files, `hover_routine.py --height`), untracked ` 2.py`/` 2.yaml`
   duplicates and `venv/`. Don't sweep those into a commit without asking.
2. **Merge with `coordination-stability`** (Jon): conflicts in `pid_controller.py` (drop his
   optional guard), `drone_controller.py`, `peer_relay.py`, `test_distance_guard.py` (delete),
   and the EKF startup files. His `payload_carry.py` uses the removed dashboard start and should
   be deleted or rebuilt from blocks.

## 10. Roadmap (next steps, in order)

1. **Example routine** (done). `routines/paired_hover.py --fly [--height] [--seconds]` runs on both
   drones and branches on `identity()`. The dashboard synchronized start, the paired/solo
   selection (`/pidrone/coordination/paired`, `FlightReadiness.paired`), the "Separation guard"
   field and the relay's `peer_preview` topic were removed.
2. **Deploy.** Run `catkin_make` on both drones so `Plan`/`PlanStatus` exist, then do a first
   onboard run. The owner deploys; don't restart hardware processes without permission.
3. **Ground check (props off), run by the owner.** Measure each drone's NTP offset (for example
   `chronyc tracking`). Run the paired takeoff with props removed and compare the logged
   `Plan N takeoff started X s after ...` lines, then estimate the start skew.
   Note: this arms the motors. Then fly `paired_hover.py`.
4. **Synchronized `land` action** (done, not flown). `Drone.land(hold_xy, check, deadline)` disarms
   at ground + 5 cm, or when stalled below ground + 10 cm with no new 1 cm of progress for 1 s; the
   default timeout floor is 10 s. `Drone.landing_seconds()` reads `AltitudeController`'s descent
   rates. The `Land` block (`actions.py`) sends `[landing_seconds + 3 s]` and runs
   `drone.land(hold_xy=True, check=check, deadline=end)`. Routines call it as `pair.land()`:
   every registered block is a `Leader` method.
5. **`move` action to the formation midpoint.**
   - Params `[mid_x, mid_y, T]`; each drone targets `(mid_x ± D/2, mid_y)`.
   - The leader's `prepare` picks T from the longer of the two travel distances and the speed limit
     (the leader limit is about 0.067 m/s). It needs the follower's position, which can come from
     the relay's `PeerState`.
   - Each drone follows a time-based reference `p0 + (p1 − p0)·s((t − start)/T)` in the room frame:
     feedforward velocity plus a proportional correction. It ends held.
   - **Open decisions:** which drone takes −D/2 (fixed, or by current X order so they never cross),
     and whether to hold on the smoothed EKF odometry or the raw room pose.
6. **Follower formation coupling.** With the leader L, follower F and side s = ±1:
   ```
   ex = (Fx − Lx) − s·D      # spacing error along X
   ey =  Fy − Ly             # alignment error
   follower velocity = planned velocity − k·(ex, ey)
   ```
   Use the leader's position projected forward: `L(now) ≈ L(stamp) + v_L·(now − stamp)`, rejecting
   anything older than 0.5 s. Relay latency of 0.1–0.4 s at 0.067 m/s is about 2 cm, the same size
   as the tolerance. The leader pauses the plan clock if `|ey|` or `|ex|` exceeds a threshold
   (proposed: 5 cm and 10 cm).
   - **Decide with the owner** whether the existing relay pose stream (`PeerState`) carries L.
7. **Validate the formation error's accuracy before trusting it.**
   - **Static, props off:** place both drones on tape marks exactly D apart on a chalk line and
     record 60 s; the mean is the bias and the spread is the noise. Swap the drones and repeat
     at 2–3 room spots.
   - **Rigid bar:** carry both drones on a bar at exactly D, aligned, at flight speed. The true
     error is zero, so the measured error is measurement error, including latency.
   - **Pass criterion:** the measurement error must be well below the control tolerance (for
     example 95% under 2 cm for a 5 cm tolerance).
   - The EKF covariance is optimistic, because UWB noise is correlated (about 1.9 cm per axis
     with a 1 s correlation).
8. **Payload actions** (pickup and others) on the same registry.

## 11. Constraints worth repeating

- Onboard code must work on Python 2.7 / ROS Kinetic with old NumPy. That means no f-strings
  and no `math.isfinite`.
- Do not arm, fly or send motor commands in automated tests. A file named `test` may be a
  hardware utility.
- Preserve the baseline disarm paths. Failures in the coordination layer raise into the routine's
  `finally: drone.finish()` (a controlled landing); they must not add new disarm triggers.
- Passing tests do not establish flight behaviour. Report software checks and flight results
  separately.
