# PALS flight stack: architecture and conventions

How flight behaviour is layered, from altitude physics up to routines, and the rules
for where new code goes. Routine authors: see `routines/README.md`. Status and
roadmap for the leader–follower work: `LEADER_FOLLOWER_HANDOFF.md` at the repo root.

## Layers

| Layer | File | Owns | Never |
|---|---|---|---|
| 1. Physics | `../altitude_controller.py` | Takeoff ramp (`takeoff_rate`), landing descent rates (`LAND_RATE`, `LAND_FINAL_RATE`, `LAND_SLOW_HEIGHT`), throttle | Knows about routines or pairing |
| 2. Control node | `../pid_controller.py` | Topics → physics: FLYING starts the ramp, `/pidrone/desired/land` starts the descent, height targets retarget the ramp | Sequences steps |
| 3. Drone | `drone_controller.py` | One drone's flight behaviour **and how long it takes**: `takeoff`, `takeoff_at`, `land`, `finish`, holds, moves, `takeoff_seconds`, `landing_seconds` | Knows about the peer or plans |
| 4. Protocol | `coordination.py` | Handshake: `Leader`, `Follower`, shared start and end, peer watch, hold window; the `Action` contract and registry | Contains blocks or flight control |
| 5. Blocks | `actions.py` | Paired actions (`Takeoff`, `Land`, …): plan params, validation, shared duration, one `Drone` call each | Re-derives physics or commands topics directly |
| 6. Routines | `routines/*.py` | Order of blocks / `Drone` calls for a mission | Contains control, timing or sensor logic |

Transport: `../../msg/Plan.msg`, `PlanStatus.msg`; the laptop relay `tools/peer_relay.py`
forwards plans and status between drones.

## Solo and paired routines

- **Solo** routines call `Drone` methods on one drone. `Drone.takeoff()` refuses while the
  peer relay is streaming the other drone (`/pidrone/coordination/peer` within
  `PEER_ATTACHED_SECONDS`), so a solo routine cannot be launched on both drones.
- **Paired** routines are named `paired_*.py`, run on both drones from one dashboard Run,
  and branch on `coordination.identity()`: `Leader` blocks on drone1, `Follower(drone).run()`
  on drone2 (see `routines/paired_hover.py`). They never call `Drone` motion directly;
  timed behaviour goes through blocks only (`takeoff_at`, `land(check, deadline)`).
- There is no dashboard synchronized start: synchronization exists only in the handshake.
- Debugging a routine that won't start: `routines/README.md`, "Troubleshooting".

## Call flow of one block

```
routine (drone1)   pair.takeoff(0.5)                → Leader.run('takeoff', [0.5], lead_s, hold_s)
coordination.py    plan, follower acceptance, wait for start, peer watch, hold window
actions.py         Takeoff.run(drone, start, end, params, check)
drone_controller   Drone.takeoff_at(height, start, end, check)
pid_controller     FLYING → AltitudeController.start_takeoff(); height target retargets the ramp
```
drone2 runs `Follower(drone).run()`, which executes the same block from the plan.

## Conventions

**Where code goes**
- Flight behaviour, sensor checks and anything physical (rates, settle, touchdown) go in
  `Drone`. Durations are physics too: `Drone` exposes them (`takeoff_seconds(height)`),
  reading layer-1 constants rather than copying numbers.
- A block in `actions.py` adds only mission policy: pads, height or speed limits,
  validation messages. It calls exactly one `Drone` method in `run`.
- The protocol (`coordination.py`) changes only when the handshake itself changes.
- Routines sequence blocks; they do not compute targets, durations or checks.
- A new paired mission is a new `routines/paired_*.py` modelled on `paired_hover.py`; a new
  capability is a new block, not routine code.

**Block contract** (`coordination.Action`)
- `prepare(drone, params)`: leader only; turns routine arguments into plan params.
  Anything both drones must agree on (durations, targets) is computed here and sent.
- `validate(drone, params)`: both drones; raise `ValueError` (or let a `Drone`
  `RuntimeError` through) to reject.
- `duration(params)`: computed **from params only**, so it is identical on both drones.
- `run(drone, start, end, params, check)`: call `check()` every cycle (pass it into the
  `Drone` method), and leave the drone **held** by `end`: FLYING with `lock_xy()`, or
  DISARMED on the ground. Raise `RuntimeError` on failure.

**`Drone` method naming**
- Use a `*_at(…, start, end, check)` method only when the behaviour must act at an exact
  timestamp itself (e.g. `takeoff_at` arms at `start`).
- Otherwise extend the existing method with `check=None` and `deadline=None`
  (e.g. `land(hold_xy, check, deadline)`), so solo routines, failure paths and blocks
  share one implementation.

**Safety**
- Failures raise; the routine's `finally: drone.finish()` lands. Do not add new automatic
  disarm paths (see `AGENTS.md`). Landing's touchdown disarm lives only in `Drone.land`.
- Paired routines `import actions` (registers the blocks); `Leader`/`Follower` refuse to
  start with an empty registry.

**Compatibility and tests**
- Onboard files (all of this directory) run on Python 2.7 / ROS Kinetic: no f-strings,
  no `math.isfinite`, old NumPy.
- Tests mirror the layers in `../localization/`: `test_altitude_controller.py` (1),
  `test_fusion.py` (2), `test_routine_control.py` (3), `test_coordination.py` (4),
  `test_actions.py` (5). Tests never arm or fly.

## Adding a block (checklist)
1. If the behaviour is new, add or extend the `Drone` method (and its duration helper)
   with a test in `test_routine_control.py`.
2. Add the `@register` class to `actions.py` following the contract above, with a test in
   `test_actions.py` (params, duration, rejections).
3. Add a row to the action table in `routines/README.md`.
