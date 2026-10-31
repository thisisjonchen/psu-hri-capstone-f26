# Repository guidance

## Project and layout

PALS is a cooperative payload-transport project using two Duckiedrone DD24 drones.
The ROS package is `pidrone_pkg`, located in `src/` in this checkout.

- `src/scripts/PALS/`: shared drone control, routines, and perception/localization nodes.
- `src/scripts/localization/`: UWB serial access, sensor validation, fusion, and tests.
- `src/scripts/flight_controller_node.py` and `h2rMultiWii.py`: flight-controller communication.
- `src/scripts/pid_controller.py`: control loop. Throttle comes from `altitude_controller.py`
  and roll/pitch from `velocity_controller.py`; `pid_class.py` serves the dashboard position mode.
- `src/native/ekf_node.cpp` and `live_ekf.cpp`: live EKF, camera tracking and ToF corrections.
- `tools/replay_ekf.py` and `src/scripts/localization/ekf_*.py`: offline replay using shared native math.
- `tools/flow_capture.py`: record hand-carried or flight data over SSH and analyze it.
- `src/launch/`, `src/params/`, and `src/msg/`: ROS configuration and messages.
- `src/web/`: dashboard, laptop calibration service, and web tests.
- Bundled libraries contain third-party code; avoid unrelated edits.

Read the relevant README and implementation before changing behavior. Documentation
can lag behind the code; verify the actual launch configuration and data path.

## Working style

- Make the smallest coherent change that addresses the request. Simplify or remove
  redundant logic instead of adding layers, configuration switches, or fallback chains.
- Keep shared behavior in the responsible class or node. Routines should primarily
  sequence actions, not duplicate control, calibration, or sensor-validation logic.
- Inspect the working tree first and preserve existing user changes. Do not commit,
  push, deploy, or restart hardware processes unless requested or already authorized.
- Explain the result, relevant verification, and remaining uncertainty concisely.
  Separate measured observations from hypotheses and software checks from flight results.
- Keep Markdown edits brief: current usage and essential limitations, without experiment history.
- For experiments, change one relevant variable at a time and reuse existing logs
  and recordings before asking the user to repeat a completed test.

## Runtime compatibility

- Onboard ROS code must work with ROS Kinetic, Python 2.7, and the drone's older NumPy.
  Avoid Python-3-only syntax and newer APIs such as `np.block` in onboard modules.
- Laptop calibration and hardware-free tests use Python 3. Check the requirements
  for the component being edited; a modern development environment does not prove
  compatibility with the drone.
- Preserve ROS topic contracts, coordinate frames, units, and measurement timestamps.
  Do not publish cached sensor values with fresh timestamps to conceal missing data.
- Verify serial device ownership. Do not guess a UWB port or open a device already
  used by the flight controller or another process.

## Flight and localization behavior

- Preserve baseline disarm paths and watchdogs. Do not add a new automatic disarm
  trigger as a side effect of validation, exception handling, or cleanup. Trace the
  full error path, including `finally` blocks, when changing telemetry handling.
- An incomplete batched IMU reply should warn and skip the sample, not introduce a
  new exception-driven shutdown. Existing baseline fault handling remains separate.
- Do not arm, fly, calibrate hardware, or send motor commands as part of automated
  verification. A file named `test` may be an executable hardware utility; inspect it first.
- The EKF is the sole runtime motion estimator: MSP IMU predicts position, velocity,
  attitude and biases; camera-image tracking and ToF correct it. Ground-rest aiding
  requires quiet IMU data and fresh floor range. The fusion node adapts EKF output
  for the controller and handles room alignment; it runs no second motion filter.
  UWB only corrects the room offset, so hold and relative moves work without UWB
  and room waypoints require it. The floor is assumed textured.
- The camera is a fisheye tilted about 45 degrees below horizontal (`ekf.yaml`).
  The EKF tracks original JPEG images and models floor geometry and angular motion.
  Encoder vectors are disabled. Both drones share drone2's calibration by assumption;
  drone1's scales are not independently verified.
- Altitude follows the PX4/ArduPilot cascade (height -> climb rate -> thrust, with a
  thrust curve and a conservative angle boost). Roll/pitch come from a velocity PI
  with a lean limit in degrees; its integral learns constant pushes and is held, not
  cleared, while the velocity estimate is unavailable.
- Heading alignment belongs at fusion-node startup while disarmed and facing map
  +Y, not in individual routines. It does not remove ongoing yaw drift.
- Preserve an existing hold target through supported sensor gaps. Do not silently
  recapture the target, extend freshness indefinitely, or substitute UWB Z for ToF.

## Tests and validation

- Do not add tests by default. Prefer updating an existing focused test. Add a case
  only for a distinct, consequential behavior that existing coverage cannot verify.
- Remove duplicate cases, obsolete experiments, speculative checks for unused
  features, and assertions that merely repeat implementation details. Keep fixtures small.
- Retain useful coverage of control direction, output limits, sensor validity,
  communication failures, and explicit disarm behavior. Do not weaken assertions
  simply to make a failure pass.
- Run the relevant existing checks once after a change. Broaden or repeat them only
  when the change, a failure, or unresolved uncertainty warrants it. Documentation-only
  edits do not require a test run.

Available hardware-free checks, from the repository root with dependencies installed
(use `.venv/bin/python` instead of `python3` when using the existing virtual environment):

```sh
PYTHONPATH=src/scripts python3 -m unittest discover -s src/scripts/localization -p 'test_*.py'
python3 -m unittest discover -s src/web/tests -p 'test_*.py'
node --test src/web/tests/dashboard.test.js
git diff --check
```

Report which checks actually ran and any environment limitations. Passing tests or
an increased telemetry rate does not establish real-flight stability. Keep experiment
results in the task handoff or analysis artifacts rather than expanding this file
with changing gains, test counts, or flight history. Save requested bag recordings
in the current directory unless the user specifies another location.
