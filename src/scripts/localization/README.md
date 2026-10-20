# UWB and ToF localization

This integrates the `localization` branch at `69051388a6bfbbf56eb32280a29e31195ad3121e`
into the DD24 ROS package. The DWM1001/PANS tag computes horizontal position from
room anchors. The ROS node publishes UWB x/y and **ToF z**, using the existing
`/pidrone/range` and `/pidrone/imu` streams. UWB height is never used in the combined pose.
Fusion estimates room XY position and velocity from UWB, blends camera optical-flow
velocity into controller feedback, and uses IMU heading to convert velocity into body axes.
ToF supplies Z. The configured IMU acceleration input predicts between measurements.
Fusion owns the continuous local state used by the PID controller.

The flight-controller loop sends RC commands before reading telemetry and queues
attitude and raw-IMU requests together before waiting for their replies. Battery
telemetry is polled at 2 Hz, leaving margin for the routines' one-second battery
freshness check. The loop still targets 60 Hz; serial turnaround determines the
achieved rate. After restarting the flight-controller node while disarmed, check
`rostopic hz /pidrone/imu` and `rostopic hz /pidrone/battery`. An incomplete batched
IMU reply warns and skips that sample without publishing cached data with a new
timestamp. The existing fault and shutdown/disarm paths remain separate.

## Install and launch on each drone

Use the interpreter that imports `rospy` (normally Python 2.7 in the repository's
ROS Kinetic container). The reader supports Python 2.7 and Python 3; hardware-free
drone tests use Python 3; laptop calibration requires Python 3.9 or newer. Do not run the ROS node with the host's Python 3
unless that environment also supplies compatible ROS Python packages.

```bash
cd ~/catkin_ws/src/pidrone_pkg
python -m pip install --user -r requirements.txt
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

The Dockerfile includes `python-monotonic` for Python 2. `pyserial` is already
installed by the image. Configure the anchors and each tag with [SETUP.md](SETUP.md).
The coordinates in `anchors.json` are examples; measure the installed anchors or
use dashboard BLE calibration below.

**Provisioned drones use the serial devices selected during setup.**
Without provisioning, the UWB tag defaults to `/dev/ttyACM1` and the flight controller
probes `/dev/ttyACM0` and `/dev/ttyACM1`; confirm the tag still owns its configured
port after reconnecting. To use another port, override it with the tag's persistent
`/dev/serial/by-id/...` path. On the drone, compare `ls -l /dev/serial/by-id/`
before and after plugging in the UWB tag.
Only one process may own the tag port; close board setup tools before launching.

```bash
roslaunch pidrone_pkg uwb_localization.launch
roslaunch pidrone_pkg fusion_localization.launch
```

The UWB Screen window uses the same default, so no port export is needed. To
override it, export `PALS_UWB_PORT` before starting Screen:

```bash
cd ~/catkin_ws/src/pidrone_pkg
screen -c pi.screenrc
```

An explicitly empty `port:=` disables serial access and publishes an actionable
error diagnostic. With a port configured, setup and reading run in a worker.
On the Pi, the reader checks existing serial owners and takes an exclusive terminal
lock before configuring the port. A busy port produces a clear error and is retried
after it is released. For manual access, stop the UWB node first, then close the
serial Screen window before restarting it; detaching Screen keeps the port open.
Privileged tools can bypass the kernel lock, so open serial terminals as `duckie`.
The reader verifies the `dwm>` PANS prompt and reboots the tag once to establish a
known stream state. Shell commands use 20 ms spacing between characters to avoid
dropped input. It reads the update rates with `aurg` (or `si`), sets both to
10 Hz with `aurs 1 1` if needed, and verifies the readback, retrying once before
enabling `lec` once. This also repairs a reverted rate on reconnect. If 10 Hz cannot
be confirmed, it logs a warning and still reads live data at the tag's configured
rate. Freshness and quality limits remain in effect. The ROS loop
checks at 20 Hz and publishes each new fix once. A delayed first fix does not toggle
the stream off. After 30 seconds without any serial data (longer for slow configured
rates), the reader closes the port and ROS retries setup automatically. It does not
toggle `lec` during a pause or terminate other programs holding the port.
`lec` reports ranging-anchor hardware IDs, configured coordinates and distances,
plus the tag's position. AN0/AN1 slot numbers are not used as anchor identities.
New position fixes require all four anchors in the same `lec` record and must
meet the quality limit. Partial anchor reports remain available for diagnostics,
but neither they nor standalone `POS` records replace the last accepted fix or
refresh its timestamp. The existing freshness timeout still applies; position
updates resume when a qualifying four-anchor fix arrives.
Serial failures invalidate the last fix immediately and trigger periodic reconnects.
For a missing/stale fix, inspect `/pidrone/localization/status`. It includes the
reader startup stage, actual update intervals/readback, serial/parsed-record ages,
last reported quality, rejected record count, and the last serial line.

The **SE** Screen window now starts fusion instead of EMA. Do not run
`state_estimator.py -p ema` alongside it: only fusion should publish
`/pidrone/state`. The legacy rigid-transform camera pose node is no longer started
by Screen. ToF, the flight controller's IMU, UWB and rosbridge remain separate
processes. The fusion launch file also starts `/optical_flow_node`, so both Screen's
SE window and routines bring up the same flow path. Camera capture remains a separate
node, shared with video and AprilTags. Existing manual movement controls keep their body-axis meaning.

The dashboard's **Connect** button starts the nodes; routines wait for them to be ready.
They refuse to start alongside the legacy `/state_estimator`. Their X/Y hold point
comes from `/pidrone/localization/fused`, the same room position shown on the
dashboard. Hold errors are rotated into body-axis velocity commands, capped at
0.10 m/s, for the existing PID; altitude changes keep that X/Y point and retain
ToF height control. Relative `move_forward`/`move_back` use the UWB room pose too: the requested
body-axis displacement is converted once into a fixed room target. Horizontal moves
request velocity proportional to the remaining error, capped at the requested speed.
The forward prototype continues one second after FLYING is confirmed, without
waiting for a settled hover.
Its velocity request is 0.5 times the remaining position error, capped at 0.5 m/s.
The takeoff altitude target remains active
(0.25 m by default). Controller output limits and paired-flight speed limits still apply.
During UWB gaps, fusion continues predicting for up to 1 second since the last
accepted fix, while IMU and altitude freshness requirements remain unchanged.
Active holds keep their target and velocity feedback. If XY trim is enabled,
it is retained but stops accumulating after the usual 0.5-second freshness window
until fresh fixes resume. The original fix timestamp is preserved throughout prediction. If the
1-second window expires, an active hold keeps its room target and requests zero XY
velocity until fusion resumes. It does not switch into a different position-hold
mode or capture a new target. Initial hold and relative horizontal moves require
a fresh fused pose; neither requires optical flow.
Local position hold retains velocity damping while moving or changing altitude.
Manual velocity commands expire from their receipt time, before PID calculation.

### Room heading

Keep the drone **disarmed, stationary and facing map +Y** when starting the fusion
ROS node. Once fresh disarmed-mode and IMU data arrive, the node automatically sets
`room_yaw_offset = pi/2 - IMU_yaw`. It logs the captured offset and exposes it as
`/fusion_localization/room_yaw_offset` for inspection. Room position is unavailable
until this startup alignment completes and UWB localization is ready.

**Floor tag 5 corrects yaw in every mode, including flight.** Keep it fixed and
flat, with the top of the standard
[printed tag](https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00005.png)
pointing toward map +Y. Fusion uses camera calibration, the mounting angles in
`apriltag_mapping.yaml`, and IMU samples matched to the image time.

Correction requires three consistent, fresh observations and is limited to
**2 degrees/second** (at most 0.2 degrees per observation). Losing the tag keeps
the last offset; hold targets and UWB position stay unchanged. This corrects
estimated heading, not a commanded yaw angle. The limits are not flight-validated.

Rebuild with `catkin_make` for the new `TagCorners` message, then restart the
detector and fusion while disarmed. The existing facing-+Y startup alignment
remains the fallback. **Align +Y** restarts fusion to repeat that alignment.
Check `tag_yaw_status`, `tag_yaw_stamp`, and `room_yaw_offset` in fusion diagnostics.
The detector sends fresh corners on `/pidrone/apriltag/reference_corners`;
the existing detections topic is unchanged.

The node also measures acceleration bias once, over at least two seconds of
stationary, disarmed IMU samples. Keep the drone still until it logs
`Startup acceleration bias ready`. Rotation, gaps, unavailable acceleration,
large residual gravity, or an unstable sample window prevent calibration.
This does not require UWB and does not recalibrate the flight controller. The bias
is frozen after capture; restarting the node measures it again. Without a bias,
the estimator continues using UWB and available optical flow, without acceleration input.

## ROS outputs and dashboard

| Topic | Type | Content |
|---|---|---|
| `/pidrone/localization/uwb` | `geometry_msgs/PoseWithCovarianceStamped` | UWB XY only, its own serial receipt timestamp; Z/orientation unmeasured |
| `/pidrone/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | UWB x/y, ToF height, position covariance, shared `map` frame |
| `/pidrone/localization/status` | `diagnostic_msgs/DiagnosticArray` | Validity, failure reason, UWB quality and height source |
| `/pidrone/localization/anchors` | `pidrone_pkg/AnchorLayout` | Latest anchor IDs and XYZ coordinates in meters, with the shared frame and original receipt time |
| `/pidrone/localization/fused` | `geometry_msgs/PoseWithCovarianceStamped` | Fused room XY, ToF Z, IMU-derived orientation and position covariance |
| `/pidrone/localization/odometry` | `nav_msgs/Odometry` | Continuous local pose in `odom`, velocity in `pidrone_body` |
| `/pidrone/state` | `pidrone_pkg/State` | Controller pose in `odom`, body-axis velocity, ToF Z |
| `/pidrone/localization/local_ready` | `std_msgs/Bool` | Fresh accepted UWB and IMU for local XY control |
| `/pidrone/localization/ready` | `std_msgs/Bool` | Fresh room solution, heading and four live anchors |
| `/pidrone/localization/fusion_status` | `diagnostic_msgs/DiagnosticArray` | Fusion readiness and rejection reason |
| `/pidrone/localization/generation` | `std_msgs/UInt32` | Reference invalidation counter; cancels active room targets |

Fusion broadcasts `map → odom → pidrone_body` transforms. Each drone has its own
ROS master; these frame names are local to that master. A combined ROS master would
need unique per-drone odom/body frames and controller topic remapping as well.

Rebuild with `catkin_make` and source `devel/setup.bash` on both drones after this
change to generate `AnchorLayout`. Restart the UWB node and rosbridge using the
rebuilt workspace. Anchor reports are published independently of ToF/IMU validity;
missing height data does not prevent the dashboard from discovering the layout.
Reports are not latched. A serial failure or stale record withdraws the current
anchor observation with an empty report; partial observations retain their actual
anchor count. Neither partial nor stale records are presented as a complete layout.

The dashboard subscribes through each drone's own rosbridge. It shows fused and raw
position/health independently, draws an oriented fused marker and a raw UWB cross, and
clears unavailable or stale positions. The map fits the four configured anchors with
a margin, keeps equal pixels per meter on X and Y, and holds its scale as drones move.
The existing raw range height card remains available for comparison.

### Room map and proposed targets

The dashboard uses laptop Bluetooth calibration or live anchor reports through
rosbridge. The last saved room is restored for display, labeled **Awaiting
verification** until recalibrated or confirmed by a complete matching live report.
Start the laptop dashboard server from the repository root:

```bash
python src/web/server.py
# Open http://localhost:8000
```

Connect a drone and wait for one complete observation of four distinct corner
anchors forming a convex XY outline. Hardware addresses label the anchor squares.
The map uses their X/Y extents, a 0.5 m margin and equal pixels per meter on both
axes. **Fit room** resets zoom; **+ / −** change zoom around the room center.
Resizing preserves equal metric scale. Reordered reports and drone movement do
not change the room or reset zoom and targets.

Fresh reports from both connected drones must agree on anchor identities, frame
and XYZ coordinates (within 0.02 m, accounting for CSV precision). Conflicting or
invalid reports pause map markers and target editing, retaining the last observed
outline. A consistent complete layout change automatically updates the room and
clears targets. A missing anchor or interrupted stream never shrinks the map. The
last complete report has a 1.5 s grace period; partial or repeated samples cannot
renew that period. Afterwards the retained outline is labeled as the last observed
layout. The map and last known positions stay visible through telemetry gaps, with
held markers faded and labeled. Fresh matching anchor and position reports resume
updates; target sending still requires live localization. Temporary fusion resets
retain held positions; disconnects and confirmed layout changes clear old markers.

`scripts/localization/anchors.json` remains a **manual board setup input only**.
Program measured coordinates with the setup tool, or use the BLE calibration below.
Reading coordinates from UWB does not detect physical movement of an anchor.
If an anchor moves, recalibrate before using room targets again.

### Dashboard anchor calibration

The laptop dashboard server uses its Bluetooth to calibrate four DWM1001 PANS anchors.
The drones read localization through their USB tags; they do not need Bluetooth.
Calibration measures all six inter-anchor distances, fits XY with shared Z = 0,
and verifies saved coordinates by reading them back from all four anchors through BLE.
The map restores the last saved coordinates for display. Restored coordinates alone
do not enable room targets; drone positions and navigation readiness always come
from live telemetry. Moving an anchor requires a fresh measurement and save.

From the repository root on your laptop (macOS or Linux, Python 3.9+):

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python src/web/server.py
```

Open `http://localhost:8000` instead of opening `index.html` directly. Enable laptop
Bluetooth and allow your terminal Bluetooth access in macOS System Settings when
prompted. Close other BLE connections to the boards and keep all four within laptop
Bluetooth range, with UWB line of sight between anchors.

Scanning, corner assignment, measurement and saving use only the laptop's Bluetooth.
They do not connect to drones or check flight state or calibration ownership.
Flight and navigation still use each drone's live telemetry.

1. Open **Calibrate anchors → Scan anchors** for new anchors. The server remembers
   discovered boards and the saved layout; returning to the same setup can go
   straight to **Measure layout**. Bluetooth reconnects and checks each board's
   identity before ranging. If an anchor is unavailable or has changed, scan again.
   Select or confirm an anchor for each labeled corner.
   Bottom left is the origin; +X runs right and +Y runs up on the map.
   All antennas must share the same height. One anchor must be the active initiator;
   the other three must be active anchors on the same PAN (default `0x0000`).
   The dashboard does not configure roles or PAN IDs.
   An empty BLE PAN value stays unknown; calibration still requires valid live
   distances between all six anchor pairs and rejects conflicting reported PAN IDs.
2. Press **Measure** and review the coordinates. Adjust each anchor's X/Y in meters
   if needed. To correct an existing calibration without measuring again, press
   **Adjust saved layout**. Edits stay in the form until you press **Save**.
3. Press **Save layout** within two minutes. Calibration completes after BLE readback
   confirms the saved coordinates on all four anchors. The server keeps four BLE
   connections open through preflight and save, then disconnects them. If the laptop
   cannot hold four connections, saving fails before any coordinates are written.
4. Keep the startup heading aligned with map +Y. A new live room fix is required
   after calibration; ToF remains the Z source.

**Cancel** stops the current operation and clears its preview.
An incomplete save retains a layout warning and recovery information in
`.calibration-state.json`, without blocking commands. Use **Measure layout → Save
layout** to verify a new layout. Incomplete saves suppress the stored map fallback.

The HTTP request/status endpoint is `/api/calibration` on the local server.
The recovery file lock prevents two laptop servers from using the same state file.
Update and restart the flight controller and fusion node on each drone when removing
the old flight lock. The controller clears the old lock parameter and publishes
`false` on `/pidrone/localization/calibration_locked` for compatibility.
The old drone **Calibration** Screen window and launch file are no longer used;
cancel any old preview, then stop coordinators still running from an older checkout.

The BLE workflow follows [PANS API guide §7.2](https://forum.qorvo.com/uploads/short-url/jT6U3NQKKCCsDP0Zjh6QFMiZ3jW.pdf).
Validate the layout on your flashed boards before flight; automated checks use simulated BLE.

Only streams whose header frame matches the configured room frame are drawn.
Leading `/` is ignored; `map`, `global`, `World` and `Body` are otherwise different
frames. Unknown or different frames are listed beneath the map and their height
graphs remain available. Transform UKF, camera and ground-truth poses into the
surveyed room frame upstream before overlaying them; changing a frame name alone
does not align coordinates. Both drones must use the same physical origin and axes.

Choose a drone in **Target drone**, then click inside the anchor outline or enter
X/Y and press **Preview**. Hollow circles and coordinate readouts show independent
targets for both drones. **Clear** removes the selected drone's proposal and cancels
its active room command. Proposals
persist through telemetry loss and are separate from measured position markers.
Previewing publishes no ROS commands. **Send target** explicitly publishes a
`geometry_msgs/PoseStamped` to `/pidrone/desired/room_target` on the selected bridge.
It requires FLYING mode, fresh fused localization and matching live anchors. The
controller independently checks freshness, frame, flight mode and that XY is inside
the anchor outline. It retains the current commanded height; the target's Z is not
an altitude command. The position error is rotated into body axes and converted to
a proportional velocity command with gain 0.20, capped at 0.10 m/s by default.
The shared Drone class uses the same settings for routine X/Y hold. These are
requested velocities, not guarantees of actual speed. Tune
`/pidrone/localization/target_gain` and `/pidrone/localization/target_speed_limit`
against the physical drone; both are read when the controller classes start.

The XY velocity controller uses proportional feedback: roll 0.50 and pitch 0.75
RC units per cm/s of velocity error. XY trim integration is temporarily disabled (Ki=0) to isolate
circling from accumulated correction; there is no fast XY integral either.
This may leave steady drift that previously required trim compensation.
The optional trim path retains its +/-5 RC limit, input cap and gap-freezing
behavior for later tuning. Roll/pitch commands remain limited to 1470–1530.
The velocity request limit is not a measured-speed governor; incorrect or delayed
velocity and heading feedback can still cause overshoot.

**Stop target** sends `/pidrone/desired/cancel_room_target` (`std_msgs/Empty`) and
returns to local hold when local feedback remains available. Manual motion,
altitude, reset or mode commands override room navigation. Room reference changes
and dashboard layout conflicts cancel active targets. A sensor timeout pauses
correction while retaining the target; fresh fusion resumes correction. Camera
loss alone does not disable XY control when UWB and IMU remain valid.
ToF altitude control remains available with fresh IMU/ToF. The outline check is
not obstacle avoidance, collision avoidance, or formation planning.

### Fusion behavior

The planar Kalman filter estimates room XY position and velocity from successive
UWB fixes, with configured IMU acceleration prediction between fixes. It filters
measurement noise rather than directly differentiating each pair of positions.
Controller velocity blends this estimate with optical flow. IMU heading rotates
the blended velocity into body axes for `/pidrone/state`; ToF supplies Z.
Local odometry integrates blended velocity without jumping at a UWB fix; direct
room-position corrections adjust the `map → odom` offset.

Optical flow arrives on `/pidrone/picamera/twist` in `pidrone_body`, with the
existing camera scale (`/pidrone/localization/flow_scale`, default 0.165).
Fusion restores the earlier EMA rotation compensation (angular rate times ToF
height times `angular_compensation`, default 0.2), then rotates the horizontal
velocity into the room frame. `flow_velocity_weight` specifies a fixed fraction
between zero and one, read at fusion startup. The launch configuration uses **0.2**:
`controller_velocity = 0.8 * uwb_imu_velocity + 0.2 * flow_velocity`.
A value of 0 disables flow's contribution; 1 uses flow for velocity whenever it
is fresh and accepted. Room position always remains anchored to UWB.
Flow does not modify the independent UWB/IMU Kalman state, so repeated camera frames
do not compound its percentage. `flow_velocity_stddev` (0.4 m/s) controls outlier
checks and reported covariance, not the blend percentage. The original
`max_flow_speed` limit (2 m/s), finite-value checks, frame, freshness and IMU/ToF
timestamp alignment checks remain separate from the statistical innovation gate.
Delayed flow observations account for known IMU velocity increments without
rewinding the estimator or jumping continuous odometry.

Flow is optional: without a fresh accepted camera sample, feedback uses 100%
UWB/IMU velocity. A rejected frame never renews the last accepted sample's timestamp.
Flow cannot initialize the room, renew a UWB timestamp,
extend the existing 1-second UWB horizon, or replace the hold target. Fusion
diagnostics include `optical flow` in `xy_source` only while an accepted flow sample
is fresh and its weight is positive, and report the active `flow_velocity_weight`
and `last_flow_result` for troubleshooting. Only fusion publishes
`/pidrone/state`; do not start the legacy EMA alongside it.

The camera motion-vector path has no tracking-confidence metric or texture gate.
A uniform floor can still produce misleading zero motion; accepted messages do
not prove the signal is accurate. No textured-floor assumption is made. Camera
headers are preserved when available; legacy headerless motion-vector messages
use the arrival time of each new message, so their acquisition latency is unknown.

The experimental `use_imu_acceleration` option adds bias-corrected acceleration
prediction while flying. The launch configuration enables it for a controlled
flight trial; sensor replay has not established a flight benefit.
When enabled, `imu_acceleration_gain` scales the room-frame XY
acceleration before integration. Its default is 0.25 and its allowed range is
greater than zero through one. This scales the acceleration input; it does not
assign fixed percentages to UWB and IMU measurements. UWB still corrects position
and velocity through the existing Kalman update. Both parameters are read when
the fusion node starts. Replay agreement with noisy UWB fixes is not a measurement
of true flight accuracy.

Acceleration is already gravity-compensated by the flight-controller node. Fusion
subtracts the startup body-frame bias, transforms right/forward/up into the IMU's
forward/left/up axes, then uses roll, pitch and yaw to obtain room XY acceleration.
It does not integrate acceleration into Z. Timestamped inputs affect prediction
only from their sample time and expire at `sensor_timeout`; invalid samples or
non-flying modes supply zero acceleration. There is no extra smoothing window.

Delayed UWB observations measure `position - age * velocity`, plus the known
acceleration contribution when enabled, with additional uncertainty over that
interval. Repeated or out-of-order observations cannot renew
a fix. Innovation gates reject inconsistent UWB observations. Prediction runs at
60 Hz, but this does not increase the UWB measurement rate or remove its latency.

Fusion owns sensor validity. `sensor_timeout` (0.5 s by default) still applies to
IMU/height data and newly received fixes. Existing XY estimates can predict for
up to 1 second without an accepted UWB fix; this limit is shared by fusion and
the controller classes through `XY_PREDICTION_TIMEOUT`.
XY feedback requires fresh IMU and an accepted UWB fix within that 1-second limit.
Once UWB expires, XY commands become neutral; the active hold target is retained for recovery. ToF
validity governs altitude output and does not reset the XY filter. Height is
validated directly from ToF and IMU timestamps; it does not depend on a synthetic
UWB sample or the current prediction time. Controllers watchdog the published
estimate, rather than timing out separate readiness heartbeats. Fused pose stamps
retain the UWB fix age.

Room output requires a recent accepted UWB fix and a validated convex four-anchor
layout. Surveyed anchors remain valid across missing/partial reports; a changed
complete layout resets the room reference. Sensor gaps do not change the room
generation. Parameters in `params/fusion_localization.yaml` are initial tuning
values. `acceleration_stddev` controls process uncertainty in predicted velocity
and replaces the former `velocity_correction_stddev` parameter, with the same 0.3
default. UWB and camera noise, latency and
heading accuracy still require measurement; fusion cannot eliminate
radio position jumps or correct IMU yaw drift.

## Height, frames and timing

Anchor coordinates and the route must share the same surveyed `map` frame on both
drones. Mount the tag close to the desired horizontal reference point: x/y are the
tag's reported coordinates, without an antenna-to-body horizontal offset correction.

By default, ToF height is tilt compensated with the IMU quaternion:

```text
z = floor_z + (ToF range - tof_offset_z) * cos(roll) * cos(pitch)
```

`tof_offset_z` is the sensor's body-z coordinate above the published reference point
(negative if the downward sensor is below it). The sensor is assumed to point along
body -z. `floor_z` is the surveyed floor height in the anchor frame. These mounting
and floor offsets default to zero. For explicitly raw, uncompensated ToF height,
launch the raw UWB/ToF node with `compensate_tilt:=false`; this removes that node's
IMU requirement. Fusion always requires fresh IMU and tilt-compensated height.
Pass identical `floor_z` and `tof_offset_z` arguments to both launch files when
using nonzero offsets. Local controller/odom Z stays height above the floor;
room pose Z includes `floor_z`, represented by the `map → odom` Z translation.

ToF reports distance to the surface underneath it. A payload, table or other raised
surface can change the apparent height. Keep the beam clear of the suspended load
and validate readings throughout the route. Replacing output z does not constrain
PANS's internal position solver. Coplanar anchor geometry still needs validation at
all flight heights; this integration does not claim a height-constrained 2D solver.

The node rejects invalid/out-of-range ToF, unavailable/invalid IMU orientations,
excessive tilt, stale sensors, and sensor timestamp disagreement. If required data
is unavailable, it publishes status but no combined pose; it never falls back to
UWB height. Each accepted UWB fix is published at most once. The header uses the
oldest contributing sensor time, rather than renewing the stamp on a cached fix.
PANS `lec` contains no acquisition timestamp: UWB time is estimated from host serial
receipt time. USB/serial buffering latency is therefore a hardware validation item.

Tune rate, quality, freshness, timestamp skew, tilt limit and initial uncertainty in
`params/uwb_localization.yaml`. Initial standard deviations (0.15 m horizontal,
0.03 m vertical) are configurable starting assumptions, not measured accuracy.
Floor, mounting offset, topic names and frame can be overridden as launch arguments.
Serial latency must be measured rather than inferred from these default noise values.

Each drone currently runs its own ROS master, so the topic names match on both.
On a shared master, use `namespace:=drone1` / `namespace:=drone2` and appropriate
range/IMU topic remaps. The current dashboard expects separate bridges and root topics.

## Verification

From the repository root:

```bash
PYTHONPATH=src/scripts python3 -m unittest discover -s src/scripts/localization -p 'test_*.py'
python3 -m unittest discover -s src/web/tests -p 'test_calibration.py'
node --test src/web/tests/dashboard.test.js
```

Tests cover serial setup/reconnect/cleanup, delayed and fragmented fixes, invalid
numbers, ToF-only height, tilt/offset math, freshness/time skew, ROS publication,
diagnostics, live anchor parsing/discovery, partial/stale/conflicting layouts,
no duplicate publication, and independent dashboard telemetry.
Fusion checks also cover delayed observations, axis rotation, covariance/outlier
handling, UWB-derived velocity and expiry, optical-flow velocity blending,
startup acceleration bias, inertial
braking with delayed fixes, layout/time resets, bounded room commands,
manual overrides and explicit dashboard sending. Hardware-free tests need NumPy and Python 3.9+.

Before flight, with props removed, inspect `rostopic echo /pidrone/localization/pose`
and `/pidrone/localization/status`; inspect `/pidrone/localization/anchors` even
with ToF temporarily unavailable. Confirm all four IDs/coordinates against the
survey and compare reports from both drones. Vary height and tilt, obstruct the range beam,
and unplug/reconnect the tag. Then measure both tags together against surveyed
positions and validate relative separation under the real payload arrangement.
