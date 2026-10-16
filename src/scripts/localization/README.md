# Camera, UWB and ToF localization

This integrates the `localization` branch at `69051388a6bfbbf56eb32280a29e31195ad3121e`
into the DD24 ROS package. The DWM1001/PANS tag computes horizontal position from
room anchors. The ROS node publishes UWB x/y and **ToF z**, using the existing
`/pidrone/range` and `/pidrone/imu` streams. UWB height is never used in the combined pose.
The fusion node combines camera planar velocity with UWB XY, uses IMU heading to
align their axes, and publishes a fused room position with ToF Z. It also owns the
continuous local state used by the PID controller.

## Install and launch on each drone

Use the interpreter that imports `rospy` (normally Python 2.7 in the repository's
ROS Kinetic container). The reader supports Python 2.7 and Python 3; hardware-free
drone tests use Python 3; laptop calibration requires Python 3.9 or newer. Do not run the ROS node with the host's Python 3
unless that environment also supplies compatible ROS Python packages.

```bash
cd ~/catkin_ws/src/pidrone_pkg
python -m pip install --user -r scripts/localization/requirements.txt
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

The Dockerfile includes `python-monotonic` for Python 2. `pyserial` is already
installed by the image. Configure the anchors and each tag with [SETUP.md](SETUP.md).
The coordinates in `anchors.json` are examples; measure the installed anchors or
use dashboard BLE calibration below.

**The UWB tag defaults to `/dev/ttyACM1`.** The flight controller already
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
Serial failures invalidate the last fix immediately and trigger periodic reconnects.
For a missing/stale fix, inspect `/pidrone/localization/status`. It includes the
reader startup stage, actual update intervals/readback, serial/parsed-record ages,
last reported quality, rejected record count, and the last serial line.

The **SE** Screen window now starts fusion instead of EMA. Do not run
`state_estimator.py -p ema` alongside it: only fusion should publish
`/pidrone/state`. The legacy rigid-transform camera pose node is no longer started
by Screen. Optical flow, ToF, the flight controller's IMU, UWB and rosbridge remain
separate processes. Existing manual movement controls keep their body-axis meaning.

The altitude and hover routines also start UWB and fusion, reusing running nodes.
They refuse to start alongside the legacy `/state_estimator`. Their X/Y hold point
comes from `/pidrone/localization/fused`, the same room position shown on the
dashboard. Hold errors are rotated into body-axis velocity commands, capped at
0.20 m/s, for the existing PID; altitude changes keep that X/Y point and retain
ToF height control. Relative `move_forward`/`move_back` still use optical flow.
During a fused-position update gap, an active hold keeps its room target and uses
the PID's existing local position hold at the gap location. Fresh fusion resumes
correction toward the original room target. The initial hold still needs a fresh
fused position, and the existing camera/ToF checks remain in effect.

### Room heading

Keep the drone **disarmed, stationary and facing map +Y** when starting the fusion
ROS node. Once fresh disarmed-mode and IMU data arrive, the node automatically sets
`room_yaw_offset = pi/2 - IMU_yaw`. It logs the captured offset and exposes it as
`/fusion_localization/room_yaw_offset` for inspection. Room position is unavailable
until this startup alignment completes and UWB localization is ready.

Alignment happens once per node start. Subsequent turns, disarming, and UWB gaps
keep that offset. Restart fusion while facing +Y to align again. No routine or
class calibration call is needed; the old manual launch/environment offset is
removed. ToF still supplies height, and UWB still corrects room X/Y. This startup
alignment does not correct ongoing IMU yaw drift.

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
| `/pidrone/localization/local_ready` | `std_msgs/Bool` | Fresh camera, IMU and ToF for local XY control |
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
python -m pip install -r src/web/requirements.txt
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
2. Press **Measure layout** and review the shape and dimensions.
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
a proportional velocity command, capped at 0.2 m/s by default. This is a requested
speed limit, not a guarantee of actual speed. Tune `/pidrone/localization/target_gain`
and `/pidrone/localization/target_speed_limit` against the physical drone.

**Stop target** sends `/pidrone/desired/cancel_room_target` (`std_msgs/Empty`) and
returns to local hold when camera/IMU/ToF remain available. Manual motion, altitude,
reset or mode commands override room navigation. Room reference changes, stale
UWB, stale/partial anchors or camera loss cancel active targets without automatic
resumption. Dashboard layout conflicts cancel both drones' room targets. Camera
loss additionally disables local XY control and clears horizontal commands;
ToF altitude control remains available with fresh IMU/ToF. The outline check is
not obstacle avoidance, collision avoidance, or formation planning.

### Fusion behavior

The small planar Kalman filter integrates camera velocity, with the existing
rotational-flow compensation, and applies covariance-weighted UWB XY corrections.
It does not fuse the legacy camera position estimate. Local odometry never jumps
when UWB corrects the room estimate: corrections adjust the `map → odom` offset.
Delayed UWB observations are advanced using interpolated camera displacement from
their own timestamp, and additional uncertainty covers that interval. Repeated or
out-of-order observations cannot renew a fix. Innovation gates reject inconsistent
UWB observations. Camera gaps are not integrated as measured motion.

Room output requires a recent accepted UWB fix and a fresh complete convex four-
anchor report; it stops instead of publishing camera-only room positions during
an outage. Reacquisition requires a new accepted UWB fix and an explicit new target.
Local odometry can continue for hover during UWB loss; it is not presented as a valid
room position. Parameters in `params/fusion_localization.yaml` are initial tuning
values. Camera velocity scale, noise assumptions and heading accuracy require
measurement. The optical-flow scale can be set with
`/pidrone/localization/flow_scale` before starting the flow node.

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
Legacy camera drivers without a motion-vector Header use callback receipt time;
drivers with a Header retain acquisition time. Camera/serial latency and flow scale
must be measured rather than inferred from these default noise values.

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
handling, camera gaps, room expiry, layout/time resets, bounded room commands,
manual overrides and explicit dashboard sending. Hardware-free tests need NumPy and Python 3.9+.

Before flight, with props removed, inspect `rostopic echo /pidrone/localization/pose`
and `/pidrone/localization/status`; inspect `/pidrone/localization/anchors` even
with ToF temporarily unavailable. Confirm all four IDs/coordinates against the
survey and compare reports from both drones. Vary height and tilt, obstruct the range beam,
and unplug/reconnect the tag. Then measure both tags together against surveyed
positions and validate relative separation under the real payload arrangement.
