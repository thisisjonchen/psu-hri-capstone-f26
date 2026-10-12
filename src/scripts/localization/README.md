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
tests use Python 3.5 or newer. Do not run the ROS node with the host's Python 3
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
roslaunch pidrone_pkg fusion_localization.launch room_yaw_offset:=YOUR_MEASURED_OFFSET_RADIANS
```

The UWB Screen window uses the same default, so no port export is needed. To
override it, export `PALS_UWB_PORT` before starting Screen. Set the measured room
heading separately on each drone:

```bash
export PALS_ROOM_YAW_OFFSET=YOUR_MEASURED_OFFSET_RADIANS
cd ~/catkin_ws/src/pidrone_pkg
screen -c pi.screenrc
```

An explicitly empty `port:=` disables serial access and publishes an actionable
error diagnostic. With a port configured, setup and reading run in a worker.
The reader verifies the `dwm>` PANS prompt, reboots the tag once to establish a known
stream state, then enables `lec` once. A delayed first fix does not toggle it off.
`lec` reports ranging-anchor hardware IDs, configured coordinates and distances,
plus the tag's position. AN0/AN1 slot numbers are not used as anchor identities.
Serial failures invalidate the last fix immediately and trigger periodic reconnects.

The **SE** Screen window now starts fusion instead of EMA. Do not run
`state_estimator.py -p ema` alongside it: only fusion should publish
`/pidrone/state`. The legacy rigid-transform camera pose node is no longer started
by Screen. Optical flow, ToF, the flight controller's IMU, UWB and rosbridge remain
separate processes. Existing manual movement controls keep their body-axis meaning.

### Calibrate heading on each drone

UWB anchors cannot supply the drone's yaw. With the drone stationary, measure the
heading of its **forward** direction counterclockwise from room +X, read the yaw
from `/pidrone/imu`, and set:

```text
room_yaw_offset = surveyed_forward_heading - IMU_yaw   (radians)
```

For example, forward along room +X with IMU yaw zero gives offset `0`; forward
along room +Y with IMU yaw zero gives `1.57079632679`. The controller's legacy axes
are X=right and Y=forward, so the body-to-odom planar rotation is `IMU_yaw - pi/2`.
The implementation includes this basis conversion; do not add another quarter turn
to the measured offset. Calibrate after FC startup and repeat after any heading
reset. A blank offset disables fused room output and room-target commands. Local
camera hover and ToF altitude do not require a room heading. Heading drift remains
an accuracy limit; a single UWB position does not correct yaw.

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

The dashboard discovers its layout exclusively from the live anchor topic through
each drone's rosbridge. There is no JSON download, file picker, stored layout, or
offline fallback. Open `web/index.html` directly, or serve the package root:

```bash
python3 -m http.server 8000 --directory src
# Open http://localhost:8000/web/index.html
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
layout, and marker display/target editing wait for fresh complete discovery. Only
live reports with matching anchors enable a drone's measured markers. Reports from
a disconnected bridge are discarded; reconnect requires a fresh observation.

`scripts/localization/anchors.json` remains a **manual board setup input only**.
Program measured coordinates with the setup tool, or use the BLE calibration below.
Reading coordinates from UWB does not detect physical movement of an anchor.
If an anchor moves, recalibrate before using room targets again.

### Dashboard anchor calibration

Either drone can host a Bluetooth calibration coordinator for four DWM1001 PANS anchors.
It triggers PANS inter-anchor ranging, fits XY using all six pairwise distances with
all anchor Z coordinates fixed at 0, previews the result, then writes and reads back each anchor.
The room map continues to use live tag reports only; there is no offline layout.
Calibration is an explicit grounded operation, not continuous tracking of moving anchors.

On each Pi that will provide calibration, rebuild the Docker image to install Bluetooth support, or install
these dependencies inside the existing ROS Kinetic container:

```bash
sudo apt-get update
sudo apt-get install bluez libglib2.0-dev libcap2-bin build-essential python-websocket
sudo python -m pip install bluepy==1.3.0
sudo setcap cap_net_raw,cap_net_admin+eip "$(python -c 'import os, bluepy; print(os.path.join(os.path.dirname(bluepy.__file__), "bluepy-helper"))')"
hciconfig
```

The Pi must have a working Bluetooth LE adapter visible inside the container
(`hci0` by default), with the host adapter powered and unblocked. The repository's
privileged container with host networking exposes host Bluetooth; a USB BLE adapter
can be used if the Pi adapter is unavailable. Close Qorvo's phone app and other BLE
connections to the anchors. All four anchors need BLE reachability from this Pi
and UWB line of sight between each other.

If scanning fails, run `hciconfig -a` inside the ROS container. It should list the
selected adapter (normally `hci0`). If it is down, run `sudo hciconfig hci0 up`;
if it is blocked, run `sudo rfkill unblock bluetooth` on the Pi host. If no adapter
appears, check the host Bluetooth setup or use a USB BLE adapter. A permission
error requires the `setcap` command above for the helper installed by this Python.

If an anchor advertises but cannot connect, close other BLE apps, power-cycle that
board and check BLE range from the Pi. Discovery tries the advertised address type
first, then the other type once. It skips tags and lists unreadable boards while
retaining reachable anchors; four verified anchors are still required to measure.

Rebuild the ROS workspace and restart the flight-controller and fusion nodes on
each participating drone to install their calibration interlock. Start its rosbridge,
tag and flight controller; keep it disarmed. One drone is enough. The **Calibration** Screen window
starts a coordinator on either drone without checking its hostname. For a running Screen session, start it manually on either Pi:

```bash
roslaunch pidrone_pkg anchor_calibration.launch
# For another Bluetooth adapter: append interface:=1 (hci1).
```

Bridge addresses default to `ws://drone1.local:9090` and `ws://drone2.local:9090`;
override `drone1_bridge` / `drone2_bridge` launch arguments if Pi-to-Pi name resolution
requires different addresses. Both coordinators may run: the dashboard selects an
available service and shows which drone it uses. Every reachable drone's flight controller must grant
that coordinator exclusive ownership before ranging or coordinate writes. A second
coordinator cannot release its locks. Update and restart participating flight controllers
before using this version; older controllers cannot grant ownership.

An idle service can be replaced automatically when it becomes unavailable. An active
calibration stays with its original service through review, saving and recovery;
reconnect that drone to finish or cancel it. No preview or incomplete save is transferred
to another Pi. The selected Pi must have Bluetooth reachability to all four anchors.

Only reachable rosbridges participate when measuring starts; an offline companion
is not required. A reachable companion must be disarmed even if its dashboard
connection is closed. Participants remain required until completion or cancellation,
so a dropped connection cannot bypass a lock or tag verification. If another drone
joins during calibration, measure again to include it before saving. The participant
list survives a coordinator process restart alongside the recovery state.

1. Connect one or both drones in the dashboard and open **Calibrate anchors → Scan anchors**.
2. Select four anchors in counterclockwise physical order. First is XY origin;
   second defines +X. The other two corners must be on the +Y side of this baseline.
   Mount all four antennas at the same height. Calibration treats their plane as
   Z = 0, so inter-anchor ranges are horizontal distances and no height entry is needed.
   Drone height still comes from ToF. With the default `floor_z:=0`, published Z is
   height above the floor. To express drone Z relative to an elevated anchor plane,
   set `floor_z` to minus the physical anchor height on both localization nodes;
   local controller height remains above the floor. The XY map uses only anchor XY.
   One selected anchor must be the active network initiator; the other three must
   be active anchors on the same PAN. Calibration does not configure these roles.
3. Press **Measure layout**. Participating flight controllers must acknowledge the flight
   inhibit before ranging starts. Check the preview against the real room and its
   dimensions. Distances alone cannot distinguish a mirrored physical arrangement;
   selecting the correct corner order is essential. A small fit residual is not an
   absolute accuracy guarantee.
4. Press **Save layout** within two minutes. The coordinator verifies BLE readback
   and waits for fresh reports of the same four saved coordinates from **each participating tag**
   before releasing flight inhibit. It rejects missing/low-quality ranges, inconsistent
   geometry and poor fits. If needed, move the grounded tags where they see all four anchors.
5. Recalibrate heading and restart fusion with the new measured `room_yaw_offset`
   on each drone. Starting measurement invalidates the previous heading even if you
   cancel, because the chosen origin and axes may change. ToF remains the Z source.

**Cancel** or an expired preview releases flight inhibit if no coordinate write
has started. A failed or partial save keeps it active: correct Bluetooth/tag reception
and **Measure layout → Save layout** again to recover. Recovery state survives a
coordinator process restart through ROS parameters while the ROS master stays up.
Do not reset interlock parameters to bypass recovery. After any complete system
restart, verify each participating tag's anchor coordinates before flight. Recalibrate
room heading on an offline companion before using it with the changed layout.

Request/status use `/pidrone/localization/calibration/request` and `/status` on
the selected drone (`std_msgs/String` JSON). Each master uses
`/pidrone/localization/calibration_owner/request` and `/status` (`std_msgs/String`
JSON) for exclusive ownership and acknowledgements identifying the request.
`/pidrone/localization/calibration_locked` (`std_msgs/Bool`) continues to report
flight inhibit to fusion and the dashboard. Legacy Boolean lock requests cannot
override a coordinator's ownership.
The implementation follows the [PANS API guide, section 7.2](https://forum.qorvo.com/uploads/short-url/jT6U3NQKKCCsDP0Zjh6QFMiZ3jW.pdf).
Its BLE payload handling and full workflow require validation on your flashed boards
before flight; the software tests use simulated BLE responses.

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
node --test src/web/tests/dashboard.test.js
```

Tests cover serial setup/reconnect/cleanup, delayed and fragmented fixes, invalid
numbers, ToF-only height, tilt/offset math, freshness/time skew, ROS publication,
diagnostics, live anchor parsing/discovery, partial/stale/conflicting layouts,
no duplicate publication, and independent dashboard telemetry.
Fusion checks also cover delayed observations, axis rotation, covariance/outlier
handling, camera gaps, room expiry, layout/time resets, bounded room commands,
manual overrides and explicit dashboard sending. Hardware-free tests need NumPy.

Before flight, with props removed, inspect `rostopic echo /pidrone/localization/pose`
and `/pidrone/localization/status`; inspect `/pidrone/localization/anchors` even
with ToF temporarily unavailable. Confirm all four IDs/coordinates against the
survey and compare reports from both drones. Vary height and tilt, obstruct the range beam,
and unplug/reconnect the tag. Then measure both tags together against surveyed
positions and validate relative separation under the real payload arrangement.
