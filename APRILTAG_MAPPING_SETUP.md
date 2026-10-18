# Map floor AprilTags into the UWB coordinates (DD24 / PALS)

## What changes

The existing `apriltag_detection_drone` node keeps publishing `TagDetection` to the
web dashboard. It also includes each tag's pixel center and image dimensions.
A separate `apriltag_mapping_node` subscribes to:

| Topic | ROS type | Use |
| --- | --- | --- |
| `/pidrone/apriltag/detections` | `pidrone_pkg/TagDetection` | Pixel center of the tag in the original camera frame |
| `/raspicam_node/camera_info` | `sensor_msgs/CameraInfo` | Measured camera intrinsics and lens distortion |
| `/pidrone/localization/fused` | `geometry_msgs/PoseWithCovarianceStamped` | UWB X/Y, ToF Z and IMU-derived room orientation |
| `/pidrone/localization/ready` | `std_msgs/Bool` | Don't map without verified room pose |
| `/pidrone/localization/generation` | `std_msgs/UInt32` | Erase tag map when UWB room definition changes |

It publishes `/pidrone/apriltag/map` (`pidrone_pkg/TagMap`), a collection of
observed centers in `map` meters, read by `src/web/js/main.js` on both independent
ROS masters. Tags are visible after 3 *different image frames* with acceptable
measurements. Positions average the latest `min_tag_samples` consistent frames
(3 by default), so older observations cannot pin a tag to its original location.
A jump beyond `max_tag_jump` (0.35 m by default) starts a new candidate cluster;
the previous marker remains until the new cluster has enough observations.
Gaps longer than `max_image_pose_skew` reset the observation window but retain
the last confirmed marker. Tags must remain on the configured horizontal plane.

A mapping observation **does not** modify the flight controller, UWB localization,
or target behavior. If both drones report the same ID, the dashboard uses the
most recently changed estimate received in that browser session. An unchanged
cached tag in another snapshot does not take precedence. This avoids averaging
a moved tag back toward its old location and does not compare unsynchronized
drone clocks. It is display ordering, not a per-tag measurement timestamp;
after reconnecting, latched reports cannot establish which drone observed it last.

## Geometry and critical assumptions

Frames: `map` is UWB +X/right, +Y/up on dashboard, +Z/up. `body` uses
+x/right, +y/forward, +z/up, matching the existing controller's fused
quaternion. OpenCV `camera` uses +x/image-right, +y/image-down, +z/optical
forward. For an image measurement `u,v`:

1. The published CameraInfo distortion model selects the pixel-to-ray conversion:
   `cv2.undistortPoints()` for `plumb_bob`/`rational_polynomial`, or
   `cv2.fisheye.undistortPoints()` for `equidistant` with four coefficients.
   Both produce the camera ray `[x,y,1]`. Equidistant calibration requires
   OpenCV's Python fisheye API; it cannot be used as pinhole coefficients.
2. The mounting rotation converts that ray to body coordinates. For a camera
   pointing **60 degrees below horizontal** and straight forward:
   `R_body_camera = [[1,0,0],[0,-sin(60),cos(60)],[0,-cos(60),-sin(60)]]`.
3. The map pose quaternion rotates the ray and measured camera offset into map.
4. Intersect ray with horizontal plane `z=tag_plane_z`:
   `P_tag = P_camera + ((tag_plane_z-P_camera.z)/ray_map.z) * ray_map`.

**This approach assumes each tag is laid flat on the same horizontal surface.**
A wall-mounted tag, a tilted tag, or an unknown tag height requires a
full 6-DOF solution such as calibrated AprilTag corners plus solvePnP and a
known tag size instead of intersecting a floor plane. The method also assumes
that UWB anchors are properly surveyed and the fusion yaw is aligned. It does
not repair drift of that heading after startup.

For a perfectly forward-looking 60°-down camera, an image-center tag viewed
from 1.00 m above the tag plane intersects approximately **0.577 m forward**
of the camera's ground projection, *not* directly underneath the drone.

## Required real-world calibration (do before treating numbers as measurements)

1. **Confirm angle convention.** `60` in `src/params/apriltag_mapping.yaml`
   means 60° **below horizontal**. If 60° means from straight down, use `30`.
   Set `camera_azimuth_deg` if the camera tilts toward rear/right/left.
2. **Measure camera lever arm** (optical center relative to the UWB position
   point) and enter `[right, forward, up]` in `camera_offset_body` in **meters**.
   The `[0,0,0]` default is a placeholder, not a measured offset.
3. **Calibrate camera** at the exact ROS frame width, height and optical
   orientation used in flight. `src/launch/raspicam_node.launch` currently
   requests width=240, height=320 and loads its camera_info from
   `package://raspicam_node/camera_info/camera_dd24.yaml`. The **unused**
   `src/params/picamera.yaml` says width=320, height=240: do not silently
   swap it in. Confirm the actual compressed frame size and CameraInfo:

   ```bash
   rostopic echo -n 1 /raspicam_node/camera_info
   rostopic echo -n 1 /raspicam_node/image/compressed/header
   ```

   If camera_info is unavailable, uncalibrated (K fx/fy zero), or its width/
   height differs from the decoded frame, create a genuine camera calibration
   file with ROS's camera_calibration tool for the **actual** camera/capture
   resolution. Point `camera_info_url` in `src/launch/raspicam_node.launch`
   to that file, then restart the camera node. Do **not** invent intrinsics.
   Verify the image x/y axes with a tag placed forward and to the drone's right.
4. **Align heading**. The existing fusion node requires the drone to be
   stationary, disarmed, and facing `map +Y` at startup. A drift in that yaw
   also rotates the calculated tag coordinates. Verify against a tape-measured
   tag position before using mapped points for any planning.
5. **Use a floor plane in the same UWB frame**. Default `tag_plane_z: 0.0`
   only fits if `z=0` is the surface carrying the tags. If the floor is at a
   different map Z (e.g. the UWB anchors define an elevated origin), set
   `tag_plane_z` to the *measured* plane height in meters.

## Build, launch and observe

Copy the changed/additional files onto **both** drones. Rebuild the `pidrone_pkg`
messages first (the `TagDetection` ROS message has new fields and a `TagMap`
message has been added). Do not launch an old running node with new message types.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
# Start the existing nodes as you normally do, with drone disarmed
roslaunch pidrone_pkg apriltag_detection.launch
# In another terminal, once UWB, fusion and camera are running:
roslaunch pidrone_pkg apriltag_mapping.launch
```

The normal `screen -c pi.screenrc` startup runs the mapper in the **TagMap**
window, alongside the **AprilTags** detector window. `PALS/drone_controller.py`
also starts the mapper if it is missing. With either startup flow, no extra
manual launch is needed. For an already-running screen session that predates
this change, launch `roslaunch pidrone_pkg apriltag_mapping.launch` in a free
window after rebuilding and sourcing the workspace. Check topics from the same
ROS master on the drone:

```bash
rostopic echo -n 1 /pidrone/apriltag/detections
rostopic echo /pidrone/apriltag/map
rostopic echo /pidrone/localization/ready
rostopic echo -n 1 /pidrone/localization/fused
```

**Ground test first:** with motors disarmed and a valid fused localization
pose, position a stationary drone so a paper AprilTag on the floor is visible,
then gently point/position the camera toward the tag. Compare the detected
map coordinates to a tape measure. Do not arm, take off, or use coordinates to
command motion just to test this feature. Image/pose pairing is limited by
`max_image_pose_skew` (default 0.35 seconds). A pose may be newer or older than
the image because detection takes time; both must still be fresh. The fused
stamp tracks contributing sensor age, not a synchronized camera-time pose,
so drone movement while mapping will increase error. If you map while flying,
hover briefly and treat the result as a provisional landmark until validated.

On the dashboard a green diamond and text label for each mapped tag will
appear. The standard status card continues to report only *visibility* of a
tag. Map coordinates are held in memory on each drone while the ROS mapping
node is running; they are not saved to a permanent file. A UWB room-generation
change clears the map. Camera offline does not turn a previously measured
location into a fresh observation. Restarting the mapping node clears it.

## Files modified

- `src/msg/TagDetection.msg` — add center pixel and image dimensions.
- `src/scripts/PALS/apriltag_detection_drone` — populate those fields.
- `src/CMakeLists.txt` — generate `TagMap.msg`; install mapping node.
- `src/scripts/PALS/drone_controller.py` — auto-start mapper with other nodes.
- `src/web/js/main.js` — subscribe to per-drone tag maps, aggregate and plot.
- `src/web/index.html` — add legend entry for tag markers.
- `src/web/tests/test_detector.py` — assert new detection fields.
- `src/web/tests/dashboard.test.js` — verify map display/generation behavior.

## Files added

- `src/msg/TagMap.msg` — ROS format for coordinates, IDs, labels and counts.
- `src/scripts/PALS/apriltag_mapping_node` — ROS node that assembles a room map.
- `src/scripts/localization/apriltag_geometry.py` — no-ROS camera geometry.
- `src/scripts/localization/test_apriltag_geometry.py` — unit tests for geometry.
- `src/params/apriltag_mapping.yaml` — physically measured camera parameters.
- `src/launch/apriltag_mapping.launch` — starts mapper.
- `APRILTAG_MAPPING_SETUP.md` — this guide.

## Caveats

- The ROS message changes require `catkin_make` on *both drones*; do not
  reuse old rosbridge message registrations without restarting those nodes.
- The optical center and image rotation depend on the actual mounting; a
  nominal 60° alone does not guarantee centimeter accuracy.
- The heading is only initially aligned in the existing fusion code;
  yaw drift cannot be corrected solely from previously unknown tags.
- A floor ray/plane intersection is cheaper than AprilTag pose estimation and
  requires no measured tag size, but is **not** a general 3D AprilTag mapper.
- The UWB room should be based on surveyed anchor positions, not invented
  corner values in example `anchors.json`.
