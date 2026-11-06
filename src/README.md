# Instructions on Working with the DD24
Working with the DD24 can be hard :/

Here are some steps to make life with the DD24 a little simpler.


## Standard Operation + Control
Assuming that the base environment is already set up on the DD24 by following the instructions [here](https://docs.duckietown.com/daffy/opmanual-dd24/preliminaries/environment-setup.html), feel free to continue.

1. Turn the drones on and connect them and your laptop to the same hotspot:
   - Network name: **duckietown**
   - Network password: **quackquack**
2. Start the dashboard:
   ```
   python3 tools/pals.py dashboard
   ```
   Open `http://localhost:8000` if it does not open automatically.
   Run the command again to replace an existing dashboard on the same port; restart while disarmed.
3. Place each drone disarmed on a level floor, stationary and facing the top of the map (+Y).
4. Click **Connect** in the drone's panel. This sets up SSH if needed, starts its nodes, and opens its terminal. Repeat for the other drone if needed.
5. Keep it still for at least eight seconds while the EKF initializes; wait for localization readiness. Under **Routines**, select a script, enter its arguments, and click **Run**.
6. Read all instructions carefully and be ready to disarm at all times.

Note: Everything done on the drone is through this `pidrone_pkg`. Add routines to `scripts/PALS/routines` to make them available in the dashboard.

**Separation guard** defaults off onboard and in the dashboard, bypassing spacing
corrections and peer checks for both drones. Confirm
**Disabled on drone** in each status panel before flight. Local estimate checks and
disarm/watchdog behavior remain active. Re-enable it under **Controls** while the
connected drones are disarmed. The dashboard resets the switch off when reopened.

For the complete firmware, driver and code rollout, see the
[drone update guide](../tools/DRONE_UPDATE.md).

## Betaflight telemetry setup

The current FC node and EKF require the bundled `imu100001` firmware. Update the
firmware and ROS package together; the older `usb2ms001` build does not supply
integrated IMU telemetry.

To install the firmware and save the 250 Hz serial setting together,
run from the laptop with the drone disarmed and props removed:

```sh
python3 tools/pals.py flash-fc --drone drone1
```

Use `--drone drone2` for drone2. The command backs up the selected drone, preserves
its configuration, and verifies the update. See the
[firmware instructions](../tools/firmware/betaflight/README.md#apply-to-a-drone) for
supported versions and the Pi-side code needed for 100 Hz IMU delivery.

Drone2 uses a SpeedyBee F405 V3 with a custom Betaflight 2026.6.1 build
(`imu100001`, based on `6dbc4218f`, board config `749fff1`, MSP API 1.48).
The firmware flushes USB replies every 2 ms. Its saved MSP serial service rate
is 250 Hz:

```text
set serial_update_rate_hz = 250
save
```

Apply this in the Betaflight CLI while disarmed, with the Pi's flight-controller
node stopped so it releases the USB port. `save` persists the setting and reboots
the flight controller. After reconnecting, verify it with
`get serial_update_rate_hz`, then restart the Pi's flight-controller node.

Drone2's PID profile 0 also uses `vbat_sag_compensation = 100` for battery-voltage
compensation. This changes motor-output scaling and requires hover-throttle
verification. The flash command preserves this setting; it does not enable it.

The serial setting controls how often Betaflight services serial traffic. The
USB interval is a compiled firmware change, preserved in the
[patch and build instructions](../tools/firmware/betaflight/README.md). Flashing a stock
image removes that change; restoring the CLI settings alone does not restore it.

The Pi requests integrated rotation and specific force at 100 Hz for
`/pidrone/imu_integrated`, with FC interval timestamps mapped into ROS time.
`/pidrone/imu_msp` retains current sensor counts stamped at Pi receipt; the existing
`/pidrone/imu` attitude messages and RC commands run at 60 Hz. These settings do not
change gyro sampling or PID loop rates. They are applied to drone2; drone1 is unverified.

## EKF recording and replay

Build the C++ EKF node onboard with `catkin_make` and source the workspace. It projects
each fresh IMU sample for output while a worker handles delayed camera/ToF fusion.
Both paths consume integrated IMU increments directly. Splitting an interval assumes
constant rotation rate and uniformly distributed delta velocity in its starting axes.
The Python replay shares `libpidrone_ekf.so`. To build that library on the laptop:

```sh
cmake -S src/native -B .venv/ekf-native
cmake --build .venv/ekf-native
```

Add `.venv/ekf-native` to `LD_LIBRARY_PATH` (`DYLD_LIBRARY_PATH` on macOS)
before running Python.

Record from the laptop with props removed:

```sh
python3 tools/flow_capture.py ekf --drone drone2 --seconds 40
```

Captures require IMU, JPEGs, camera calibration and ToF, and include published
estimates. Data is buffered on the Pi, downloaded, then removed there. Allow about
70 MB for a 40-second capture. IMU timestamps mark host receipt; camera timestamps
mark driver callbacks, not sensor acquisition.
`camera_delay_s` subtracts the camera's delay relative to IMU timestamps during
fusion; it leaves recorded timestamps unchanged.

Fill in the [replay calibration profile](../tools/ekf_calibration.example.json), then run:

```sh
.venv/bin/python tools/replay_ekf.py CAPTURE --calibration PROFILE.json --out NEW_FOLDER
```

The profile supplies sensor geometry, scales and initial state. Use
`stationary_intervals_s` only for known periods without translation or rotation;
leave it empty when rest is unknown. Replay does not affect flight control.
For older captures containing `motion_vectors_raw`, use `--camera encoder`.
New captures use JPEGs and omit encoder vectors.

## Changing Branches
Open **Branches** next to GitHub, select a published branch, then **Switch**.
Both drones must be reachable, report DISARMED, and have no running routine or local
changes. Save or stash local changes in their terminals, or use **Clear local changes**
to discard uncommitted changes and untracked files on both drones after confirmation.
Clearing keeps commits and ignored files. Switching fetches origin,
updates each tracking branch without discarding commits, and leaves nodes running.
Rebuild if native code changed, then use **Restart** with props removed, stationary
on the floor facing +Y. **Refresh** shows each drone's current branch and revision.
   
   
## Cloning the Repo
If the DD24 does not have the repo cloned yet, here are instructions for doing so.

1. `ssh duckie@[drone].local` (password **quackquack**).
2. Run the following commands to clone the repo
   ```
   cd ~/catkin_ws && git clone https://github.com/thisisjonchen/psu-hri-capstone-f26.git
   ```
3. Delete the old `pidrone_pkg`
   ```
   rm -rf ~/catkin_ws/src/pidrone_pkg
   ```
4. Ensure that it is gone (should say "No such file or directory")
   ```
   ls ~/catkin_ws/src/pidrone_pkg
   ```
5. Create a symlink from our repo as the new `pidrone_pkg`
   ```
   ln -s ~/catkin_ws/psu-hri-capstone-f26/src \
      ~/catkin_ws/src/pidrone_pkg
   ```
6. Check the symlink works
   ```
   ls -l ~/catkin_ws/src/pidrone_pkg
   readlink -f ~/catkin_ws/src/pidrone_pkg
   ```
7. You are now ready to go. Continue from **Step 2** in **Standard Operation + Control**.

## Yellow Payload Detection

The onboard `payload_detection` node starts with the normal dashboard connection.
It reads `/raspicam_node/image/compressed` and publishes
`/pidrone/payload/detection` (`pidrone_pkg/PayloadDetection`). Each drone's camera
panel shows a yellow bounding box and center dot, with separate payload and tag
status. The detector reports image coordinates.

The onboard AprilTag mapper projects the yellow center onto a fixed height and
publishes `/pidrone/payload/map_point`. Set `payload_plane_z` in
`params/apriltag_mapping.yaml` to the height of the visible yellow surface in
the map when the payload is resting on the floor. The default 0.0 assumes that
surface is at floor level. The mapper uses the same camera calibration, mounting
and fresh fused pose as the floor tags. The dashboard shows a yellow diamond from
the most recently received usable estimate and hides it when observations expire.
This marker is for display and assumes the payload is at the configured height.

`params/payload_detection.yaml` contains starting HSV thresholds and minimum image
area. Tune these using actual camera images of the payload under the zone's
lighting, including shadows and reflections. HSV hue uses OpenCV's 0..179 scale;
saturation and brightness use 0..255. The node initially chooses the largest
qualifying yellow region, then associates nearby regions across frames. After a
short gap it can acquire the largest region again. Similar yellow objects can be
mistaken for the payload; this does not establish unique identity.

Detection results retain the source image's timestamp and frame. Missing, stale,
invalid, or non-increasing image timestamps clear the observation. A fresh image
without a candidate reports no detection immediately; association history is
retained only for `track_timeout`. The dashboard also expires results when their
image timestamp stops advancing. Its overlay and MJPEG video arrive separately,
so a moving box may lag the video; the overlay is a display aid.

After these changes are copied to a drone, rebuild the catkin workspace to
generate the ROS messages before starting nodes. For a separate perception-only
session with an existing camera and ROS master, use:

```sh
roslaunch pidrone_pkg payload_detection.launch
```

Check visibility and tune color thresholds while disarmed. The current camera
stream and OpenCV dependency are reused; no extra camera or trained model is
required. Camera mounting and lighting still determine whether the payload can
be detected throughout transport.

## Camera Out of Focus

The camera may be out of focus. Here are some steps to fix that.
1. Follow **Standard Operation + Control**. You should have the drone connected to the interface and see a camera image.
2. Take a look at the drone's camera: you should see a black ring. Twist it until it comes into focus, but not too much that it becomes blurry again.
3. Congrats, you focused the camera!
4. If you need the camera to be calibrated, ask Jon

NOTE: IF CAMERA IS CALIBRATED, DO NOT TOUCH THE FOCUS RING.

## UWB + ToF Localization

Start each drone facing the top of the map (+Y) before starting its flight controller.

When calibrating:
- 4D12: Tag 6
- 980C: Tag 5
- 9781: Tag 4
- CA83: Tag 3

## Tips + Tricks
- Use `d + [enter] to exit out of a screen session
- If you come across an issue with the screen session, run:
   ```
   screen -ls
   screen -S <session-id> -X quit
   cd ~/catkin_ws/src/pidrone_pkg
   screen -c pi.screenrc
   ```

## To Install Python Dependencies on Drone
1. Add to `requirements.txt`
2. SSH and start container on `/catkin_ws/src/pidrone_pkg` by using `rake start`
3. Run `python -m pip install -r requirements.txt`
