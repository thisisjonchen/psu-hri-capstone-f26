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
3. Place each drone disarmed on a level floor, stationary and facing the top of the map (+Y).
4. Click **Connect** in the drone's panel. This sets up SSH if needed, starts its nodes, and opens its terminal. Repeat for the other drone if needed.
5. Keep it still for at least eight seconds while the EKF initializes; wait for localization readiness. Under **Routines**, select a script, enter its arguments, and click **Run**.
6. Read all instructions carefully and be ready to disarm at all times.

Note: Everything done on the drone is through this `pidrone_pkg`. Add routines to `scripts/PALS/routines` to make them available in the dashboard.

## Betaflight telemetry setup

To install the USB timing firmware and save the 250 Hz serial setting together,
run from the laptop with the drone disarmed and props removed:

```sh
python3 tools/pals.py flash-fc --drone drone1
```

Use `--drone drone2` for drone2. The command backs up the selected drone, preserves
its configuration, and verifies the update. See the
[firmware instructions](../tools/firmware/betaflight/README.md#apply-to-a-drone) for
supported versions and the Pi-side code needed for 100 Hz IMU delivery.

Drone2 uses a SpeedyBee F405 V3 with a custom Betaflight 2026.6.1 build
(`usb2ms001`, based on `6dbc4218f`, board config `749fff1`, MSP API 1.48).
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

The serial setting controls how often Betaflight services serial traffic. The
USB interval is a compiled firmware change, preserved in the
[patch and build instructions](../tools/firmware/betaflight/README.md). Flashing a stock
image removes that change; restoring the CLI settings alone does not restore it.

The Pi requests fresh MSP accelerometer and gyro counts for `/pidrone/imu_msp`
at 100 Hz, while the existing `/pidrone/imu` attitude messages and RC commands
run at 60 Hz. IMU message timestamps mark reception on the Pi, not acquisition
inside the flight controller. These delivery settings do not change the gyro
sampling or PID loop rates. They have been applied to drone2; drone1 is unverified.

## EKF recording and replay

Record from the laptop with props removed:

```sh
python3 tools/flow_capture.py ekf --drone drone2 --seconds 40
```

Captures require IMU, JPEGs, camera calibration and ToF, and include published
estimates. Data is buffered on the Pi, downloaded, then removed there. Allow about
70 MB for a 40-second capture. IMU timestamps mark host receipt; camera timestamps
mark driver callbacks, not sensor acquisition.

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
If the repo has been cloned on the drone and you wish to change the branch the drone is currently on, do the following.
1. Use the drone's dashboard terminal, or `ssh duckie@[drone].local` (password **quackquack**).
2. Go to the directory where we have the repo: `cd ~/catkin_ws/src/pidrone_pkg`
3. You should have the branch you want to change to in mind. If you are unsure of the current branch of the drone, run `git branch --show-current`
4. To change to the branch you want, do `git checkout -f [branch]`
5. You are ready to go! Feel free to test as needed. If you need to modify any code, do so on your laptop/base station, push to the branch, then pull.
6. Restart the nodes while disarmed and facing +Y: `bash ~/catkin_ws/src/pidrone_pkg/boot/pals-start.sh restart`.
   
   
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
