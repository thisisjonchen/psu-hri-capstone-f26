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
3. Place each drone disarmed, stationary, and facing the top of the map (+Y).
4. Click **Connect** in the drone's panel. This sets up SSH if needed, starts its nodes, and opens its terminal. Repeat for the other drone if needed.
5. Under **Routines**, select a script, enter its arguments, and click **Run**.
6. Read all instructions carefully and be ready to disarm at all times.

Note: Everything done on the drone is through this `pidrone_pkg`. Add routines to `scripts/PALS/routines` to make them available in the dashboard.

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

## Yellow Payload Detection

The onboard `payload_detection` node starts with the normal dashboard connection.
It reads `/raspicam_node/image/compressed` and publishes
`/pidrone/payload/detection` (`pidrone_pkg/PayloadDetection`). Each drone's camera
panel shows a yellow bounding box and center dot, with separate payload and tag
status. This is image-based detection only: no distance, room position, payload
orientation, or flight commands are produced. Existing localization and AprilTag
heading correction remain active.

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
generate the new message before starting nodes. For a separate perception-only
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
