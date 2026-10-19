# Instructions on Working with the DD24
Working with the DD24 can be hard :/

Here are some steps to make life with the DD24 a little simpler.


## Standard Operation + Control
Assuming that the base environment is already set up on the DD24 by following the instructions [here](https://docs.duckietown.com/daffy/opmanual-dd24/preliminaries/environment-setup.html), feel free to continue.

1. Turn the drones on and connect them and your laptop to the same hotspot:
   - Network name: **duckietown**
   - Network password: **quackquack**
2. For first-time setup, clone this repo on your laptop and run these commands from the repository root.
   ```
   python3 tools/pals.py ssh-setup
   python3 tools/pals.py provision
   ```
   Use `--drone drone1` to set up only one drone.
3. Start the dashboard:
   ```
   python3 tools/pals.py dashboard
   ```
   Open `http://localhost:8000` if it does not open automatically.
4. Place each drone disarmed, stationary, and facing the top of the map (+Y).
5. Click **Connect** in the drone's panel. This sets up SSH if needed, starts its nodes, and opens its terminal. Repeat for the other drone if needed.
6. For paired flight, connect both drones before arming. The dashboard server automatically relays peer telemetry once both ROS connections are available.
7. Under **Routines**, select a script, enter its arguments, and click **Execute**. For `prototype-1-forward.py`, use `--fly --distance 0.25` on both drones to take off, move forward 0.25 m, and begin landing together. See [coordination](scripts/PALS/COORDINATION.md) for paired-flight details.
8. Read all instructions carefully and be ready to disarm at all times. Press Esc to leave a terminal, then Space to disarm all connected drones.

Use **Restart nodes** after updating code, or **Align +Y** to re-align heading. Keep the drones disarmed, stationary, and facing +Y for either action.

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

## Camera Out of Focus

The camera may be out of focus. Here are some steps to fix that.
1. Follow **Standard Operation + Control**. You should have the drone connected to the interface and see a camera image.
2. Take a look at the drone's camera: you should see a black ring. Twist it until it comes into focus, but not too much that it becomes blurry again.
3. Congrats, you focused the camera!
4. If you need the camera to be calibrated, ask Jon

NOTE: IF CAMERA IS CALIBRATED, DO NOT TOUCH THE FOCUS RING.

## AprilTag Detector Installation
To allow AprilTag detection to work, we will have to install some dependencies.


1. Run the following
```
cd ~/catkin_ws/src/pidrone_pkg
python -m pip install --user -r scripts/PALS/requirements.txt
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
2. Done!

## UWB + ToF Localization

Start each drone facing the top of the map (+Y) before starting its flight controller.

When calibrating:
- 4D12: Tag 6
- 980C: Tag 5
- 9781: Tag 4
- CA83: Tag 3

Start the dashboard on your laptop:

```bash
python3 tools/pals.py dashboard
```

Run from the repository root. The dashboard opens at `http://localhost:8000`.

On Windows, install Python 3.11 or newer and run from PowerShell:

```powershell
py -3 tools/pals.py dashboard
```

The launcher creates `.venv` and installs dependencies. Clicking **Connect** sets up
SSH using the default password (`quackquack`) when needed; existing keys are reused.
If a drone is offline, click **Connect** again once it is online. Bluetooth calibration
requires a Bluetooth LE adapter. The drones must be reachable as `drone1.local`
and `drone2.local` on the same network. The peer relay runs automatically with the dashboard.

## Tips + Tricks
- Use `d + [enter] to exit out of a screen session
- If you come across an issue with the screen session, run:
   ```
   screen -ls
   screen -S <session-id> -X quit
   cd ~/catkin_ws/src/pidrone_pkg
   screen -c pi.screenrc
   ```
