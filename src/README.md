# Instructions on Working with the DD24
Working with the DD24 can be hard :/

Here are some steps to make life with the DD24 a little simpler.


## Standard Operation + Control
Assuming that the base environment is already set up on the DD24 by following the instructions [here](https://docs.duckietown.com/daffy/opmanual-dd24/preliminaries/environment-setup.html), feel free to continue.

### One-time setup (per laptop, then per drone)
Both the laptop and the drones must be on the same hotspot (network **duckietown**, password **quackquack**). From the repository root on your laptop (macOS, Linux or Windows, Python 3.8+):

```
python3 tools/pals.py ssh-setup   # creates ~/.ssh/pals_ed25519 and installs it; asks for the drone password once
python3 tools/pals.py provision   # pins serial ports, pip install + catkin_make; does not start nodes
```
Use `--drone drone1` to target one drone. After provisioning, `ssh drone1` works without a password.

### Every session
1. Turn the drones on. Nothing starts at boot.
2. Start the dashboard: `python3 tools/pals.py dashboard` (creates `.venv` and installs requirements the first time; opens `http://localhost:8000`).
3. Place each drone disarmed, stationary and **facing map +Y**.
4. Select Drone1 or Drone2 and click **Connect** (repeat for the other drone). Connect opens that drone's terminal in **Routines** and starts the `pidrone_pkg` container and the `pidrone` screen session with every node, including the flight controller; fusion aligns its heading now. The startup output appears in the terminal, then the terminal attaches to the session's `routine` window, which checks each node (`ok` or `MISSING`, with the window to inspect). Telemetry connects once rosbridge is up. Connecting again while nodes run changes nothing; **Disconnect** closes the terminal too (nodes and routines keep running).
5. Pick a script, set each drone's arguments (default `--fly`), and click **Execute**. Each routine waits disarmed; the dashboard sends start releases after every selected drone reports fresh readiness and all launch requests succeed. A failed launch, disconnect, Stop or Disarm cancels a pending release. Ctrl-C in a terminal interrupts that drone's routine (it lands). Use <kbd>`</kbd> + number to switch screen windows (e.g. FC, UWB). Esc leaves the terminal so Space disarms all again.
6. Read all instructions carefully and be ready to disarm at all times.

To re-align after a drone is moved, face it along +Y and click **Align +Y**: only fusion restarts, and it is refused unless the drone is DISARMED and no routine is running. **Restart nodes** restarts everything (e.g. after updating code).

A terminal's own toggle opens it without starting nodes. Each button's result appears under that drone's terminal header. Node output stays in the drone's screen windows inside the terminal (<kbd>`</kbd> <kbd>3</kbd> = SE/fusion, which logs `Startup heading aligned with map +Y`).

Closing the tab or losing Wi-Fi only detaches a terminal; a running routine continues and lands itself.

Synchronized start is a best-effort readiness barrier, not an identical takeoff
timestamp or synchronization of later route stages. A routine started manually
also waits for a dashboard release and times out after 30 seconds without one.
For paired flight, connect both drones before arming and start the laptop
`tools/peer_relay.py` separately. Distance control retains its 0.75 m target and
the solo/paired choice made before arming; see [coordination](scripts/PALS/COORDINATION.md).

### Updating code on the drones
Push your changes to GitHub, then in each drone's terminal run `cd ~/catkin_ws/src/pidrone_pkg && git pull` and click **Restart nodes** (drones disarmed and facing +Y, since restarting re-aligns heading). Run `catkin_make` in `~/catkin_ws` first if messages or build files changed. Don't edit code on the drones.

### Manual fallback
`ssh duckie@[drone].local`, then `docker exec -it pidrone_pkg screen -x pidrone` to attach to the nodes (`bash ~/catkin_ws/src/pidrone_pkg/boot/pals-start.sh start` starts them). On an unprovisioned drone, `rake start` followed by `screen -c pi.screenrc` still works; the FC window now starts the flight controller automatically.

Note: Everything done on the drone is through this `pidrone_pkg`. Add any custom scripts or code to `~/.../pidrone_pkg/scripts/PALS`; routines in `scripts/PALS/routines` appear in the dashboard's script list.

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
7. You are now ready to go. Run the one-time setup in **Standard Operation + Control**.

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
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r src/web/requirements.txt
python src/web/server.py
```

Run these commands from the repository root, then open `http://localhost:8000`.

## Tips + Tricks
- Use `d + [enter] to exit out of a screen session
- If you come across an issue with the screen session, run:
   ```
   screen -ls
   screen -S <session-id> -X quit
   cd ~/catkin_ws/src/pidrone_pkg
   screen -c pi.screenrc
   ```
