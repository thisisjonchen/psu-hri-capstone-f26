# Instructions on Working with the DD24
Working with the DD24 can be hard :/

Here are some steps to make life with the DD24 a little simpler.


## Standard Operation + Control
Assuming that the base environment is already set up on the DD24 by following the instructions [here](https://docs.duckietown.com/daffy/opmanual-dd24/preliminaries/environment-setup.html), feel free to continue.

1. Turn the drone on
2. Enable a mobile hotspot. This can be done on a Windows laptop or via a phone hotspot, but ensure the following. Ensure that both the control base station and the drone are on the same network:
   1. Network name: **duckietown**
   2. Network password: **quackquack**
3. SSH onto the DD24 with the following command. Replace [drone] with drone1 or drone2.
   ```
   ssh duckie@[drone].local
   ```
4. Wait until the connection is established and you see a pop-up in the terminal for a password input.
   1. Password: **quackquack**
5. Run these commands in the terminal afterward:
   ```
   cd ~/catkin_ws/src/pidrone_pkg
   git pull
   rake start
   ```
6. Wait until the container starts. Then run:
   ```
   screen -wipe
   screen -c pi.screenrc
   ```
7. Your terminal should have changed to show a variety of different controls. Use `'# to navigate, where # is the screen you want to get to. Each screen corresponds to a different node/component on the drone; for example, $1FC is the flight controller.
8. Go through each screen (1-9) and ensure its processes are running. Then, proceed to the $1FC screen and run the flight_controller_node.py. 
10. To control it, on your laptop/base station, clone this repo. Open the website at `packages/pidrone_pkg/web/index.html`.
    1. When the interface opens, you have an option of which drone you want to connect to. Simply select either Drone1 or Drone2, click connect, and you are good to go.
11. Read all instructions carefully and be ready to disarm at all times.
12. Congratulations, the drone is now ready to fly!

Note: Everything done on the drone is through this `pidrone_pkg`. Add any custom scripts or code to `~/.../pidrone_pkg/scripts/PALS`.

## Changing Branches
If the repo has been cloned on the drone and you wish to change the branch the drone is currently on, do the following.
1. Follow the procedure above (Standard Operation + Control) **until Step 4**. You should be SSH'd and logged in.
2. Go to the directory where we have the repo: `cd ~/catkin_ws/src/pidrone_pkg`
3. You should have the branch you want to change to in mind. If you are unsure of the current branch of the drone, run `git branch --show-current`
4. To change to the branch you want, do `git checkout -f [branch]`
5. You are ready to go! Feel free to test as needed. If you need to modify any code, do so on your laptop/base station, push to the branch, then pull.
6. Continue from **Step 5** in **Standard Operation + Control**.
   
   
## Cloning the Repo
If the DD24 does not have the repo cloned yet, here are instructions for doing so.

1. Follow the procedure in **Standard Operation + Control**, **up to Step 4**. You should be SSH'd and logged in.
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
   ln -s ~/catkin_ws/psu-hri-capstone-f26/packages/pidrone_pkg \
      ~/catkin_ws/src/pidrone_pkg
   ```
6. Check the symlink works
   ```
   ls -l ~/catkin_ws/src/pidrone_pkg
   readlink -f ~/catkin_ws/src/pidrone_pkg
   ```
7. You are now ready to go. Take a look at **Changing Branches**. Otherwise, follow the rest of **Standard Operation + Control** from **Step 5**.

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

## Tips + Tricks
- Use `d + [enter] to exit out of a screen session
- If you come across an issue with the screen session, run:
   ```
   screen -ls
   screen -S <session-id> -X quit
   cd ~/catkin_ws/src/pidrone_pkg
   screen -c pi.screenrc
   ```