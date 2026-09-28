# PiDrone web dashboard

From the repository root, start a local static server:

```sh
cd packages/pidrone_pkg/web
python3 -m http.server 8000
```

Open <http://localhost:8000>, choose **Drone 1** (`duckie@drone1.local`) or **Drone 2** (`duckie@drone2.local`), and select **Connect**. Switching drones disconnects the current session; select **Connect** again for the new drone. The dashboard expects its ROS bridge on port 9090; start it with `roslaunch rosbridge_server rosbridge_websocket.launch` if needed. For live camera video, start `rosrun web_video_server web_video_server` on the drone (port 8080). The JavaScript and CSS dependencies are included here; no package install or build step is needed.
