# Camera driver correction

`subscriber-vectors.patch` preserves the deployed `raspicam_node` change against
revision `d4e1cd901da0db0728c7cbc0771a19aa73f0f18a`. It skips unused motion-vector
processing and handles the encoder's padding column. JPEG delivery is unchanged.

In the drone container, with the camera stopped:

```sh
cd ~/catkin_ws/src/raspicam_node
git apply --check ~/catkin_ws/psu-hri-capstone-f26/tools/camera/subscriber-vectors.patch
git apply ~/catkin_ws/psu-hri-capstone-f26/tools/camera/subscriber-vectors.patch
cd ~/catkin_ws
catkin_make --pkg raspicam_node install -j1
```

Restart the camera and localization while stationary on the floor facing +Y.
