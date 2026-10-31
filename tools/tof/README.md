# ToF driver correction

`driver-errors.patch` fixes incomplete I²C transfers, checks acquisition errors,
and discards results during interrupt-acknowledgement recovery. It targets
`vl53l1x_ros` revision `cd4960c`; ranging settings remain unchanged.

Inside the drone's `pidrone_pkg` container, with ToF stopped:

```sh
cd ~/catkin_ws/src/vl53l1x_ros
git apply --check ~/catkin_ws/psu-hri-capstone-f26/tools/tof/driver-errors.patch
git apply ~/catkin_ws/psu-hri-capstone-f26/tools/tof/driver-errors.patch
cd ~/catkin_ws
catkin_make --pkg vl53l1x install -j1
```

Restart ToF and localization while disarmed, stationary on the floor facing +Y.
The correction does not identify obstructions that the sensor reports as valid.
