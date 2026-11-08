# ToF driver correction

`driver-errors.patch` fixes incomplete I²C transfers, stops startup at the first
sensor error with a nonzero exit status, and checks acquisition and interrupt
acknowledgement errors. It targets `vl53l1x_ros` revision `cd4960c`; ranging settings
remain unchanged.

Inside the drone's `pidrone_pkg` container, with ToF stopped, reverse any earlier
version of this patch before applying the current version:

```sh
cd ~/catkin_ws/src/vl53l1x_ros
git apply --check ~/catkin_ws/psu-hri-capstone-f26/tools/tof/driver-errors.patch
git apply ~/catkin_ws/psu-hri-capstone-f26/tools/tof/driver-errors.patch
cd ~/catkin_ws
catkin_make install --pkg vl53l1x -j1
```

Restart ToF and localization while disarmed, stationary on the floor facing +Y.
The correction does not repair a faulty sensor connection or identify obstructions
that the sensor reports as valid.
