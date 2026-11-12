# Bring drone1 up to the shared configuration

Use the same committed revision on the laptop and drone1. Remove props and keep
drone1 disarmed throughout the update. Preserve its serial-port assignments,
receiver configuration and hardware calibration; do not apply the old SpeedyBee
configuration dump or copy drone2's complete Betaflight configuration.

| Component | What to apply |
| --- | --- |
| PALS repository | Current ROS messages, C++ EKF, FC/PID code, launch files, shared parameters and warm restart helper |
| Betaflight | Bundled `imu100001`: integrated IMU telemetry and 2 ms USB flush; installer saves 250 Hz serial service |
| ToF driver | [`driver-errors.patch`](tof/driver-errors.patch): reject failed reads and interrupt-recovery results |
| Laptop dashboard | Current web files and root `requirements.txt`, including `psutil` |

## Apply in this order

1. Commit and sync the intended repository changes first. On drone1, inspect
   `git status` in `~/catkin_ws/psu-hri-capstone-f26` and reconcile local changes
   before updating to that revision. Verify `~/catkin_ws/src/pidrone_pkg` points
   to its `src` directory. Do not force-reset unknown changes.

2. Stop the onboard nodes while disarmed, leaving the container running. Inside
   the `pidrone_pkg` container, apply the [ToF patch](tof/README.md) to its
   documented driver revision. For an already-patched driver,
   `git apply --reverse --check PATCH` should succeed;
   skip reapplying it. If neither forward nor reverse checks pass, reconcile the
   driver source first. Build ToF as documented, then rebuild PALS:

   ```sh
   cd ~/catkin_ws
   python -m pip install --user -r src/pidrone_pkg/requirements.txt
   catkin_make --pkg pidrone_pkg -j1
   source devel/setup.bash
   ```

3. From the laptop repository, install the firmware using the existing tool:

   ```sh
   python3 tools/pals.py flash-fc --drone drone1
   ```

   The [firmware guide](firmware/betaflight/README.md) lists supported builds and
   recovery details. The installer verifies the board, preserves drone1's
   configuration and saves `serial_update_rate_hz = 250`. It does not deploy ROS
   code. The source patches are already compiled into the bundled DFU.

4. With the Pi FC node still stopped, check Betaflight's active flight PID
   profile and Angle-mode assignment. Match these settings, then save/reboot:

   ```text
   set angle_limit = 60
   set vbat_sag_compensation = 100
   save
   ```

   Verify both values and `serial_update_rate_hz` after reconnecting. Do not
   replace drone1's receiver/AUX mapping with drone2's mapping.

5. Update the laptop environment with `python3 -m pip install -r requirements.txt`
   and restart the dashboard. Start drone1's nodes from a fresh ROS session so
   old parameter overrides cannot survive. Leave it powered for a few minutes,
   then use **Restart** with props removed, stationary on the floor facing +Y.
   This reboots the warm FC, restarts all nodes, and checks gyro readings while the
   EKF initializes. The gyro mean is reported, not automatically pass/fail gated.

6. Check readiness, IMU continuity, ToF response and camera tracking props-off
   before a user-operated hover. Drone1 has not yet been flight-validated with
   this configuration; shared calibration is an assumption.

The shared baseline is 1540 us hover throttle, 0.30 XY hold gain, 150 ms fusion
delay, 40 ms camera delay and a 100 ms ToF budget. The 60-degree value above is
the Betaflight command scale; the camera geometry remains 45 degrees below
horizontal. Hover duration is a routine argument, not a calibration setting.
