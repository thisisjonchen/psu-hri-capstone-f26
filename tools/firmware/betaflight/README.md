# SpeedyBee F405 V3 firmware

The bundled Betaflight 2026.6.1 build is `imu100001`: a 2 ms USB flush interval
and [integrated IMU telemetry](imu-integration.patch). It accumulates rotation
and specific force between acknowledged replies, with FC timestamps and durations.
Standard MSP readings remain unchanged. The serial setting is 250 Hz.

Drone2 uses this build with the matching ROS package. Drone1 is unverified.

## Apply to a drone

Keep the flight controller connected to its Pi by USB, remove the props, and
keep the drone disarmed. From this repository on the laptop, run:

```sh
python3 tools/pals.py flash-fc --drone drone1
```

Use `--drone drone2` for drone2. SSH access must already work, as it does for the
dashboard. Type the drone's name at the prompt to confirm the target. The command
installs `dfu-util` and `python3-serial` on the Pi host if needed, using its
passwordless sudo access.

The command handles the complete firmware update:

1. Stops the FC node after checking that it reports DISARMED, and checks USB ownership.
2. Checks the board and installed firmware, then backs up that drone's settings
   and full flash before writing.
3. Installs the bundled DFU and verifies its bytes and the preserved configuration sector.
4. Saves `serial_update_rate_hz = 250`, reboots, and verifies the setting and all
   other saved settings. Restarts the FC node if it was running beforehand.

It supports the exact original Betaflight 2026.6.1 build listed below and the
patched `usb2ms001` and `imu100001` images. A different board or firmware build is rejected
before writing firmware; prepare a matching build before updating such a drone.
It uses each drone's existing configuration, including its calibration and
receiver settings. No configuration dump from another drone is applied.

Backups are kept in `firmware_backups/<drone>_<timestamp>/` on the laptop and
`~/catkin_ws/firmware_backups/` on the Pi. If the operation fails, the command
prints the backup location and does not restart a stopped FC node after an error. A
controller left in DFU mode before any firmware write can return to its original
firmware by disconnecting and reconnecting its power.

The 100 Hz IMU delivery also needs the current Pi-side
`flight_controller_node.py`, `h2rMultiWii.py`, the generated `IntegratedImu` message,
and the rebuilt EKF. Update firmware and ROS code together. The flash command
does not deploy Pi code. Restart localization on the floor after restarting the FC.

## Build

Use these exact sources and compiler:

- [Betaflight](https://github.com/betaflight/betaflight) commit
  `6dbc4218fd6bc33bf16ea32c670304d4f89321d5`.
- [Betaflight board configs](https://github.com/betaflight/config) revision
  `749fff1`, target `SPEEDYBEEF405V3`.
- Arm GNU Toolchain 13.3.Rel1, GCC 13.3.1.

Apply `usb-flush-2ms.patch` and `imu-integration.patch` from the Betaflight source
root with `git apply`.
Then build there, replacing the two `/path/to/` paths below. The feature flags
match drone2's original official build
[`a7fa899c9773c51462447321eaff4ec0`](https://build.betaflight.com/api/builds/a7fa899c9773c51462447321eaff4ec0/json).

```sh
make -j4 hex CONFIG=SPEEDYBEEF405V3 \
  CONFIG_DIR=/path/to/config \
  ARM_SDK_PREFIX=/path/to/toolchain/bin/arm-none-eabi- \
  AUTOHYDRATE_STAMPS= REVISION=imu100001 \
  "CONFIG_REVISION_DEFINE=-D'__CONFIG_REVISION__=\"749fff1\"'" \
  'EXTRA_FLAGS=-DCLOUD_BUILD -DUSE_ACRO_TRAINER -DUSE_DSHOT -DUSE_GPS -DUSE_GPS_PLUS_CODES -DUSE_LED_STRIP -DUSE_OPTICALFLOW -DUSE_OSD -DUSE_OSD_HD -DUSE_OSD_SD -DUSE_PINIO -DUSE_RANGEFINDER -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_VTX'
```

The bundled DFU is `betaflight_2026.6.1_SPEEDYBEEF405V3_imu100001.dfu`.
`manifest.json` records its checksum and the supported firmware fingerprints.
Rebuilds can differ because Betaflight embeds the build date and time. Betaflight
is distributed under the included [GPL license](LICENSE); its pinned upstream
source and local patches are linked above. A stock reflash removes both patches.
