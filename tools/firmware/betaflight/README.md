# SpeedyBee F405 V3 firmware

Drone2's SpeedyBee F405 V3 runs Betaflight 2026.6.1 with revision marker
`usb2ms001`. The [USB patch](usb-flush-2ms.patch) changes
`CDC_IN_FRAME_INTERVAL` from 15 to 1 in the F405 USB driver. Its post-increment
frame counter makes this a 2 ms USB IN flush interval. The saved CLI setting is
`serial_update_rate_hz = 250`; see [telemetry setup](../../../src/README.md#betaflight-telemetry-setup).

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
already-patched `usb2ms001` image. A different board or firmware build is rejected
before writing firmware; prepare a matching build before updating such a drone.
It uses each drone's existing configuration, including its calibration and
receiver settings. No configuration dump from another drone is applied.

Backups are kept in `firmware_backups/<drone>_<timestamp>/` on the laptop and
`~/catkin_ws/firmware_backups/` on the Pi. If the operation fails, the command
prints the backup location and does not restart a stopped FC node after an error. A
controller left in DFU mode before any firmware write can return to its original
firmware by disconnecting and reconnecting its power.

The 100 Hz IMU delivery also needs the current Pi-side
`flight_controller_node.py`, `h2rMultiWii.py`, and generated `MspImu` ROS message.
Sync this branch to the Pi and build its ROS package if those are missing. The
flash command handles firmware and its serial setting; it does not deploy Pi code
or claim a measured IMU rate. Drone2 has already been measured at 99.6 Hz.

## Build

Use these exact sources and compiler:

- [Betaflight](https://github.com/betaflight/betaflight) commit
  `6dbc4218fd6bc33bf16ea32c670304d4f89321d5`.
- [Betaflight board configs](https://github.com/betaflight/config) revision
  `749fff1`, target `SPEEDYBEEF405V3`.
- Arm GNU Toolchain 13.3.Rel1, GCC 13.3.1.

Apply `usb-flush-2ms.patch` from the Betaflight source root with `git apply`.
Then build there, replacing the two `/path/to/` paths below. The feature flags
match drone2's original official build
[`a7fa899c9773c51462447321eaff4ec0`](https://build.betaflight.com/api/builds/a7fa899c9773c51462447321eaff4ec0/json).

```sh
make -j4 hex CONFIG=SPEEDYBEEF405V3 \
  CONFIG_DIR=/path/to/config \
  ARM_SDK_PREFIX=/path/to/toolchain/bin/arm-none-eabi- \
  AUTOHYDRATE_STAMPS= REVISION=usb2ms001 \
  "CONFIG_REVISION_DEFINE=-D'__CONFIG_REVISION__=\"749fff1\"'" \
  'EXTRA_FLAGS=-DCLOUD_BUILD -DUSE_ACRO_TRAINER -DUSE_DSHOT -DUSE_GPS -DUSE_GPS_PLUS_CODES -DUSE_LED_STRIP -DUSE_OPTICALFLOW -DUSE_OSD -DUSE_OSD_HD -DUSE_OSD_SD -DUSE_PINIO -DUSE_RANGEFINDER -DUSE_SERIALRX -DUSE_SERIALRX_CRSF -DUSE_TELEMETRY -DUSE_TELEMETRY_CRSF -DUSE_VTX'
```

The bundled DFU is `betaflight_2026.6.1_SPEEDYBEEF405V3_usb2ms001.dfu`, SHA-256
`b965e3bc8e8e57d5bd7a8425f14ba29fc423081297e81ccc0cbbd016a6493e83`.
`manifest.json` records its checksum and the supported firmware fingerprints.
Drone2's patched HEX has SHA-256
`81c71352c2649182b233e0b71a24372a8c5c9d947c97ede722d0469dceef9a18`.
Rebuilds can differ because Betaflight embeds the build date and time. Betaflight
is distributed under the included [GPL license](LICENSE); its pinned upstream
source and local patch are linked above.

## Installed configuration and backups

The firmware was flashed using DfuSe elements at `0x08000000` and `0x08008000`.
Readback verified all 452,772 firmware bytes and confirmed that the 16 KB
configuration sector at `0x08004000` was unchanged. The saved CLI configuration
was also checked after reboot. PID, receiver, motor, sensor and calibration
settings were preserved.

Drone2's installed HEX, original firmware, full pre-flash device backup, CLI
configuration, and verification results are retained locally under
`flow_capture_ekf_drone2_20261030/firmware_usb_100hz/` at the repository root.
That recording directory is ignored by Git. The DFU, source patch, manifest and
build recipe here are included in the repository. A stock firmware reflash removes
the patch.
