# DWM1001-DEV Board Setup

Every board runs the same Qorvo PANS firmware. Anchor vs. tag is a configuration setting, not a different firmware image, so you never flash a "tag build" or an "anchor build".

You need 5 boards for a working fix: 4 anchors (A0–A3) and 1 tag per drone.

## 1. Prerequisites

- `pip3 install -r requirements.txt` (on the Pi: `sudo apt install python3-serial` also works)
- Linux only: `sudo usermod -aG dialout $USER`, then log out and back in
- A micro-USB **data** cable (J9, the USB port next to the J-Link chip)

Find the board's serial port after plugging it in:

| OS | Port |
|---|---|
| Linux / Pi | `/dev/ttyACM0` |
| macOS | `ls /dev/cu.usbmodem*` |

## 2. Check the firmware

```
screen <port> 115200
```

Press **Enter twice within 1 second** to get the `dwm>` prompt, then run `si` and `help`. Quit with `Ctrl-A` then `K`.

If `help` lists `acas`, `acts`, `aps`, `aurs`, `lep`, `nis` and `reset`, skip to step 4. If there's no prompt, the output is garbled, or the PANS version is old, reflash (step 3).

## 3. Flash PANS (only if needed)

1. From the [DWM1001-DEV product page](https://www.qorvo.com/products/ek/DWM1001-DEV), download the DWM1001 software and documentation package and find `DWM1001_PANS_R2.0.hex` (or the latest PANS `.hex` in that package).
2. Install the [SEGGER J-Link Software](https://www.segger.com/downloads/jlink/). The board has an onboard J-Link, so no external programmer is needed.
3. Flash with **J-Flash Lite**: device `nRF52832_xxAA`, interface `SWD`, speed `1000 kHz`, *Erase Chip*, select the `.hex`, *Program Device*.
   Or, with Nordic's command-line tools: `nrfjprog -f nrf52 --chiperase --program DWM1001_PANS_R2.0.hex --reset`
4. Repeat step 2 to confirm the shell works.

Use the same `.hex` for every board.

## 4. Set anchor positions

Mount the anchors, measure their positions in meters with A0 as the origin, and enter them in `anchors.json`. Each anchor stores its position when it's configured, so if you move an anchor later you have to configure it again.

Keep exactly one anchor as `"initiator": true` (A0).

## 5. Configure each board

Close any open `screen` session first, because only one program can hold the port. From `localization/`:

```
python3 configure_board.py --port <port> --anchor A0   # then A1, A2, A3
python3 configure_board.py --port <port> --tag         # once per drone
```

Label each board as you go. The script:

1. sets the network ID (`nis`), which must match on every board
2. writes the full role config with UWB active and LEDs/BLE on: `acas` for anchors (initiator flag from `anchors.json`), `acts` for tags (location engine on, stationary detection off)
3. resets the board and re-enters the shell
4. sets the anchor position in mm (`aps`) or the tag update rate (`aurs`)
5. prints `si`

### Check the `si` output

| Board | Expect in `si` |
|---|---|
| A0 | mode `ani` (anchor initiator), UWB `act`, the position you set |
| A1–A3 | mode `an`, UWB `act`, the position you set |
| Tag | mode `tn`, UWB `act`, location engine `le` |

Every board should show the same PAN ID (`0x1234`). If a command errored, check its name against `help` for your PANS version. As a fallback, you can configure everything from the Decawave DRTLS Manager app over BLE.

## 6. Bring it up

1. Power the anchors from any USB supply (wall adapter or power bank). Power up A0 (the initiator) first. The other anchors join its network.
2. Plug the tag into the Pi (or your laptop) and run, from the repo root:
   ```
   python3 -m localization.uwb_locator <port>
   ```
3. You should see `Position(x=..., y=..., ...)` with `fresh`. Move the tag and the position should follow.

## Troubleshooting

| Symptom | Fix |
|---|---|
| No `dwm>` prompt | Press Enter twice faster. Check that the cable carries data. Reflash (step 3). |
| `Permission denied` on the port | Linux: join `dialout`. Close `screen` or any other program using the port. |
| `No POS data ... is the tag configured?` | Check the tag's `si`: mode `tn`, UWB `act`, `le` on. Check that the anchors are powered and share the PAN ID. |
| Position always `STALE` | Quality is below `min_quality` (50). Need line of sight to at least 3 anchors. Check the anchor positions in `si`. |
| Position is offset or mirrored | Wrong `aps` values. Fix `anchors.json` and re-run `configure_board.py` on the affected anchors. |
