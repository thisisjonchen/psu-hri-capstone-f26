# Localization (UWB)

Each drone's DWM1001-DEV tag works out its own 2D position from four corner anchors (stock PANS firmware). The Pi reads that position over USB with `UWBLocator`. z will come from the ToF sensor (see `UWBLocator.read_z`, still a TODO).

## Files

| File | Purpose |
|---|---|
| `uwb_locator.py` | Runtime module for the Pi. `UWBLocator` opens the tag's serial port, switches PANS into shell mode, and turns on the `lep` position stream. A daemon thread then keeps the latest `Position(x, y, z, quality, timestamp)` and drops fixes below `min_quality`. `position` and `is_fresh()` never block. `read_z()` is the stub where the ToF height will go. Run it directly to print live positions. |
| `configure_board.py` | One-time setup script. Sets a board's network ID and mode (initiator, anchor, or tag). Anchors also get their position (converted to mm); the tag gets its update rate. Prints `si` at the end so you can check the result. |
| `anchors.json` | Network ID, tag update rate (in 100 ms units), and anchor positions in meters. A0 is the initiator. |
| `__init__.py` | Makes the folder importable: `from localization import UWBLocator, Position`. |
| `requirements.txt` | Only dependency is `pyserial` (on 16.04, `apt install python3-serial` works too). |
| `test_uwb_locator.py` | Unit tests that need no hardware. A `FakeSerial` stands in for the board, and the tests cover line parsing, freshness, start/retry/stop, the quality filter, the z hook, serial failure, and the commands `configure_board.py` sends. Takes about 13 s. |

All code targets Python 3.5 (Ubuntu 16.04's stock Python).

## World frame

```
 y
 6 A3 ●───────● A2 (3,6)
   |         |
   |         |
 0 A0 ●───────● A1 (3,0)
   0         3   x      (meters, z up, A0 = origin)
```

Anchor positions live in `anchors.json`. Measure the real positions and update the file before configuring.
All anchors sit at z = 0, so UWB can't resolve height. That's expected, because z comes from ToF.

## Setup (once per board)

1. `pip3 install -r requirements.txt` and add the user to `dialout`: `sudo usermod -aG dialout $USER`
2. Plug a board into USB and run one of these:
   - `python3 configure_board.py --anchor A0` (repeat for A1, A2, A3)
   - `python3 configure_board.py --tag` (one per drone)
3. Check the `si` output it prints. If a command fails, compare against `help` in the board shell, or configure with the Decawave DRTLS Android app instead.

## Usage on the Pi

```python
from localization import UWBLocator

loc = UWBLocator('/dev/ttyACM0')
loc.start()
p = loc.position        # Position(x, y, z, quality, timestamp) or None
if not loc.is_fresh():  # no good fix within stale_after seconds
    ...                 # hold / land
loc.stop()
```

To check a tag quickly: `python3 -m localization.uwb_locator /dev/ttyACM0`

Tests (no hardware needed; run from the repo root): `python3 -m unittest localization.test_uwb_locator`
