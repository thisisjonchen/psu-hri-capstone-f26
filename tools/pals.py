#!/usr/bin/env python3
"""Launch the PALS dashboard or update a flight controller.

  flash-fc    install the bundled Betaflight USB firmware and 250 Hz serial setting
  dashboard   start the dashboard server and open it in the browser
"""
import argparse
import hashlib
import os
import subprocess
import sys
import threading
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / 'src' / 'web'
VENV = ROOT / '.venv'
VENV_PYTHON = VENV / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
REQUIREMENTS = ROOT / 'requirements.txt'


def ensure_venv():
    """Re-run inside .venv with the root laptop requirements installed."""
    if not VENV_PYTHON.exists():
        print('Creating', VENV)
        subprocess.check_call([sys.executable, '-m', 'venv', str(VENV)])
    stamp = VENV / 'pals-requirements.sha256'
    digest = hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()
    if not stamp.exists() or stamp.read_text() != digest:
        subprocess.check_call([str(VENV_PYTHON), '-m', 'pip', 'install', '-r', str(REQUIREMENTS)])
        stamp.write_text(digest)
    if Path(sys.prefix).resolve() != VENV.resolve():
        sys.exit(subprocess.call([str(VENV_PYTHON), __file__] + sys.argv[1:]))


def dashboard(port):
    threading.Timer(1.5, webbrowser.open, ['http://localhost:%d' % port]).start()
    sys.exit(subprocess.call([sys.executable, str(WEB / 'server.py'), '--port', str(port)]))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('flash-fc', 'dashboard'))
    parser.add_argument('--drone', choices=('drone1', 'drone2'),
                        help='target drone for flash-fc')
    parser.add_argument('--port', type=int, default=8000, help='dashboard port')
    args = parser.parse_args()
    if args.command == 'flash-fc' and not args.drone:
        parser.error('flash-fc requires --drone drone1 or --drone drone2')
    ensure_venv()
    sys.path.insert(0, str(WEB))
    if args.command == 'flash-fc':
        from flash_fc import flash
        try:
            flash(args.drone)
        except (RuntimeError, OSError, EOFError) as error:
            parser.exit(1, 'flash-fc: %s\n' % error)
    else:
        dashboard(args.port)


if __name__ == '__main__':
    main()
