#!/usr/bin/env python3
"""Record and analyze optical-flow / IMU / fusion data while moving a drone by hand.

  record   stream the topics from the drone to CSV files on this laptop
  camera   record only the camera: raw motion vectors, flow output, height, IMU
  ekf      capture raw MSP counts, JPEGs, range and published estimates
  analyze  summarize a recording: rates, height, velocities, distances, sign check,
           and (for camera recordings) where in the image motion is seen

Usage (from the repository root, props off):
  python3 tools/flow_capture.py record --drone drone1 --seconds 30
  python3 tools/flow_capture.py camera --drone drone1 --seconds 30
  python3 tools/flow_capture.py analyze flow_capture_<time>
  python3 tools/flow_capture.py ekf --drone drone1 --seconds 30

Uses the same `ssh <drone>` login as the rest of the tools. Subscribers run in
the drone's pidrone_pkg container; recording files are written on this laptop.
"""
import argparse
import base64
import csv
import json
import math
import re
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

TOPICS = {
    'flow': '/pidrone/picamera/twist',
    'state': '/pidrone/state',
    'imu': '/pidrone/imu',
    'range': '/pidrone/range',
    'status': '/pidrone/localization/fusion_status',
    'mode': '/pidrone/mode',
    'commands': '/pidrone/fly_commands',
    'battery': '/pidrone/battery',
}
# Raw camera motion vectors: one 16x16-pixel macroblock per value, plus match cost (sad).
CAMERA_TOPICS = {
    'vectors': '/raspicam_node/motion_vectors',
    'flow': TOPICS['flow'],
    'range': TOPICS['range'],
    'imu': TOPICS['imu'],
}


def record_ekf(drone, seconds, out_dir):
    """Buffer on the Pi so an SSH stall cannot block camera recording callbacks."""
    if not 0 < seconds <= 3600:
        raise ValueError('seconds must be in (0, 3600]')
    out_dir.mkdir(parents=True)
    worker = Path(__file__).with_name('ekf_stream.py').read_bytes()
    command = ("pals_capture=$(mktemp /tmp/pals-ekf.XXXXXX) || exit 1; "
               "trap 'rm -f \"$pals_capture\"' EXIT; "
               "docker exec -i pidrone_pkg bash -ic 'exec python -u - %d' > \"$pals_capture\"; "
               "pals_status=$?; cat \"$pals_capture\" || exit $?; exit \"$pals_status\"") % seconds
    print('Recording on %s for %d seconds; download follows. Remain stationary initially.' % (drone, seconds))
    with (out_dir / 'events.jsonl').open('wb') as output, (out_dir / 'capture.stderr').open('wb') as errors:
        result = subprocess.run(['ssh', drone, command], input=worker, stdout=output,
                                stderr=errors, timeout=seconds + 120)
    if result.returncode:
        raise RuntimeError('Capture failed; see %s' % (out_dir / 'capture.stderr'))
    print('Saved capture to %s' % out_dir)


def read_ekf_events(folder, metadata=None):
    """Read messages in arrival order with their original timestamps.

    Reading through the end also checks that the recording completed.
    """
    session = None
    schemas, counts = {}, Counter()
    sequence, last_elapsed, finished = 0, -1, False
    with (folder / 'events.jsonl').open() as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except ValueError:
                if session is None:  # An interactive login shell can print a preamble.
                    continue
                raise ValueError('Invalid JSON at line %d' % line_number)
            if not isinstance(record, dict):
                raise ValueError('Expected a record at line %d' % line_number)
            kind = record.get('kind')
            if session is None:
                if kind != 'session' or record.get('version') != 1:
                    raise ValueError('Unsupported or missing capture session')
                session = record
                if metadata is not None:
                    metadata.update(record.get('metadata', {}))
                continue
            if finished:
                raise ValueError('Unexpected data after capture completion')
            if kind == 'schema':
                schemas[(record['type'], record['md5sum'])] = record['definition']
            elif kind == 'message':
                if record['sequence'] != sequence:
                    raise ValueError('Missing or reordered event at sequence %d' % sequence)
                for field in ('received_ns', 'elapsed_ns'):
                    if type(record[field]) is not int or record[field] < 0:
                        raise ValueError('Invalid integer timestamp: ' + field)
                stamp = record['header_stamp_ns']
                if stamp is not None and (type(stamp) is not int or stamp < 0):
                    raise ValueError('Invalid header timestamp')
                if record['elapsed_ns'] < last_elapsed:
                    raise ValueError('Recorder monotonic time moved backwards')
                last_elapsed = record['elapsed_ns']
                schema = (record['type'], record['md5sum'])
                if schema not in schemas:
                    raise ValueError('Message has no recorded schema')
                record['definition'] = schemas[schema]
                record['payload'] = base64.b64decode(record['payload'], validate=True)
                counts[record['topic']] += 1
                sequence += 1
                yield record
            elif kind == 'end':
                if record['reason'] != 'duration':
                    raise ValueError('Capture ended early: ' + record['reason'])
                if record['messages'] != sequence or any(
                        record['counts'].get(topic, 0) != counts[topic]
                        for topic in set(record['counts']) | set(counts)):
                    raise ValueError('Capture completion counts do not match')
                finished = True
            else:
                raise ValueError('Unknown capture record: %s' % kind)
    if not finished:
        raise ValueError('Incomplete capture: missing completion marker')


def record(drone, seconds, out_dir, topics=TOPICS):
    out_dir.mkdir(parents=True)
    processes = []
    for name, topic in topics.items():
        command = "docker exec pidrone_pkg bash -ic 'timeout %d rostopic echo -p %s'" % (seconds + 5, topic)
        stream = open(out_dir / (name + '.csv'), 'w')
        processes.append((name, stream, subprocess.Popen(
            ['ssh', drone, command], stdout=stream, stderr=subprocess.DEVNULL)))
    print('Connecting to %s; start moving when told.' % drone)
    time.sleep(4)
    for count in (3, 2, 1):
        print('  %d...' % count)
        time.sleep(1)
    print('RECORDING for %d s: move the drone now. Note each motion and its distance.' % seconds)
    start = time.time()
    while time.time() - start < seconds:
        time.sleep(1)
        left = seconds - int(time.time() - start)
        if left % 5 == 0 and left:
            print('  %d s left' % left)
    print('Stopping...')
    for name, stream, process in processes:
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
        stream.close()
        rows = max(0, sum(1 for _ in open(out_dir / (name + '.csv'))) - 1)
        print('  %-7s %6d messages  (%s)' % (name, rows, topics[name]))
    print('Saved to %s\nAnalyze with: python3 tools/flow_capture.py analyze %s' % (out_dir, out_dir))


def load(path):
    """Rows of a `rostopic echo -p` file as dicts, skipping any shell noise before the header."""
    lines = path.read_text(errors='replace').splitlines() if path.exists() else []
    start = next((i for i, line in enumerate(lines) if line.startswith('%time')), None)
    if start is None:
        return []
    return list(csv.DictReader(lines[start:]))


def number(row, *names):
    for name in names:
        try:
            return float(row[name])
        except (KeyError, TypeError, ValueError):
            continue
    return float('nan')


def series(rows, t0, *fields):
    return [(float(r['%time']) / 1e9 - t0,) + tuple(number(r, f) for f in fields) for r in rows]


def rate_report(name, samples):
    if len(samples) < 2:
        return '  %-6s %5d msgs  NO DATA' % (name, len(samples))
    times = [s[0] for s in samples]
    gaps = [b - a for a, b in zip(times, times[1:])]
    return '  %-6s %5d msgs  %5.1f Hz  largest gap %.2f s' % (
        name, len(samples), (len(times) - 1) / (times[-1] - times[0]), max(gaps))


def nearest(samples, t):
    best = min(samples, key=lambda s: abs(s[0] - t)) if samples else None
    return best if best and abs(best[0] - t) < 0.2 else None


def smooth(values, width=5):
    half = width // 2
    return [sum(values[max(0, i - half):i + half + 1]) / len(values[max(0, i - half):i + half + 1])
            for i in range(len(values))]


def correlation(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if not (math.isnan(x) or math.isnan(y))]
    if len(pairs) < 10:
        return float('nan')
    ma, mb = sum(p[0] for p in pairs) / len(pairs), sum(p[1] for p in pairs) / len(pairs)
    sa = math.sqrt(sum((p[0] - ma) ** 2 for p in pairs))
    sb = math.sqrt(sum((p[1] - mb) ** 2 for p in pairs))
    return sum((p[0] - ma) * (p[1] - mb) for p in pairs) / (sa * sb) if sa and sb else float('nan')


def sign_check(flow, imu, axis):
    """Correlate flow-derived acceleration with IMU acceleration on one body axis."""
    if len(flow) < 20 or len(imu) < 20:
        return 'not enough data'
    times = [s[0] for s in flow]
    velocity = smooth([s[axis] for s in flow])
    flow_accel = [(velocity[i + 1] - velocity[i - 1]) / (times[i + 1] - times[i - 1])
                  for i in range(1, len(times) - 1)]
    imu_accel = []
    for t in times[1:-1]:
        sample = nearest(imu, t)
        imu_accel.append(sample[axis] if sample else float('nan'))
    r = correlation(smooth(flow_accel), smooth(imu_accel))
    verdict = ('AGREE' if r > 0.3 else 'OPPOSITE SIGN?' if r < -0.3 else 'unclear (move more, or flow is weak)')
    return 'r = %+.2f  -> %s' % (r, verdict)


def camera_report(folder, t0, imu, flow):
    """Where in the image the camera sees motion, to test texture and 60 degree tilt effects."""
    rows = load(folder / 'vectors.csv')
    if not rows:
        return
    columns = list(rows[0])
    xs = sorted((c for c in columns if re.match(r'field\.x\d+$', c)), key=lambda c: int(c[7:]))
    ys = sorted((c for c in columns if re.match(r'field\.y\d+$', c)), key=lambda c: int(c[7:]))
    sads = sorted((c for c in columns if re.match(r'field\.sad\d+$', c)), key=lambda c: int(c[9:]))
    count = len(xs)
    width = int(number(rows[0], 'field.mbx')) if not math.isnan(number(rows[0], 'field.mbx')) else 0
    if not width or count % width:
        width = next((w for w in (16, 21, 15, 20) if count % w == 0), count)
    height = count // width
    frames = []
    for row in rows:
        x = [number(row, c) for c in xs]
        y = [number(row, c) for c in ys]
        sad = [number(row, c) for c in sads]
        if any(math.isnan(v) for v in x + y):
            continue
        frames.append((float(row['%time']) / 1e9 - t0, x, y, sad))
    print('\n== Raw camera motion vectors: %d frames, %d x %d macroblock grid (columns x rows)' % (
        len(frames), width, height))
    print(rate_report('vectors', [(f[0],) for f in frames]))
    if not frames:
        return

    def moving(frame):
        accel = nearest(imu, frame[0])
        return accel is not None and math.hypot(accel[1], accel[2]) > 0.4

    print('\n   time | blocks moving | mean x  mean y (px/frame) | mean |v| | mean sad | flow node x  y')
    end = frames[-1][0]
    t = 0.0
    while t < end:
        window = [f for f in frames if t <= f[0] < t + .5]
        if window:
            nonzero = sum(sum(1 for a, b in zip(f[1], f[2]) if a or b) for f in window) / (count * len(window))
            mx = sum(sum(f[1]) / count for f in window) / len(window)
            my = sum(sum(f[2]) / count for f in window) / len(window)
            mag = sum(sum(math.hypot(a, b) for a, b in zip(f[1], f[2])) / count for f in window) / len(window)
            sad = sum(sum(f[3]) / max(1, len(f[3])) for f in window) / len(window)
            fl = [s for s in flow if t <= s[0] < t + .5]
            fx = sum(s[1] for s in fl) / len(fl) if fl else float('nan')
            fy = sum(s[2] for s in fl) / len(fl) if fl else float('nan')
            print('  %5.1f |     %5.1f%%    | %+6.2f  %+6.2f            |  %5.2f   | %7.0f  | %+6.3f %+6.3f' % (
                t, 100 * nonzero, mx, my, mag, sad, fx, fy))
        t += .5

    active = [f for f in frames if moving(f)] or frames
    print('\n== Image rows while moving (%d frames with IMU motion); row 0 = top of the image' % len(active))
    print('   row | mean |x|  mean |y| | spread from centre (x * side)')
    for r in range(height):
        cells = [(f[1][r * width + c], f[2][r * width + c], c) for f in active for c in range(width)]
        ax = sum(abs(a) for a, _, _ in cells) / len(cells)
        ay = sum(abs(b) for _, b, _ in cells) / len(cells)
        spread = sum(a * (1 if c >= width / 2.0 else -1) for a, _, c in cells) / len(cells)
        print('   %3d |  %6.2f    %6.2f   |  %+6.2f' % (r, ax, ay, spread))
    print('  A straight-down camera moves every row equally. A 60 deg forward-tilted camera shows')
    print('  smaller motion in rows that see distant floor, and forward motion spreads outward.')


def command_report(data, t0):
    """Roll/pitch/throttle sent to Betaflight while FLYING (commands are only sent then)."""
    commands = series(data.get('commands', []), t0, 'field.roll', 'field.pitch', 'field.throttle')
    if not commands:
        return
    print('\n== Commands to Betaflight while flying (us; 1500 = level)')
    for i, name in ((1, 'roll'), (2, 'pitch'), (3, 'throttle')):
        values = sorted(c[i] for c in commands if not math.isnan(c[i]))
        print('  %-8s mean %7.1f  min %7.1f  max %7.1f  median %7.1f' % (
            name, sum(values) / len(values), values[0], values[-1], values[len(values) // 2]))
    for i, name in ((1, 'roll'), (2, 'pitch')):
        extreme = max(abs(c[i] - 1500) for c in commands)
        pinned = sum(1 for c in commands if abs(c[i] - 1500) >= extreme - 0.5) / len(commands)
        print('  %-8s at its largest lean (%.1f us) %.0f%% of the time' % (name, extreme, 100 * pinned))
    print('  The steady throttle in a level hover approximates hover_throttle (params/altitude.yaml).')
    battery = [b for b in series(data.get('battery', []), t0, 'field.vbat') if not math.isnan(b[1])]
    if battery:
        start, end = commands[0][0], commands[-1][0]
        flying = [b[1] for b in battery if start <= b[0] <= end] or [b[1] for b in battery]
        print('  battery  %.2f V at start of flight, %.2f V lowest, %.2f V at end' % (
            flying[0], min(flying), flying[-1]))


def analyze(folder):
    names = TOPICS if (folder / 'state.csv').exists() else CAMERA_TOPICS
    data = {name: load(folder / (name + '.csv')) for name in TOPICS}
    data.update({name: load(folder / (name + '.csv')) for name in names})
    starts = [float(rows[0]['%time']) / 1e9 for rows in data.values() if rows]
    if not starts:
        sys.exit('No data in %s' % folder)
    t0 = min(starts)
    flow = series(data['flow'], t0, 'field.twist.linear.x', 'field.twist.linear.y')
    state = series(data['state'], t0, 'field.twist_with_covariance.twist.linear.x',
                   'field.twist_with_covariance.twist.linear.y', 'field.pose_with_covariance.pose.position.z')
    imu = series(data['imu'], t0, 'field.linear_acceleration.x', 'field.linear_acceleration.y',
                 'field.angular_velocity.x', 'field.angular_velocity.y', 'field.angular_velocity.z')
    height = series(data['range'], t0, 'field.range')

    print('== Topic rates (expected: flow ~30 Hz, imu ~50-60 Hz, range ~10 Hz, state ~60 Hz)')
    for name, samples in (('flow', flow), ('imu', imu), ('range', height), ('state', state)):
        print(rate_report(name, samples))
    modes = Counter(r.get('field.mode', '?') for r in data['mode'])
    print('  modes seen: %s' % (dict(modes) or 'none'))

    ranges = [s[1] for s in height if not math.isnan(s[1])]
    if ranges:
        print('\n== ToF height: min %.3f  median %.3f  max %.3f m  (flow is used above 0.10 m)' % (
            min(ranges), sorted(ranges)[len(ranges) // 2], max(ranges)))

    print('\n== Velocity over time, 0.5 s bins (body frame: x = right, y = forward, m/s)')
    print('   time  height | flow x  flow y | fused x fused y | gyro z')
    end = max(s[0] for s in flow + state + imu + height)
    t = 0.0
    while t < end:
        def mean(samples, i):
            values = [s[i] for s in samples if t <= s[0] < t + .5 and not math.isnan(s[i])]
            return sum(values) / len(values) if values else float('nan')
        print('  %5.1f  %6.3f | %+6.2f  %+6.2f | %+6.2f  %+6.2f | %+5.2f' % (
            t, mean(height, 1), mean(flow, 1), mean(flow, 2), mean(state, 1), mean(state, 2), mean(imu, 5)))
        t += .5

    def distance(samples, i):
        total = 0.0
        for a, b in zip(samples, samples[1:]):
            if not (math.isnan(a[i]) or b[0] - a[0] > .2):
                total += a[i] * (b[0] - a[0])
        return total
    print('\n== Net distance (velocity integrated over the recording)')
    print('  flow : right %+.2f m, forward %+.2f m' % (distance(flow, 1), distance(flow, 2)))
    print('  fused: right %+.2f m, forward %+.2f m' % (distance(state, 1), distance(state, 2)))
    print('  Compare with the distances you moved (tape measure). Flow values scale with height.')

    print('\n== Sign check: flow acceleration vs IMU acceleration (same body axis)')
    print('  x (right)  : %s' % sign_check(flow, imu, 1))
    print('  y (forward): %s' % sign_check(flow, imu, 2))
    camera_report(folder, t0, imu, flow)
    command_report(data, t0)

    # Some diagnostic values contain commas ([x, y]), so read key,value pairs from the raw text.
    results, sources = Counter(), Counter()
    path = folder / 'status.csv'
    for line in (path.read_text(errors='replace').splitlines() if path.exists() else []):
        for key, counter in (('last_velocity_result', results), ('velocity_source', sources)):
            match = re.search(r',%s,([^,]*)' % key, line)
            if match:
                counter[match.group(1)] += 1
    if results:
        print('\n== Fusion results (how often each message appeared)')
        for name, counter in (('velocity source', sources), ('last velocity result', results)):
            for value, count in counter.most_common():
                print('  %-22s %5d  %s' % (name, count, value))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    rec = sub.add_parser('record', help='record while moving the drone by hand')
    rec.add_argument('--drone', required=True, choices=('drone1', 'drone2'))
    rec.add_argument('--seconds', type=int, default=30)
    rec.add_argument('--out', type=Path, help='output folder (default: ./flow_capture_<time>)')
    cam = sub.add_parser('camera', help='record only the camera topics while moving the drone')
    cam.add_argument('--drone', required=True, choices=('drone1', 'drone2'))
    cam.add_argument('--seconds', type=int, default=30)
    cam.add_argument('--out', type=Path, help='output folder (default: ./flow_capture_camera_<time>)')
    ana = sub.add_parser('analyze', help='summarize a recording')
    ana.add_argument('folder', type=Path)
    ekf = sub.add_parser('ekf', help='capture IMU, images and other EKF inputs in callback order')
    ekf.add_argument('--drone', required=True, choices=('drone1', 'drone2'))
    ekf.add_argument('--seconds', type=int, default=30)
    ekf.add_argument('--out', type=Path, help='output folder (default: ./flow_capture_ekf_<time>)')
    args = parser.parse_args()
    if args.command == 'record':
        record(args.drone, args.seconds, args.out or Path('flow_capture_' + time.strftime('%Y%m%d_%H%M%S')))
    elif args.command == 'camera':
        record(args.drone, args.seconds,
               args.out or Path('flow_capture_camera_' + time.strftime('%Y%m%d_%H%M%S')), CAMERA_TOPICS)
    elif args.command == 'ekf':
        record_ekf(args.drone, args.seconds,
                   args.out or Path('flow_capture_ekf_' + time.strftime('%Y%m%d_%H%M%S')))
    else:
        analyze(args.folder)


if __name__ == '__main__':
    main()
