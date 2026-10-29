#!/usr/bin/env python3
"""Summarize a dashboard flight log (Logs tab > CSV) to find where hover drift comes from.

Usage (from the repository root, with the laptop requirements installed):
  python3 tools/analyze_log.py drone1-flight-<time>.csv
  python3 tools/analyze_log.py drone1-flight-<time>.csv --observed 0.8 --direction right

--observed is how far you saw the drone slide (m, e.g. from a tape mark), and
--direction which way (left/right/forward/back); both are optional.

Phases: baseline (Pre-Arm, untouched), idle (armed on the ground), and hover (from
the first ToF height 8 cm above the baseline to the end). Reports sample timing,
sensor noise, optical-flow status, what odometry believed, attitude, yaw and the
controller's commands, then lists findings. Read-only.
"""
import argparse
import csv
import math
import sys
from collections import defaultdict

import numpy as np

# Filter assumptions from src/params/fusion_localization.yaml and optical_flow.yaml.
ACCELERATION_STDDEV = 0.3   # m/s^2 per sqrt(Hz)
FLOW_MIN_HEIGHT = 0.10      # m; below this fusion ignores optical flow and coasts on the IMU
TAKEOFF_HEIGHT = 0.25       # m; pid_controller.py /pidrone/altitude/takeoff_height default
FLOW_STATUS = ('rejected', 'zero', 'measured')
LIFTOFF_HEIGHT = 0.08       # m above the baseline ToF reading
integrate = getattr(np, 'trapezoid', None) or np.trapz  # NumPy 2 renamed trapz


def load(path):
    signals, events = defaultdict(lambda: ([], [])), {}
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            t = float(row['time_s'])
            if row['signal'] == 'event':
                events.setdefault(row['value'], t)
            else:
                times, values = signals[row['signal']]
                times.append(t)
                values.append(float(row['value']))
    return {name: (np.array(t), np.array(v)) for name, (t, v) in signals.items()}, events


def window(data, name, start, end):
    t, v = data.get(name, (np.array([]), np.array([])))
    keep = (t >= start) & (t < end)
    return t[keep], v[keep]


def noise(t, v, seconds=0.5):
    """Standard deviation around a centred moving average, so slow motion is not counted as noise."""
    if len(v) < 5:
        return float('nan')
    width = max(3, int(round(seconds / np.median(np.diff(t)))) | 1)
    if len(v) <= width:
        return float(np.std(v))
    trend = np.convolve(v, np.ones(width) / width, mode='valid')
    return float(np.std(v[width // 2:width // 2 + len(trend)] - trend))


def phases(data, events):
    end = events.get('stop', max(t[-1] for t, _ in data.values()))
    arm = events.get('arm', end)
    takeoff = events.get('takeoff', end)
    t, height = window(data, 'height.raw.height', 0, arm)
    ground = float(np.median(height)) if len(height) else None
    t, height = window(data, 'height.raw.height', takeoff, end)
    airborne = t[height > ground + LIFTOFF_HEIGHT] if ground is not None else []
    liftoff = float(airborne[0]) if len(airborne) else None
    return {'baseline': (0.0, arm), 'idle': (arm, liftoff if liftoff is not None else takeoff),
            'hover': (liftoff, end) if liftoff is not None else None}, ground


def fmt(value, digits=3):
    return '-' if value is None or not np.isfinite(value) else ('%.' + str(digits) + 'f') % value


def timing_report(data):
    print('\nSample timing (whole log)')
    print('  %-10s %8s %8s %8s %6s' % ('sensor', 'median', 'p95', 'max', 'gaps'))
    found = []
    for name in ('imu', 'flight_raw', 'range', 'flow', 'uwb'):
        _, interval = data.get('interval.' + name, (None, np.array([])))
        if not len(interval):
            continue
        median = np.median(interval)
        gaps = int(np.sum(interval > 2.5 * median))
        print('  %-10s %6.1f ms %6.1f ms %6.0f ms %6d' % (
            name, median, np.percentile(interval, 95), interval.max(), gaps))
        if name == 'imu' and (np.percentile(interval, 95) > 25 or interval.max() > 100):
            found.append('IMU timing is irregular (p95 %.0f ms, max %.0f ms). Gaps make the IMU prediction '
                         'and the differenced angular rate unreliable.' % (np.percentile(interval, 95), interval.max()))
    return found


def noise_report(data, spans):
    rows = [('accel.raw.%s', 'Raw accel %s (m/s^2)', 'xyz'), ('accel.processed.%s', 'Accel %s (m/s^2)', 'xyz'),
            ('rate.raw.%s', 'Raw gyro %s (deg/s)', 'xyz'), ('height.raw.height', 'ToF range (m)', None),
            ('velocity.raw.%s', 'Flow velocity %s (m/s)', 'xy')]
    names = [p for p in ('baseline', 'idle', 'hover') if spans[p]]
    print('\nNoise (std around a 0.5 s moving average)')
    print('  %-24s' % 'signal' + ''.join('%12s' % p for p in names))
    results = {}
    for pattern, label, axes in rows:
        for axis in (axes or [None]):
            name = pattern % axis if axis else pattern
            values = [noise(*window(data, name, *spans[p])) for p in names]
            if all(not np.isfinite(v) for v in values):
                continue
            results[name] = dict(zip(names, values))
            print('  %-24s' % (label % axis if axis else label) + ''.join('%12s' % fmt(v) for v in values))
    return results


def vibration_report(data, span):
    t, z = window(data, 'accel.raw.z', *span)
    if len(z) < 64:
        return
    dt = float(np.median(np.diff(t)))
    uniform = np.interp(np.arange(t[0], t[-1], dt), t, z)
    spectrum = np.abs(np.fft.rfft(uniform - uniform.mean())) / len(uniform)
    freqs = np.fft.rfftfreq(len(uniform), dt)
    peak = int(np.argmax(spectrum[1:])) + 1
    print('\nVibration: strongest raw z-acceleration component in hover at %.1f Hz (sampled at %.0f Hz;'
          ' faster motor vibration folds down below %.0f Hz)' % (freqs[peak], 1 / dt, 0.5 / dt))


def flow_report(data, span):
    t, status = window(data, 'flowstatus.status', *span)
    if not len(status):
        print('\nOptical flow: no status samples in hover (log recorded before flow status was added?)')
        return None
    counts = [float(np.mean(status == i)) for i in range(3)]
    zero = status == 1
    longest, start = 0.0, None
    for time, is_zero in zip(t, zero):
        start = (start if start is not None else time) if is_zero else None
        if start is not None:
            longest = max(longest, time - start)
    print('\nOptical flow in hover: %s' % ', '.join(
        '%s %.0f%%' % (name, 100 * share) for name, share in zip(FLOW_STATUS, counts)))
    print('  longest run of "zero" frames: %.1f s' % longest)
    return counts[1], counts[0], longest


def slide_report(data, span):
    """What odometry believed: body-frame velocity integrated over hover, plus odometry displacement."""
    t, vx = window(data, 'velocity.processed.x', *span)
    _, vy = window(data, 'velocity.processed.y', *span)
    print('\nWhat the estimator believed during hover (%.1f s)' % (span[1] - span[0]))
    if len(t) < 2:
        print('  no fused velocity samples')
        return None
    right, forward = integrate(vx, t), integrate(vy, t)
    print('  integrated fused velocity: %s m right, %s m forward' % (fmt(right, 2), fmt(forward, 2)))
    ot, ox = window(data, 'odometry.x', *span)
    _, oy = window(data, 'odometry.y', *span)
    if len(ot) > 1:
        print('  odometry moved %.2f m' % math.hypot(ox[-1] - ox[0], oy[-1] - oy[0]))
    return right, forward


def attitude_report(data, spans):
    print('\nAttitude and commands (means)')
    found = []
    base, hover = spans['baseline'], spans['hover']
    for axis in ('roll', 'pitch'):
        rest = np.mean(window(data, 'attitude.' + axis, *base)[1]) if base else float('nan')
        air = np.mean(window(data, 'attitude.' + axis, *hover)[1])
        _, command = window(data, 'commands.' + axis, *hover)
        print('  %-5s at rest %s deg, hover %s deg, command offset %s (1500 = level)' % (
            axis, fmt(rest, 2), fmt(air, 2), fmt(np.mean(command) - 1500, 1) if len(command) else '-'))
        if len(command) > 10:
            third = len(command) // 3
            early, late = np.mean(command[:third]) - 1500, np.mean(command[-third:]) - 1500
            print('        command offset first third %s, last third %s' % (fmt(early, 1), fmt(late, 1)))
    t, yaw = window(data, 'attitude.yaw', *hover)
    if len(yaw) > 1:
        change = math.degrees(np.unwrap(np.radians(yaw))[-1] - np.unwrap(np.radians(yaw))[0])
        print('  yaw changed %.1f deg during hover (%.2f deg/s)' % (change, change / max(t[-1] - t[0], 1e-6)))
        if abs(change) > 5:
            found.append('Yaw turned %.0f deg in hover. Heading error rotates velocity into the wrong direction; '
                         'check that tag 5 is visible or that yaw drift is low.' % change)
    for axis in ('x', 'y'):
        _, a = window(data, 'accel.processed.' + axis, *base) if base else (None, np.array([]))
        if len(a) and abs(np.mean(a)) > 0.15:
            found.append('At rest, processed %s acceleration averages %.2f m/s^2 (about %.1f deg of IMU level error). '
                         'The filter treats this as motion until its bias state learns it.'
                         % (axis, np.mean(a), math.degrees(math.atan(abs(np.mean(a)) / 9.8))))
    return found


def height_report(data, events):
    """Did the drone climb to its target, and high enough for fusion to use optical flow?"""
    if 'takeoff' not in events:
        return []
    t, height = window(data, 'height.processed.height', events['takeoff'], events.get('stop', float('inf')))
    if not len(height):
        return []
    below = float(np.mean(height < FLOW_MIN_HEIGHT))
    _, throttle = window(data, 'throttle.throttle', events['takeoff'], events.get('stop', float('inf')))
    print('\nAfter takeoff: median height %.2f m (target %.2f m), max %.2f m; below the %.2f m optical-flow '
          'minimum %.0f%% of the time; final throttle %s' % (np.median(height), TAKEOFF_HEIGHT, height.max(),
                                                           FLOW_MIN_HEIGHT, 100 * below,
                                                           fmt(np.mean(throttle[-20:]), 0) if len(throttle) else '-'))
    found = []
    if height.max() < TAKEOFF_HEIGHT - 0.05:
        found.append('The drone never reached its %.2f m takeoff height (max %.2f m): the altitude loop ran out of '
                     'throttle (hover_throttle plus the rate_i_limit integral). Flying a few cm above the floor puts '
                     'it in ground effect, which pushes it sideways.' % (TAKEOFF_HEIGHT, height.max()))
    if below > 0.2:
        found.append('Height was under flow_min_height for %.0f%% of the flight, so fusion ignored optical flow and '
                     'coasted on the IMU: the estimator could not see the drift.' % (100 * below))
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv', help='CSV downloaded from the dashboard Logs tab')
    parser.add_argument('--observed', type=float, help='slide you saw during hover (m)')
    parser.add_argument('--direction', choices=('left', 'right', 'forward', 'back'), help='direction of that slide')
    args = parser.parse_args()
    data, events = load(args.csv)
    if not data:
        sys.exit('No samples in %s' % args.csv)
    spans, ground = phases(data, events)
    print('%s: events %s' % (args.csv, ', '.join('%s %.1f s' % item for item in sorted(events.items(), key=lambda e: e[1]))
                             or 'none'))
    for name in ('baseline', 'idle', 'hover'):
        span = spans[name]
        print('  %-8s %s' % (name, '%.1f-%.1f s' % span if span else 'not found (no liftoff in the ToF height)'))
    findings = timing_report(data)
    findings += height_report(data, events)
    noise_by_signal = noise_report(data, spans)
    hover = spans['hover']
    if not hover or hover[1] - hover[0] < 2:
        findings.append('No hover of at least 2 s was found, so drift cannot be analyzed.')
    else:
        vibration_report(data, hover)
        flow = flow_report(data, hover)
        slide = slide_report(data, hover)
        findings += attitude_report(data, spans)
        dt = np.median(np.diff(window(data, 'accel.processed.x', *hover)[0])) if len(
            window(data, 'accel.processed.x', *hover)[0]) > 2 else None
        for axis in ('x', 'y'):
            sigma = noise_by_signal.get('accel.processed.' + axis, {}).get('hover')
            if dt and sigma and np.isfinite(sigma) and sigma * math.sqrt(dt) > 2 * ACCELERATION_STDDEV:
                findings.append('Hover %s acceleration noise is about %.2f m/s^2 per sqrt(Hz), more than twice the '
                                'filter\'s acceleration_stddev (%.1f): it trusts the IMU too much.'
                                % (axis, sigma * math.sqrt(dt), ACCELERATION_STDDEV))
        if flow and flow[0] > 0.25:
            findings.append('Optical flow read "zero" in %.0f%% of hover frames (longest %.1f s). Motion slower than '
                            'one pixel per frame (about 6 cm/s at 0.3 m) is reported as exactly 0 m/s, so a slow '
                            'steady slide is invisible to the filter and the velocity integral never cancels it.'
                            % (100 * flow[0], flow[2]))
        if flow and flow[1] > 0.2:
            findings.append('Optical flow rejected %.0f%% of hover frames (too few blocks see nearby floor); '
                            'the filter coasts on the IMU then.' % (100 * flow[1]))
        # Below ~15 cm the tape measurement and the integration error are the same size.
        if slide is not None and args.observed and args.observed >= 0.15:
            believed = math.hypot(*slide)
            if believed < 0.5 * args.observed:
                findings.append('You saw %.2f m of slide, but the estimator believed only %.2f m: it is missing '
                                'the drift (flow dead zone or flow scale), so the controller has nothing to correct.'
                                % (args.observed, believed))
            else:
                findings.append('The estimator saw most of the slide (%.2f of %.2f m), yet the drone kept going: '
                                'look at the controller (gain_i, i_limit_deg, max_lean_deg) and the command offsets above.'
                                % (believed, args.observed))
            sideways = {'left': -1, 'right': 1}.get(args.direction)
            if sideways and abs(slide[0]) > 0.05 and np.sign(slide[0]) != sideways:
                findings.append('The estimator placed the slide to the %s, opposite to what you saw: check flow '
                                'signs (flip_vectors_vertically) and yaw.' % ('right' if slide[0] > 0 else 'left'))
    print('\nFindings')
    for finding in findings or ['Nothing stood out. Add --observed with the slide you measured.']:
        print('  - ' + finding)


if __name__ == '__main__':
    main()
