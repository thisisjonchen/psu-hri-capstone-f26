"""PANS BLE payloads and four-anchor calibration; no ROS or Bluetooth dependency."""
from __future__ import division

import math
import struct

import numpy as np


SERVICE = '680c21d9-c946-4c1f-9c11-baa1c21329e7'
MODE = '3f0afd88-7770-46b0-b5e7-9fc099598964'
PAN = '80f9d8bc-3bff-45bb-a181-2d6a37991208'
DATA_MODE = 'a02b947e-df97-4516-996a-1882521e0ead'
LOCATION = '003bbdf2-c634-4b3d-ab56-7ec889b89a37'
INFO = '1e63b1eb-d4ed-444e-af54-c1e965192501'
POSITION = 'f0f26c9b-2c8c-49ac-ab60-fe03def1b40c'
CLUSTER = '17b1613e-98f2-4436-bcde-23af17a10c72'


def decode_info(info, mode, pan):
    if len(info) != 29 or len(mode) != 2 or len(pan) != 2:
        raise ValueError('Unsupported PANS device information format')
    flags, options = struct.unpack('<BB', mode)
    address = struct.unpack('<Q', info[:8])[0]
    return {'id': '%04X' % (address & 0xffff), 'address': '%016X' % address,
            'anchor': bool(flags & 0x80), 'active': ((flags >> 5) & 3) == 2,
            'initiator': bool(options & 0x80), 'pan': struct.unpack('<H', pan)[0]}


def decode_location(data):
    """API guide 7.1.3: type, optional XYZ/QF, count and <HI B> range records."""
    if not data:
        raise ValueError('PANS returned empty location data')
    kind = struct.unpack('<B', data[:1])[0]
    if kind not in (0, 1, 2):
        raise ValueError('Unsupported PANS location type %s' % kind)
    offset, position, distances = 1, None, {}
    if kind in (0, 2):
        if len(data) < offset + 13:
            raise ValueError('Truncated PANS position')
        x, y, z, quality = struct.unpack_from('<iiiB', data, offset)
        if quality > 100:
            raise ValueError('Invalid PANS position quality')
        position = {'x': x / 1000., 'y': y / 1000., 'z': z / 1000., 'quality': quality}
        offset += 13
    if kind in (1, 2):
        if len(data) <= offset:
            raise ValueError('Missing PANS distance count')
        count = struct.unpack_from('<B', data, offset)[0]
        offset += 1
        if count > 15 or len(data) != offset + 7 * count:
            raise ValueError('Unsupported PANS range payload length; capture BLE bytes for diagnosis')
        for _ in range(count):
            address, millimeters, quality = struct.unpack_from('<HIB', data, offset)
            offset += 7
            key = '%04X' % address
            if key in distances or millimeters == 0 or quality > 100:
                raise ValueError('Invalid or duplicate PANS range')
            distances[key] = (millimeters / 1000., quality)
    if len(data) != offset:
        raise ValueError('Unexpected bytes in PANS location data')
    return position, distances


def encode_position(position):
    coordinates = [float(position[axis]) for axis in ('x', 'y', 'z')]
    if not all(np.isfinite(v) and abs(v) <= 1000 for v in coordinates):
        raise ValueError('Anchor coordinates must be finite and within 1000 m')
    return struct.pack('<iiiB', *([int(round(v * 1000)) for v in coordinates] + [100]))


def solve_layout(anchors, rows, min_quality=50, max_rms=0.10, max_error=0.20):
    """Fit six 3D ranges with measured Z; A0 is XY origin, A1 sets +X, A2 is +Y."""
    if len(anchors) != 4 or len(set(a['id'] for a in anchors)) != 4:
        raise ValueError('Select four distinct anchors')
    z = np.array([float(a['z']) for a in anchors])
    if not np.all(np.isfinite(z)) or np.any(np.abs(z) > 100):
        raise ValueError('Enter measured anchor heights in meters')
    pairs, measured, horizontal = [], [], {}
    for i in range(4):
        for j in range(i + 1, 4):
            values = []
            for source, target in ((i, j), (j, i)):
                sample = rows.get(anchors[source]['id'], {}).get(anchors[target]['id'])
                if sample is not None and sample[1] >= min_quality:
                    values.append(float(sample[0]))
            if not values or not all(np.isfinite(v) and 0 < v < 1000 for v in values):
                raise ValueError('Missing good range between %s and %s' % (anchors[i]['id'], anchors[j]['id']))
            if max(values) - min(values) > max_error:
                raise ValueError('Opposite range measurements disagree; check line of sight')
            distance = sum(values) / len(values)
            squared = distance ** 2 - (z[i] - z[j]) ** 2
            if squared < 0.3 ** 2:
                raise ValueError('Anchor spacing or height is inconsistent (minimum XY spacing 0.3 m)')
            horizontal[i, j] = math.sqrt(squared)
            pairs.append((i, j))
            measured.append(distance)
    baseline = horizontal[0, 1]
    initial = [baseline]
    for j in (2, 3):
        x = (horizontal[0, j] ** 2 + baseline ** 2 - horizontal[1, j] ** 2) / (2 * baseline)
        squared = horizontal[0, j] ** 2 - x ** 2
        if squared <= 0.3 ** 2:
            raise ValueError('Nearly collinear or inconsistent anchors')
        initial.extend([x, math.sqrt(squared)])
    values = np.array(initial)
    measured = np.array(measured)

    def residual(v):
        xy = np.array([[0., 0.], [v[0], 0.], v[1:3], v[3:5]])
        return np.array([math.sqrt(np.sum((xy[i] - xy[j]) ** 2) + (z[i] - z[j]) ** 2)
                         for i, j in pairs]) - measured

    damping = 0.001
    for _ in range(100):
        error = residual(values)
        jacobian = np.column_stack([(residual(values + np.eye(5)[k] * 1e-5) - error) / 1e-5
                                    for k in range(5)])
        step = np.linalg.solve(jacobian.T.dot(jacobian) + np.eye(5) * damping,
                               -jacobian.T.dot(error))
        candidate = values + step
        if np.sum(residual(candidate) ** 2) <= np.sum(error ** 2):
            values = candidate
            damping = max(1e-9, damping / 2)
            if np.linalg.norm(step) < 1e-7:
                break
        else:
            damping *= 10
    error = residual(values)
    rms, worst = float(np.sqrt(np.mean(error ** 2))), float(np.max(np.abs(error)))
    xy = np.array([[0., 0.], [values[0], 0.], values[1:3], values[3:5]])
    if values[0] <= 0 or xy[2, 1] <= 0 or xy[3, 1] <= 0:
        raise ValueError('Invalid orientation; order anchors counterclockwise from the origin')
    for i in range(4):
        a, b, c = xy[i], xy[(i + 1) % 4], xy[(i + 2) % 4]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if cross < 0.1:
            raise ValueError('Anchors must form four well-separated convex corners in the selected order')
    if rms > max_rms or worst > max_error:
        raise ValueError('Range fit is inconsistent (RMS %.3f m, max %.3f m)' % (rms, worst))
    positions = [dict(a, x=float(xy[i, 0]), y=float(xy[i, 1]), z=float(z[i]))
                 for i, a in enumerate(anchors)]
    return {'positions': positions, 'rms': rms, 'max_error': worst}
