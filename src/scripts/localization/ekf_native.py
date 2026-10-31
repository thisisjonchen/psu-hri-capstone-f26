"""Python replay bindings for the native EKF math."""
import ctypes

import numpy as np


try:
    _library = ctypes.CDLL('libpidrone_ekf.so')
except OSError as error:
    raise ImportError('Build pidrone_ekf and source the workspace before using the EKF: '+str(error))

_vector = np.ctypeslib.ndpointer(dtype=np.float64, shape=(3,), flags='C_CONTIGUOUS')
_state = np.ctypeslib.ndpointer(dtype=np.float64, shape=(10,), flags=('C_CONTIGUOUS', 'WRITEABLE'))
_matrix = np.ctypeslib.ndpointer(dtype=np.float64, shape=(15, 15), flags='C_CONTIGUOUS')
_writable_matrix = np.ctypeslib.ndpointer(dtype=np.float64, shape=(15, 15),
                                       flags=('C_CONTIGUOUS', 'WRITEABLE'))
_float_matrix = np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags='C_CONTIGUOUS')

predict = _library.pals_ekf_predict
predict.argtypes = [_state, _writable_matrix, _vector, _vector, _vector, _matrix, ctypes.c_double]
predict.restype = ctypes.c_int

_float_vector = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C_CONTIGUOUS')
_correction_state = np.ctypeslib.ndpointer(dtype=np.float64, shape=(16,),
                                          flags=('C_CONTIGUOUS', 'WRITEABLE'))
_correct = _library.pals_ekf_correct
_correct.argtypes = [_correction_state, _writable_matrix, _float_vector, _float_matrix,
                     _float_matrix, ctypes.c_size_t, ctypes.c_double, _float_matrix, _float_vector]
_correct.restype = ctypes.c_int


def correct(state, covariance, innovation, observation, noise, gate):
    size = len(innovation)
    observation = np.ascontiguousarray(observation, dtype=float)
    if observation.shape != (size, 15):
        raise ValueError('Invalid observation shape')
    total, nis = np.empty((size, size)), np.zeros(1)
    status = _correct(state, covariance, innovation, observation, noise, size, gate, total, nis)
    return status, total, float(nis[0])


_quaternion = np.ctypeslib.ndpointer(dtype=np.float64, shape=(4,), flags='C_CONTIGUOUS')
_rotation = np.ctypeslib.ndpointer(dtype=np.float64, shape=(3, 3), flags='C_CONTIGUOUS')
_indices = np.ctypeslib.ndpointer(dtype=np.int64, ndim=1, flags=('C_CONTIGUOUS', 'WRITEABLE'))
_observations = np.ctypeslib.ndpointer(dtype=np.float64, ndim=3, flags=('C_CONTIGUOUS', 'WRITEABLE'))
_camera_indices = _library.pals_camera_indices
_camera_indices.argtypes = [_vector, _quaternion, _rotation, _vector, _float_matrix,
                            ctypes.c_size_t, ctypes.c_double, ctypes.c_double, ctypes.c_double, _indices]
_camera_indices.restype = ctypes.c_ssize_t
_camera_linearize = _library.pals_camera_linearize
_camera_linearize.argtypes = [_vector, _vector, _quaternion, _vector, _vector,
                             _rotation, _vector, _float_matrix, ctypes.c_size_t,
                             ctypes.c_double, ctypes.c_double, ctypes.c_double,
                             _indices, _float_matrix, _observations]
_camera_linearize.restype = ctypes.c_ssize_t


def _rays(value):
    result = np.ascontiguousarray(value, dtype=float)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError('Invalid ray shape')
    return result


def camera_indices(position, quaternion, rotation, offset, rays, floor, min_height, max_reach):
    rays = _rays(rays)
    indices = np.empty(len(rays), dtype=np.int64)
    count = _camera_indices(np.ascontiguousarray(position, dtype=float),
        np.ascontiguousarray(quaternion, dtype=float), rotation, offset, rays, len(rays),
        floor, min_height, max_reach, indices)
    if count < 0:
        raise ValueError('Zero or unbounded quaternion')
    return indices[:count]


def camera_linearize(position, velocity, quaternion, bias, rate, rotation, offset,
                     rays, floor, min_height, max_reach):
    rays = _rays(rays)
    indices = np.empty(len(rays), dtype=np.int64)
    prediction, observation = np.empty((len(rays), 2)), np.empty((len(rays), 2, 15))
    position, velocity, quaternion, bias, rate = [np.ascontiguousarray(v, dtype=float)
        for v in (position, velocity, quaternion, bias, rate)]
    count = _camera_linearize(position, velocity, quaternion, bias, rate, rotation, offset,
        rays, len(rays), floor, min_height, max_reach, indices, prediction, observation)
    if count < 0:
        raise ValueError('Zero or unbounded quaternion')
    return indices[:count], prediction[:count], observation[:count]
