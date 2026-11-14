"""Configuration shared by the flight controllers and routines."""
import os

import yaml


def _load_settings(filename):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'params', filename)
    with open(path) as stream:
        return yaml.safe_load(stream)


def _validate_positive(settings):
    for name, value in settings.items():
        if not 0 < value < float('inf'):
            raise ValueError('Setting {} must be finite and positive'.format(name))


def load_flight_settings():
    settings = _load_settings('flight.yaml')
    _validate_positive({name: value for name, value in settings.items()
                        if name not in ('altitude', 'velocity', 'rc')})
    if not 8.0 <= settings['low_flight_v'] <= settings['min_start_v'] <= 20.0:
        raise ValueError('Battery thresholds must fit the accepted voltage range')
    if settings['max_floor_height_m'] + settings['land_disarm_clearance_m'] > settings['max_disarm_height_m']:
        raise ValueError('Floor height and landing clearance exceed the maximum disarm height')
    return settings


def load_coordination_settings():
    settings = _load_settings('coordination.yaml')
    _validate_positive({name: settings[name] for name in ('state_timeout_s', 'selection_timeout_s')})
    return settings


def load_position_settings():
    import rospy

    settings = _load_settings('position.yaml')
    settings['hold_gain'] = float(rospy.get_param('/pidrone/localization/target_gain', settings['hold_gain']))
    settings['hold_speed_limit_m_s'] = float(rospy.get_param(
        '/pidrone/localization/target_speed_limit', settings['hold_speed_limit_m_s']))
    _validate_positive(settings)
    return settings
