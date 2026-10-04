try:
    import ujson as _json
    _JSON_IS_UMODULE = True
except ImportError:
    import json as _json
    _JSON_IS_UMODULE = False

from fll_lib.runtime import detect_platform


PLATFORM = detect_platform()

PI = 3.141592653589793

# Edit robot settings here. Every create_robot() call loads these values.
DEFAULTS = {
    "wheel_diameter_cm": 5.6,
    "wheel_circumference_cm": 5.6 * PI,
    "track_width_cm": 14.5,
    "left_wheel_port": "B",
    "right_wheel_port": "D",
    "left_manipulator_port": "A",
    "right_manipulator_port": "C",
    "left_wheel_sign": 1,
    "right_wheel_sign": 1,
    "turn_sign": None,
    "pybricks_top_side": "Z",
    "pybricks_front_side": "X",
    "left_manipulator_limits": [-120, 120],
    "right_manipulator_limits": [-120, 120],
    "manipulator_safe_margin": 10,
    "max_speed": 100,
    "gyro_pid": {"kp": 2.0, "ki": 0.1, "kd": 0.3, "integral_limit": 15,
                 "deadzone": 0.5, "deriv_filter": 0.15},
    "turn_control": {
        "settle_tolerance": 1.5,
        "settle_wheel_speed": 45.0,
        "settle_reads": 3,
        "settle_ms": 30,
        "fallback_tolerance": 4.0,
        "stop_mode": "hold",
        "turndown_deg": 25.0,
        "fast_ramp": 28,
        "creep_duty": 30,
        "creep_ramp": 8,
        "stop_deg": 9.0,
        "latch_wait_ms": 60,
        "latch_max_ms": 350,
        "tap_duty": 34,
        "tap_wheel_deg": 8.0,
        "tap_min_deg": 1.5,
        "tap_ratio": 1.6,
        "tap_max_ms": 600,
        "tap_max": 3,
        "tap_revert_deg": 10.0,
        "budget_base_ms": 650,
        "budget_per_deg": 9.0,
        "budget_min_ms": 800,
        "budget_max_ms": 2600,
    },
    "drive_stop_mode": "hold",
    "turn_debug": False,
    "drift_forward_factor": 1.0,
    "drift_backward_factor": 1.0,
    "drive_launch_scale": 0.6,
    "drive_launch_ms": 300,
    "drive_end_ramp_deg": 40.0,
    "drive_end_min_power": 18.0,
    "drive_luff_enable": True,
    "drive_luff_power": 22.0,
    "drive_luff_ms": 30,
    "turn_error_compensation": {
        "enabled": False,
        "alpha": 0.1,
        "min_sample_deg": 1.5,
        "max_correction_deg": 6.0,
        "max_sample_deg": 8.0,
    },
    "drive_speed": 75,
    "turn_speed": 60,
    "manipulator_speed": 50,
    "manipulator_tolerance": 1.0,
    "manipulator_hold": True,
    "release_manips_on_drive": True,
    "left_manipulator_offset": 0.0,
    "right_manipulator_offset": 0.0,
    "manipulator_debug": False,
}


# Mirrored wheel and gyro direction on the physical robot.
# The desktop mock uses the directions from DEFAULTS above.
PYBRICKS_OVERRIDES = {
    "right_wheel_sign": -1,
    "turn_sign": -1,
}

def has_filesystem():
    return PLATFORM != "pybricks"


def _clone_defaults():
    cfg = {}
    for key, value in DEFAULTS.items():
        if isinstance(value, dict):
            cfg[key] = dict(value)
        elif isinstance(value, list):
            cfg[key] = list(value)
        else:
            cfg[key] = value
    if PLATFORM == "pybricks":
        cfg = _merge_deep(cfg, PYBRICKS_OVERRIDES)
    return cfg


def _merge_deep(base, override):
    out = dict(base)
    for key, value in override.items():
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            out[key] = _merge_deep(base[key], value)
        elif key in base and isinstance(base[key], list) and isinstance(value, (list, tuple)):
            out[key] = list(value)
        else:
            out[key] = value
    return out


def load_config(path="config.json"):
    cfg = _clone_defaults()
    if not has_filesystem():
        return cfg
    try:
        with open(path, "r") as f:
            user = _json.load(f)
        if isinstance(user, dict):
            cfg = _merge_deep(cfg, user)
    except (OSError, ValueError, TypeError):
        pass
    return cfg


def merge_config(override):
    cfg = _clone_defaults()
    if isinstance(override, dict):
        cfg = _merge_deep(cfg, override)
    return cfg


def save_config(cfg, path="config.json"):
    if not has_filesystem():
        return
    try:
        with open(path, "w") as f:
            if _JSON_IS_UMODULE:
                _json.dump(cfg, f)
            else:
                _json.dump(cfg, f, indent=2)
    except OSError:
        pass
