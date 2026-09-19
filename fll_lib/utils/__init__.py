try:
    import time as _time
except ImportError:
    _time = None

from fll_lib.runtime import detect_platform

PLATFORM = detect_platform()

if PLATFORM == "pybricks":
    from pybricks.tools import StopWatch
    from pybricks.tools import wait as _pb_wait
    _sw = StopWatch()
else:
    _sw = None
    _pb_wait = None


def now_ms():
    if _sw is not None:
        return _sw.time()
    if _time is not None:
        return int(_time.time() * 1000)
    return 0


def ticks_diff(new, old):
    return new - old


def sleep_ms(ms):
    ms = int(ms)
    if _pb_wait is not None:
        _pb_wait(ms)
        return
    if _time is not None:
        _time.sleep(ms / 1000.0)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def atan2_deg(y, x):
    import math
    return math.degrees(math.atan2(y, x))
