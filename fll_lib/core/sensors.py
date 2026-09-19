from fll_lib.runtime import detect_platform
from fll_lib.utils import now_ms


PLATFORM = detect_platform()


class GyroSensor:
    def __init__(self, opts=None):
        self._opts = opts or {}
        self._platform = PLATFORM
        self._mock_yaw = 0.0
        self._hub = None
        self._left = None
        self._right = None
        self._last_tick = now_ms()
        if self._platform == "pybricks":
            from pybricks.hubs import PrimeHub
            from pybricks.parameters import Axis
            top = getattr(Axis, self._opts.get("pybricks_top_side", "Z"), Axis.X)
            front = getattr(Axis, self._opts.get("pybricks_front_side", "X"), Axis.X)
            self._hub = PrimeHub(top_side=top, front_side=front)

        self.hub = self._hub

    def attach_wheels(self, left, right):
        if self._platform == "pybricks":
            return
        self._left = left
        self._right = right
        self._last_lpos = None
        self._last_rpos = None
        self._last_tick = now_ms()

    def yaw(self):
        if self._platform == "pybricks":
            return self._hub.imu.heading()
        if self._left is not None and self._right is not None:
            self._last_tick = now_ms()

            try:
                L = self._left.get_position()
                R = self._right.get_position()
            except Exception:
                return self._mock_yaw
            if self._last_lpos is None:
                self._last_lpos = L
                self._last_rpos = R
                return self._mock_yaw
            dL = L - self._last_lpos
            dR = R - self._last_rpos
            self._last_lpos = L
            self._last_rpos = R
            if abs(dL) > 300.0 or abs(dR) > 300.0:

                return self._mock_yaw
            self._mock_yaw += (dL - dR) * 0.3
        return self._mock_yaw

    def reset(self):
        if self._platform == "pybricks":
            self._hub.imu.reset_heading(0)
        else:
            self._mock_yaw = 0.0

    def pitch(self):
        if self._platform == "pybricks":
            return self._hub.imu.tilt()[0]
        return 0.0

    def roll(self):
        if self._platform == "pybricks":
            return self._hub.imu.tilt()[1]
        return 0.0

    def set_mock_yaw(self, angle):
        self._mock_yaw = angle
