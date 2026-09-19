import math

from fll_lib.utils import normalize_angle


class PoseTracker:

    def __init__(self, wheel_circumference_cm, x=0.0, y=0.0, heading=0.0):
        self.wheel_circumference_cm = float(wheel_circumference_cm)
        self.reset(x, y, heading)

    def reset(self, x=0.0, y=0.0, heading=0.0):
        self._x = float(x)
        self._y = float(y)
        self._heading = float(heading)
        self._distance = 0.0

    def x(self):
        return self._x

    def y(self):
        return self._y

    def heading(self):
        return self._heading

    def position(self):
        return (self._x, self._y)

    def distance(self):
        return self._distance

    def drift(self):
        return (self._x * self._x + self._y * self._y) ** 0.5

    def set_heading(self, heading_deg):
        self._heading = normalize_angle(float(heading_deg))

    def integrate(self, wheel_deg, heading_deg):
        self._heading = normalize_angle(float(heading_deg))
        d_cm = wheel_deg / 360.0 * self.wheel_circumference_cm
        rad = math.radians(self._heading)
        self._x += d_cm * math.cos(rad)
        self._y += d_cm * math.sin(rad)
        self._distance += abs(d_cm)
