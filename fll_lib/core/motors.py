from fll_lib.runtime import detect_platform


PLATFORM = detect_platform()


def _make_driver(port_name):
    if PLATFORM == "pybricks":
        return _PybricksDriver(port_name)
    from fll_lib.mocks import MockMotor
    return MockMotor(port_name)


class WheelDriver:
    def __init__(self, port_name, is_left=None, sign=1):
        self.port_name = port_name
        self.is_left = is_left
        self.sign = -1 if sign < 0 else 1
        self._driver = _make_driver(port_name)

        self.name = self.port_name

    def set_power(self, percent):
        self._driver.set_power(percent * self.sign)

    def run_to_angle(self, speed_percent, target_deg, hold=False):
        try:
            return bool(self._driver.run_to_angle(speed_percent,
                                                  target_deg * self.sign,
                                                  hold))
        except Exception:
            return False

    def done(self):
        try:
            return bool(self._driver.done())
        except Exception:
            return True

    def stop(self, hold=False):
        self._driver.stop(hold)

    def get_position(self):
        return self._driver.get_position() * self.sign

    def reset_position(self, pos=0):
        self._driver.reset_position(pos * self.sign)

    def velocity(self):
        try:
            return self._driver.velocity() * self.sign
        except Exception:
            return 0

    def status(self):
        try:
            return self._driver.status()
        except Exception:
            return 0

    def advance(self, dt_ms=20):
        try:
            self._driver.advance(dt_ms)
        except Exception:
            pass


class _PybricksDriver:
    def __init__(self, port_name):
        from pybricks.pupdevices import Motor
        from pybricks.parameters import Port
        name = str(port_name).upper()
        if "PORT_" in name:
            name = name.rsplit("PORT_", 1)[-1]
        self._port = getattr(Port, name)

        self._motor = Motor(self._port, reset_angle=False)
        self._running = False
        self._goal = None

    def set_power(self, percent):
        percent = max(-100, min(100, int(percent)))
        self._motor.dc(percent)
        self._running = percent != 0
        self._goal = None

    def stop(self, hold=False):
        if hold:
            self._motor.hold()
        else:
            self._motor.brake()
        self._running = False
        self._goal = None

    def run_to_angle(self, speed_percent, target_deg, hold=False):

        from pybricks.parameters import Stop
        speed = int(max(10, min(100, speed_percent)) * 10)
        self._goal = int(target_deg)
        stop_mode = Stop.HOLD if hold else Stop.COAST
        self._motor.run_target(speed, self._goal, then=stop_mode, wait=False)
        self._running = True

    def done(self):
        if self._goal is None:
            return True
        try:
            return bool(self._motor.done())
        except Exception:
            return True

    def get_position(self):
        return self._motor.angle()

    def reset_position(self, pos=0):
        self._motor.reset_angle(pos)

    def velocity(self):
        return self._motor.speed()

    def status(self):
        return 1 if self._running else 0

    def advance(self, dt_ms=20):
        pass
