from fll_lib.utils import now_ms, ticks_diff, clamp, normalize_angle


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-100, 100),
                 deadzone=0.0, deriv_filter=0.15, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.deadzone = deadzone
        self.deriv_filter = deriv_filter
        self.integral_limit = integral_limit
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._last_error = None
        self._filtered_deriv = 0.0
        self._last_time = None

    def update(self, error):
        now = now_ms()
        if self._last_time is None:
            dt = 0.02
        else:
            dt = max(ticks_diff(now, self._last_time), 10) / 1000.0
        self._last_time = now

        in_deadzone = abs(error) < self.deadzone
        ctrl_error = 0.0 if in_deadzone else error

        p = ctrl_error * self.kp

        if not in_deadzone:
            self._integral += error * dt
            if self.integral_limit is not None:
                self._integral = clamp(self._integral, -self.integral_limit, self.integral_limit)
        i = self._integral * self.ki

        if self._last_error is not None:

            raw_deriv = (error - self._last_error) / dt
            alpha = dt / (self.deriv_filter + dt)
            self._filtered_deriv = alpha * raw_deriv + (1 - alpha) * self._filtered_deriv
        d = self._filtered_deriv * self.kd
        self._last_error = error

        output = p + i + d
        lo, hi = self.output_limits
        if output > hi:
            output = hi
            if not in_deadzone:
                self._integral -= error * dt
        elif output < lo:
            output = lo
            if not in_deadzone:
                self._integral -= error * dt
        return output


class GyroPID(PIDController):
    def __init__(self, get_heading, target=0.0, **kwargs):
        super().__init__(**kwargs)
        self._get_heading = get_heading
        self._target = target

    def set_target(self, target):
        self._target = target

    def update(self):
        error = normalize_angle(self._target - self._get_heading())
        return super().update(error)

    def error(self):
        return normalize_angle(self._target - self._get_heading())


class PositionPID(PIDController):
    def __init__(self, get_position, target=0.0, **kwargs):
        super().__init__(**kwargs)
        self._get_position = get_position
        self._target = target

    def set_target(self, target):
        self._target = target

    def update(self):
        error = self._target - self._get_position()
        return super().update(error)

    def error(self):
        return self._target - self._get_position()


def clamp_speed(value, max_speed):
    return clamp(value, -max_speed, max_speed)
