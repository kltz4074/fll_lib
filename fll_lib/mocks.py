from fll_lib.utils import now_ms


class MockMotor:

    MAX_ACCEL = 2200.0
    BREAKAWAY_DUTY = 26.0

    def __init__(self, name="mock_motor"):
        self.name = name
        self._power = 0
        self._deg_per_sec = 0
        self._target_rate = 0
        self._position = 0
        self._running = False
        self._goal = None
        self._goal_speed = 0
        self._last_tick = now_ms()

    def _tick(self):
        now = now_ms()
        dt = (now - self._last_tick) / 1000.0
        self._last_tick = now
        if dt > 0:
            if self._goal is not None:
                error = self._goal - self._position
                if abs(error) <= 1.0:
                    self._position = self._goal
                    self._goal = None
                    self._goal_speed = 0
                    self._deg_per_sec = 0
                    self._target_rate = 0
                    self._running = False
                else:

                    step_vel = max(30.0, min(abs(error) * 8.0,
                                             self._goal_speed * 10.0))
                    self._position += (1 if error > 0 else -1) * step_vel * dt
            else:

                target = self._target_rate
                if self._deg_per_sec == 0 and abs(target) <= self.BREAKAWAY_DUTY * 10:
                    rate = 0.0
                else:
                    diff = target - self._deg_per_sec
                    step = self.MAX_ACCEL * dt
                    if diff > step:
                        rate = self._deg_per_sec + step
                    elif diff < -step:
                        rate = self._deg_per_sec - step
                    else:
                        rate = target
                self._deg_per_sec = rate
                self._position += rate * dt

    def set_power(self, percent):
        self._tick()
        self._power = max(-100, min(100, int(percent)))
        self._target_rate = self._power * 10
        self._running = self._power != 0
        self._goal = None
        self._goal_speed = 0

    def stop(self, hold=False):
        self._tick()
        self._power = 0
        self._deg_per_sec = 0
        self._target_rate = 0
        self._running = False
        self._goal = None
        self._goal_speed = 0

    def run_to_angle(self, speed_percent, target_deg, hold=False):
        self._tick()
        self._goal = float(target_deg)
        self._goal_speed = int(max(1, min(100, speed_percent)))
        self._deg_per_sec = self._goal_speed * 10
        self._target_rate = self._deg_per_sec
        self._running = True

    def done(self):
        self._tick()
        return self._goal is None

    def get_position(self):
        self._tick()
        return self._position

    def reset_position(self, pos=0):
        self._tick()
        self._position = pos

    def velocity(self):
        self._tick()
        return self._deg_per_sec

    def status(self):
        self._tick()
        return 1 if self._running else 0

    def advance(self, dt_ms=20):
        self._tick()


class MockGyro:
    YAW_RESPONSE = 0.3
    RESET_DETECT_DEG = 300.0

    def __init__(self, left=None, right=None):
        self._yaw = 0.0
        self._left = left
        self._right = right
        self._last_tick = now_ms()
        self._last_lpos = None
        self._last_rpos = None

    def _tick(self):
        now = now_ms()
        self._last_tick = now
        if self._left is None or self._right is None:
            return

        L = self._left.get_position()
        R = self._right.get_position()
        if self._last_lpos is None:
            self._last_lpos = L
            self._last_rpos = R
            return
        dL = L - self._last_lpos
        dR = R - self._last_rpos
        self._last_lpos = L
        self._last_rpos = R
        if abs(dL) > self.RESET_DETECT_DEG or abs(dR) > self.RESET_DETECT_DEG:

            return
        self._yaw += (dL - dR) * self.YAW_RESPONSE

    def yaw(self):
        self._tick()
        return self._yaw

    def reset(self):
        self._tick()
        self._yaw = 0.0

    def set_yaw(self, angle):
        self._tick()
        self._yaw = angle

    def pitch(self):
        return 0.0

    def roll(self):
        return 0.0


class MockHub:
    def __init__(self):
        self.left_motor = MockMotor("left_wheel")
        self.right_motor = MockMotor("right_wheel")
        self.left_manipulator = MockMotor("left_manipulator")
        self.right_manipulator = MockMotor("right_manipulator")
        self.gyro = MockGyro(left=self.left_motor, right=self.right_motor)

    def advance(self, dt_ms=20):
        self.left_motor.advance(dt_ms)
        self.right_motor.advance(dt_ms)
        self.left_manipulator.advance(dt_ms)
        self.right_manipulator.advance(dt_ms)
