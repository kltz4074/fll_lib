from fll_lib.utils import clamp, sleep_ms, now_ms, ticks_diff


class Manipulator:
    def __init__(self, motor, min_angle=-120, max_angle=120,
                 safe_margin=10.0, speed=300, tolerance=1.0,
                 max_travel_time_ms=5000, hold_at_end=True, offset=0.0,
                 debug=False):
        self.motor = motor
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.safe_margin = float(safe_margin)
        self.speed = float(clamp(speed, 1, 100))
        self.tolerance = tolerance
        self.max_travel_time_ms = max_travel_time_ms
        self.hold_at_end = bool(hold_at_end)
        self.debug = bool(debug)

        self.global_offset = float(offset)

    def angle(self):

        return self.motor.get_position() - self.global_offset

    def calibrate_zero(self, physical_angle=0.0):

        self.global_offset = self.motor.get_position() - float(physical_angle)
        return self.angle()

    def reset_zero(self):
        return self.calibrate_zero(0.0)

    def _safe_target(self, target):
        return clamp(target,
                     self.min_angle + self.safe_margin,
                     self.max_angle - self.safe_margin)

    def _outside_limits(self, pos):
        return pos < self.min_angle or pos > self.max_angle

    def move_to(self, target):
        target = self._safe_target(target)
        start_angle = self.angle()

        if self._outside_limits(start_angle):
            if self.debug:
                print("MANIP move_to ABORT start outside"
                      " ->{} start={:.1f}".format(target, start_angle))
            self.motor.stop(hold=True)
            return False

        start_time = now_ms()
        motor_target = target + self.global_offset
        if self.debug:
            print("MANIP move_to ->{:.1f} start={:.1f}".format(
                target, start_angle))
        self.motor.run_to_angle(self.speed, motor_target,
                                hold=self.hold_at_end)

        sleep_ms(30)

        while not self.motor.done():
            pos = self.angle()
            if self._outside_limits(pos):

                if self.debug:
                    print("MANIP move_to ABORT hit limit"
                          " ->{} at={:.1f}".format(target, pos))
                self.motor.stop(hold=True)
                return False
            if ticks_diff(now_ms(), start_time) > self.max_travel_time_ms:
                if self.debug:
                    print("MANIP move_to TIMEOUT"
                          " ->{} at={:.1f}".format(target, self.angle()))
                self.motor.stop(hold=self.hold_at_end)
                return False
            sleep_ms(20)

        ok = abs(self.angle() - target) <= self.tolerance
        if self.debug:
            print("MANIP move_to done ok={} ->{} at={:.1f}".format(
                ok, target, self.angle()))
        return ok

    def stop(self):
        self.motor.stop(hold=True)

    @staticmethod
    def move_both(mani_a, angle_a, mani_b, angle_b, speed=100):
        target_a = mani_a._safe_target(angle_a)
        target_b = mani_b._safe_target(angle_b)
        speed = clamp(speed, 1, 100)

        if (mani_a._outside_limits(mani_a.angle()) or
                mani_b._outside_limits(mani_b.angle())):
            if mani_a.debug or mani_b.debug:
                print("MANIP move_both ABORT start outside limits"
                      " ->{}/{}".format(target_a, target_b))
            mani_a.motor.stop(hold=True)
            mani_b.motor.stop(hold=True)
            return False

        if mani_a.debug or mani_b.debug:
            print("MANIP move_both ->{}/{} starts {:.1f}/{:.1f}".format(
                target_a, target_b, mani_a.angle(), mani_b.angle()))
        mani_a.motor.run_to_angle(speed, target_a + mani_a.global_offset,
                                  hold=mani_a.hold_at_end)
        mani_b.motor.run_to_angle(speed, target_b + mani_b.global_offset,
                                  hold=mani_b.hold_at_end)
        sleep_ms(30)

        start_time = now_ms()
        done_a = False
        done_b = False

        while not (done_a and done_b):
            if not done_a:
                if mani_a.motor.done():
                    done_a = (abs(mani_a.angle() - target_a)
                              <= mani_a.tolerance)
                elif mani_a._outside_limits(mani_a.angle()):
                    if mani_a.debug or mani_b.debug:
                        print("MANIP move_both ABORT A hit limit"
                              " at={:.1f}".format(mani_a.angle()))
                    mani_a.motor.stop(hold=True)
                    mani_b.motor.stop(hold=True)
                    return False
            if not done_b:
                if mani_b.motor.done():
                    done_b = (abs(mani_b.angle() - target_b)
                              <= mani_b.tolerance)
                elif mani_b._outside_limits(mani_b.angle()):
                    if mani_a.debug or mani_b.debug:
                        print("MANIP move_both ABORT B hit limit"
                              " at={:.1f}".format(mani_b.angle()))
                    mani_a.motor.stop(hold=True)
                    mani_b.motor.stop(hold=True)
                    return False
            sleep_ms(20)
            if ticks_diff(now_ms(), start_time) > 6000:
                if mani_a.debug or mani_b.debug:
                    print("MANIP move_both TIMEOUT"
                          " at={:.1f}/{:.1f}".format(
                              mani_a.angle(), mani_b.angle()))
                mani_a.motor.stop(hold=mani_a.hold_at_end)
                mani_b.motor.stop(hold=mani_b.hold_at_end)
                return False

        return True
