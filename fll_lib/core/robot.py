from fll_lib.core.chassis import DifferentialDrive
from fll_lib.core.manipulator import Manipulator
from fll_lib.core.sensors import GyroSensor
from fll_lib.core.motors import WheelDriver
from fll_lib.control.drift import DriftCompensator
from fll_lib.config import load_config, merge_config
from fll_lib.utils import clamp


def _manip_speed(spd):
    return float(clamp(spd, 1, 100))


class Robot:
    def __init__(self, left_wheel, right_wheel, gyro, drift, chassis,
                 manip_l, manip_r, config):
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.gyro = gyro
        self.drift = drift
        self.chassis = chassis
        self.pose = chassis.pose
        self.manip_l = manip_l
        self.manip_r = manip_r
        self.config = config

        self.release_manips_on_drive = bool(
            config.get("release_manips_on_drive", True))

    def _maybe_release_manips(self):
        if self.release_manips_on_drive:
            self.release_manipulators()

    def release_manipulators(self):
        self.manip_l.motor.stop(hold=False)
        self.manip_r.motor.stop(hold=False)

    def coast_manipulators(self):
        self.manip_l.motor.set_power(0)
        self.manip_r.motor.set_power(0)

    def forward(self, distance_cm=None, speed=None):
        cm = distance_cm if distance_cm is not None else 30
        spd = speed if speed is not None else self.config.get("drive_speed", 60)
        self._maybe_release_manips()
        return self.chassis.forward(cm, spd)

    def backward(self, distance_cm=None, speed=None):
        cm = distance_cm if distance_cm is not None else 30
        spd = speed if speed is not None else self.config.get("drive_speed", 60)
        self._maybe_release_manips()
        return self.chassis.backward(cm, spd)

    def turn_right(self, angle_deg=90, speed=None):
        spd = speed if speed is not None else self.config.get("turn_speed", 40)
        self._maybe_release_manips()
        return self.chassis.turn(angle_deg, spd)

    def turn_left(self, angle_deg=90, speed=None):
        spd = speed if speed is not None else self.config.get("turn_speed", 40)
        self._maybe_release_manips()
        return self.chassis.turn(-angle_deg, spd)

    def turn_to_heading(self, heading_deg, speed=None):

        delta = self.chassis.normalize_angle_delta(heading_deg - self.gyro.yaw())
        spd = speed if speed is not None else self.config.get("turn_speed", 40)
        self._maybe_release_manips()
        return self.chassis.turn(delta, spd)

    def pose_x(self):
        return self.chassis.pose.x()

    def pose_y(self):
        return self.chassis.pose.y()

    def pose_heading(self):
        return self.chassis.pose.heading()

    def pose_position(self):
        return self.chassis.pose.position()

    def pose_distance(self):
        return self.chassis.pose.distance()

    def pose_drift(self):
        return self.chassis.pose.drift()

    def reset_pose(self, x=0.0, y=0.0, heading=None):
        self.chassis.pose.reset(x, y,
                                heading if heading is not None
                                else self.gyro.yaw())

    def set_pose_heading(self, heading_deg):
        self.chassis.pose.set_heading(heading_deg)

    def goto(self, x, y, final_heading=None, speed=None):
        dx = x - self.pose.x()
        dy = y - self.pose.y()
        dist = (dx * dx + dy * dy) ** 0.5
        if dist < 0.5:
            if final_heading is None:
                return True
            return self.turn_to_heading(final_heading, speed=speed)
        travel = Robot._atan2_deg(dy, dx)
        delta = self.chassis.normalize_angle_delta(travel - self.pose.heading())
        spd = speed if speed is not None else self.config.get("drive_speed", 60)
        turn_spd = self.config.get("turn_speed", 40)
        self._maybe_release_manips()
        ok = self.chassis.turn(delta, turn_spd)
        if ok:
            ok = self.chassis.forward(dist, spd)
        if ok and final_heading is not None:
            ok = self.turn_to_heading(final_heading, speed=turn_spd)
        return ok

    @staticmethod
    def _atan2_deg(y, x):
        from fll_lib.utils import atan2_deg
        return atan2_deg(y, x)

    def turn_error_report(self):
        return self.chassis.turn_error_report()

    def reset_turn_comp(self):
        self.chassis.reset_turn_comp()

    def stop(self):
        self._maybe_release_manips()
        self.chassis.stop()

    def reset_gyro(self):
        self.gyro.reset()

    def calibrate_turn(self):
        return self.chassis.calibrate_turn_sign()

    def heading(self):
        return self.gyro.yaw()

    def move_manip_l(self, angle, speed=None):
        self.manip_l.speed = _manip_speed(
            speed if speed is not None else self.config.get("manipulator_speed", 50))
        return self.manip_l.move_to(angle)

    def move_manip_r(self, angle, speed=None):
        self.manip_r.speed = _manip_speed(
            speed if speed is not None else self.config.get("manipulator_speed", 50))
        return self.manip_r.move_to(angle)

    def manip_l_angle(self):

        return self.manip_l.angle()

    def manip_r_angle(self):
        return self.manip_r.angle()

    def reset_manip_zero(self):

        self.manip_l.reset_zero()
        self.manip_r.reset_zero()

    def calibrate_manip_angles(self, angle_l, angle_r):

        self.manip_l.calibrate_zero(angle_l)
        self.manip_r.calibrate_zero(angle_r)

    def move_both_manipulators(self, angle_l, angle_r, speed=None):
        spd = _manip_speed(speed if speed is not None else self.config.get("manipulator_speed", 50))
        return Manipulator.move_both(self.manip_l, angle_l, self.manip_r, angle_r, spd)


def create_robot(config=None,
                 left_wheel_port=None, right_wheel_port=None,
                 left_manipulator_port=None, right_manipulator_port=None,
                 left_manipulator_limits=None, right_manipulator_limits=None,
                 left_wheel_sign=None, right_wheel_sign=None,
                 **extra):
    cfg = merge_config(config) if config is not None else load_config()
    if isinstance(cfg.get("gyro_pid"), dict):
        cfg["gyro_pid"] = dict(cfg["gyro_pid"])
    if isinstance(cfg.get("left_manipulator_limits"), list):
        cfg["left_manipulator_limits"] = list(cfg["left_manipulator_limits"])
    if isinstance(cfg.get("right_manipulator_limits"), list):
        cfg["right_manipulator_limits"] = list(cfg["right_manipulator_limits"])
    for key, value in extra.items():
        cfg[key] = value

    if left_wheel_port:
        cfg["left_wheel_port"] = left_wheel_port
    if right_wheel_port:
        cfg["right_wheel_port"] = right_wheel_port
    if left_manipulator_port:
        cfg["left_manipulator_port"] = left_manipulator_port
    if right_manipulator_port:
        cfg["right_manipulator_port"] = right_manipulator_port
    if left_manipulator_limits:
        cfg["left_manipulator_limits"] = list(left_manipulator_limits)
    if right_manipulator_limits:
        cfg["right_manipulator_limits"] = list(right_manipulator_limits)
    if left_wheel_sign is not None:
        cfg["left_wheel_sign"] = left_wheel_sign
    if right_wheel_sign is not None:
        cfg["right_wheel_sign"] = right_wheel_sign

    if cfg.get("left_manipulator_port") == cfg.get("right_manipulator_port"):
        raise ValueError("left_manipulator_port and right_manipulator_port "
                         "must be different ports")

    left = WheelDriver(cfg["left_wheel_port"], is_left=True,
                       sign=cfg.get("left_wheel_sign", 1))
    right = WheelDriver(cfg["right_wheel_port"], is_left=False,
                        sign=cfg.get("right_wheel_sign", 1))
    gyro = GyroSensor(opts=cfg)
    gyro.attach_wheels(left, right)
    drift = DriftCompensator(
        forward=cfg.get("drift_forward_factor", 1.0),
        backward=cfg.get("drift_backward_factor", 1.0),
    )

    lm = WheelDriver(cfg["left_manipulator_port"])
    rm = WheelDriver(cfg["right_manipulator_port"])

    lm_lim = cfg.get("left_manipulator_limits", [-120, 120])
    rm_lim = cfg.get("right_manipulator_limits", [-120, 120])
    safe_margin = cfg.get("manipulator_safe_margin", 10)
    manip_speed = _manip_speed(cfg.get("manipulator_speed", 50))
    hold_at_end = bool(cfg.get("manipulator_hold", True))
    manip_debug = bool(cfg.get("manipulator_debug", False))

    manip_l = Manipulator(motor=lm,
                          min_angle=lm_lim[0], max_angle=lm_lim[1],
                          safe_margin=safe_margin, speed=manip_speed,
                          tolerance=cfg.get("manipulator_tolerance", 1.0),
                          hold_at_end=hold_at_end,
                          offset=cfg.get("left_manipulator_offset", 0.0),
                          debug=manip_debug)
    manip_r = Manipulator(motor=rm,
                          min_angle=rm_lim[0], max_angle=rm_lim[1],
                          safe_margin=safe_margin, speed=manip_speed,
                          tolerance=cfg.get("manipulator_tolerance", 1.0),
                          hold_at_end=hold_at_end,
                          offset=cfg.get("right_manipulator_offset", 0.0),
                          debug=manip_debug)

    chassis = DifferentialDrive(left, right, gyro, drift, cfg)

    return Robot(
        left_wheel=left, right_wheel=right,
        gyro=gyro, drift=drift, chassis=chassis,
        manip_l=manip_l, manip_r=manip_r,
        config=cfg,
    )
