from fll_lib.utils.logging import Logger
from fll_lib.control.drift import DriftCompensator
from fll_lib.runtime import detect_platform


PLATFORM = detect_platform()


def calibrate_travel(robot, distance_cm, measured_cm=None, speed=60, logger=None):
    log = logger or Logger(console=True)
    log.log("calibrate: drive {} cm at speed {}, then measure actual distance".format(
        distance_cm, speed))
    robot.gyro.reset()
    robot.forward(distance_cm, speed)
    if measured_cm is None:
        if PLATFORM == "mock":
            log.log("drive done. enter the ACTUAL measured distance in cm:")
            try:
                measured_cm = float(input())
            except Exception:
                measured_cm = distance_cm
        else:
            raise ValueError("on the hub, pass measured_cm to calibrate_travel()")
    factor = DriftCompensator.compute_factor(distance_cm, measured_cm)
    log.log("travel factor = {:.3f}".format(factor))
    return factor


def apply_factors(robot, forward_factor, backward_factor=None):
    bf = forward_factor if backward_factor is None else backward_factor
    robot.drift.forward_factor = forward_factor
    robot.drift.backward_factor = bf
    robot.config["drift_forward_factor"] = forward_factor
    robot.config["drift_backward_factor"] = bf
