from fll_lib.core.robot import create_robot
from fll_lib.utils import sleep_ms
from fll_lib.utils.logging import Logger


def check_wheel_signs(robot, log):

    log.log("=== wheel sign check (both wheels forward 0.4s @ 35%) ===")
    robot.left_wheel.set_power(35)
    robot.right_wheel.set_power(35)
    sleep_ms(400)
    robot.stop()
    drift = abs(robot.heading())
    log.log("    yaw drift: {:.1f} deg".format(drift))
    if drift > 5.0:
        log.log("    WARNING: wheels fight each other (mirrored motor).")
        log.log("    Set left_wheel_sign=-1 OR right_wheel_sign=-1 in")
        log.log("    fll_lib/config.py until this message disappears.")
    else:
        log.log("    wheels ok (straight drive).")
    robot.reset_gyro()


def run():
    log = Logger(console=True, filepath=None)

    robot = create_robot()

    log.log("robot ready. keep it still 2s while gyro calibrates")
    sleep_ms(2000)
    robot.reset_gyro()
    log.log("gyro reset. heading={}".format(robot.heading()))

    check_wheel_signs(robot, log)

    robot.reset_gyro()
    robot.forward(20, 80)
    robot.turn_left(90, 50)
    robot.turn_right(90, 50)
    robot.backward(20, 80)

    robot.stop()
    log.log("=== PYBRICKS DEMO COMPLETE ===")


if __name__ == "__main__":
    run()
