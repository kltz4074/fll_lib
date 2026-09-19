from fll_lib.core.robot import create_robot
from fll_lib.utils.logging import Logger
from fll_lib.utils import sleep_ms


def run():
    log = Logger(console=True, filepath=None)

    robot = create_robot(
        left_wheel_port="B",
        right_wheel_port="D",
        left_manipulator_port="A",
        right_manipulator_port="C",
        left_manipulator_limits=(-180, 180),
        right_manipulator_limits=(-180, 180),
        left_wheel_sign=1,
        right_wheel_sign=-1,
        turn_sign=-1,
        turn_debug=False,
    )

    log.log("MANIPULATOR ANGLE TEST")
    log.log("Put arms in a safe pose, then wait 2 s...")
    sleep_ms(2000)

    robot.reset_manip_zero()
    sleep_ms(300)
    robot.manip_r.move_to(90)
    robot.manip_l.move_to(-90)
    sleep_ms(400)
    robot.manip_r.move_to(-90)
    robot.manip_l.move_to(90)
    sleep_ms(400)
    robot.move_both_manipulators(0, 0, 60)





    robot.stop()
    log.log("=== MANIPULATOR ANGLE TEST COMPLETE ===")

    try:
        if robot.gyro.hub is not None:
            for _ in range(3):
                robot.gyro.hub.speaker.beep(1000, 150)
                sleep_ms(200)
    except Exception:
        pass


if __name__ == "__main__":
    run()
