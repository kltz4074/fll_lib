import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from fll_lib.core.robot import create_robot
from fll_lib.utils.logging import Logger
from fll_lib.utils import sleep_ms
from fll_lib.missions.path_runner import PathRunner


def run():
    log = Logger(console=True)
    log.clear()
    log.log("=== FLL LIBRARY DEMO ===")

    robot = create_robot(
        left_wheel_port="A",
        right_wheel_port="B",
        left_manipulator_port="C",
        right_manipulator_port="D",
        left_manipulator_limits=(-120, 120),
        right_manipulator_limits=(-120, 120),
    )

    robot.reset_gyro()
    log.log("gyro reset, safe_heading={}".format(robot.heading()))

    log.log("--- 1. drive forward 30 cm (gyro-corrected) ---")
    robot.forward(30)
    log.log("    heading now: {}".format(robot.heading()))

    log.log("--- 2. turn right 90 deg ---")
    robot.turn_right(90)
    log.log("    heading now: {}".format(robot.heading()))

    log.log("--- 3. drive forward 20 cm ---")
    robot.forward(20)

    log.log("--- 4. turn left 180 deg ---")
    robot.turn_left(180)
    log.log("    heading now: {}".format(robot.heading()))

    log.log("--- 5. drive backward 15 cm ---")
    robot.backward(15)
    log.log("    heading now: {}".format(robot.heading()))

    log.log("--- 6. manipulator L: +90 then -90 then 0 ---")
    robot.move_manip_l(90)
    sleep_ms(300)
    robot.move_manip_l(-90)
    sleep_ms(300)
    robot.move_manip_l(0)

    log.log("--- 7. manipulator R: -90 then +90 then 0 ---")
    robot.move_manip_r(-90)
    sleep_ms(300)
    robot.move_manip_r(90)
    sleep_ms(300)
    robot.move_manip_r(0)

    log.log("--- 8. both manipulators together: L +60 / R -60, then reversed ---")
    robot.move_both_manipulators(60, -60)
    sleep_ms(300)
    robot.move_both_manipulators(-60, 60)
    sleep_ms(300)
    robot.move_both_manipulators(0, 0)

    log.log("--- 9. safety check: target 999 gets clamped to safe limit ---")
    ok = robot.move_manip_l(999)
    log.log("    clamped move result: {}".format(ok))
    robot.move_manip_l(0)

    log.log("--- 10. PathRunner: command sequence ---")
    runner = PathRunner(robot)
    (runner
        .reset_gyro()
        .forward(20, 60)
        .turn_right(45, 40)
        .forward(15, 50)
        .turn_left(90, 40)
        .backward(10, 50)
        .both_manips(40, -40, 45)
        .pause(400)
        .both_manips(0, 0, 45)
        .turn_left(45, 40))
    runner.run()

    log.log("--- 11. square loop: 4x (forward 20, turn right 90) ---")
    for i in range(4):
        robot.forward(20, 50)
        robot.turn_right(90, 40)
        log.log("    side {} complete, heading={}".format(i + 1, robot.heading()))

    log.log("--- 12. opportunistic feature: heading reader ---")
    log.log("    final heading={} pitch={} roll={}".format(
        robot.heading(), robot.gyro.pitch(), robot.gyro.roll()))

    robot.stop()
    log.log("=== DEMO COMPLETE ===")


if __name__ == "__main__":
    run()
