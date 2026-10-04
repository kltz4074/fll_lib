import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from fll_lib.core.robot import create_robot
from fll_lib.utils.logging import Logger
from fll_lib.utils import sleep_ms
from fll_lib.missions.path_runner import PathRunner


def simulate():
    robot = create_robot()
    log = Logger(console=True, filepath=None)

    def say(msg):
        log.log(msg)

    say("--- PC SIMULATION of FLL demo ---")
    robot.reset_gyro()
    say("heading start={:.1f}".format(robot.heading()))

    robot.forward(20, 60)
    say("forward 20cm -> heading={:.1f}".format(robot.heading()))

    robot.turn_right(90, 40)
    say("turn right 90 -> heading={:.1f}".format(robot.heading()))

    robot.forward(15, 50)
    say("forward 15cm -> heading={:.1f}".format(robot.heading()))

    robot.turn_left(180, 40)
    say("turn left 180 -> heading={:.1f}".format(robot.heading()))

    robot.backward(10, 50)
    say("backward 10cm -> heading={:.1f}".format(robot.heading()))

    robot.move_manip_l(90)
    say("manip L +90 -> pos={}".format(robot.manip_l.motor.get_position()))
    robot.move_manip_l(-90)
    say("manip L -90 -> pos={}".format(robot.manip_l.motor.get_position()))
    robot.move_manip_l(0)

    robot.move_manip_r(-90)
    say("manip R -90 -> pos={}".format(robot.manip_r.motor.get_position()))
    robot.move_manip_r(90)
    robot.move_manip_r(0)

    robot.move_both_manipulators(60, -60)
    say("both manips -> L={} R={}".format(
        robot.manip_l.motor.get_position(), robot.manip_r.motor.get_position()))
    robot.move_both_manipulators(-60, 60)
    robot.move_both_manipulators(0, 0)

    say("safety: manip L target 999 (clamped)")
    robot.move_manip_l(999)
    say("clamped pos={}".format(robot.manip_l.motor.get_position()))
    robot.move_manip_l(0)

    runner = PathRunner(robot)
    (runner
        .reset_gyro()
        .forward(20, 60)
        .turn_right(45, 40)
        .forward(15, 50)
        .turn_left(90, 40)
        .backward(10, 50)
        .both_manips(40, -40, 45)
        .pause(100)
        .both_manips(0, 0, 45)
        .turn_left(45, 40))
    runner.run()
    say("pathrunner done, heading={:.1f}".format(robot.heading()))

    for i in range(4):
        robot.forward(20, 50)
        robot.turn_right(90, 40)
        say("square side {} heading={:.1f}".format(i + 1, robot.heading()))

    robot.stop()
    say("--- SIMULATION COMPLETE ---")


if __name__ == "__main__":
    simulate()
