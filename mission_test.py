from fll_lib.core.robot import create_robot
from fll_lib.runtime import detect_platform
from fll_lib.utils.logging import Logger
from fll_lib.utils import sleep_ms


def run():
    log = Logger(console=True, filepath=None)

    on_mock = detect_platform() == "mock"
    robot = create_robot(
        left_wheel_port="B",
        right_wheel_port="D",
        left_manipulator_port="A",
        right_manipulator_port="C",
        left_manipulator_limits=(-120, 120),
        right_manipulator_limits=(-120, 120),
        left_wheel_sign=1,
        right_wheel_sign=-1 if not on_mock else 1,
        turn_sign=-1 if not on_mock else None,
        turn_debug=True,
    )

    log.log("=== ACCURACY TEST: back-home round trip ===")
    log.log("keep robot still 2s while the gyro calibrates")
    sleep_ms(2000)
    robot.reset_gyro()
    robot.reset_pose(0, 0)
    log.log("gyro reset. heading={:.1f}".format(robot.heading()))

    def current_pose():
        return robot.heading(), robot.pose_distance()

    pose_log = []

    def record_label(label):
        hdg, dist = current_pose()
        pose_log.append((label, hdg, dist))
        log.log("{:14s} hdg={:+6.2f}  wheel_dist={:6.2f} cm".format(label, hdg, dist))

    def leg(distance_cm, speed=60):
        robot.forward(distance_cm, speed)

    def turn(angle_deg, speed=40):
        robot.turn_right(angle_deg, speed)

    record_label("start")

    leg(40, 60)
    turn(90, 40)
    leg(30, 60)
    robot.move_both_manipulators(90, -90)
    turn(90, 40)
    robot.move_both_manipulators(-90, 90)
    turn(360, 40)
    leg(40, 60)
    turn(90, 40)
    leg(30, 60)
    turn(90, 40)

    leg(25, 50)
    record_label("out (25cm)")
    turn(180, 40)
    record_label("after 180")
    leg(25, 50)
    record_label("back (25cm)")
    turn(180, 40)
    record_label("after final 180")

    log.log("")
    log.log("--- position (odometry) vs start ---")
    log.log("odometry x={:+.2f} cm  y={:+.2f} cm  |drift|={:.2f} cm".format(
        robot.pose_x(), robot.pose_y(), robot.pose_drift()))
    log.log("final gyro heading offset vs start: {:+.2f} deg".format(
        robot.heading() - 360.0 if robot.heading() > 180.0 else robot.heading()))
    log.log("")

    log.log("--- leg table ---")
    for label, hdg, dist in pose_log:
        log.log("  {:14s} hdg={:+6.2f}  wheel_dist={:6.2f} cm".format(
            label, hdg, dist))
    log.log("")

    robot.stop()
    log.log("=== ACCURACY TEST COMPLETE ===")

    try:
        if robot.gyro.hub is not None:
            for _ in range(3):
                robot.gyro.hub.speaker.beep(1000, 150)
                sleep_ms(200)
    except Exception:
        pass


if __name__ == "__main__":
    run()
