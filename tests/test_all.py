import os
import sys
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import unittest

from fll_lib.utils import normalize_angle, clamp
from fll_lib.control.pid import PIDController, GyroPID
from fll_lib.control.drift import DriftCompensator
from fll_lib.core.manipulator import Manipulator
from fll_lib.core.chassis import DifferentialDrive
from fll_lib.core.robot import Robot, create_robot
from fll_lib.missions.strategy import MissionPlan
from fll_lib.missions.scoring import ScoringSystem
from fll_lib.mocks import MockGyro
from fll_lib.utils.logging import Logger
from fll_lib.runtime import detect_platform


class TestUtils(unittest.TestCase):
    def test_clamp(self):
        self.assertEqual(clamp(5, 0, 10), 5)
        self.assertEqual(clamp(-1, 0, 10), 0)
        self.assertEqual(clamp(15, 0, 10), 10)

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(190), -170)
        self.assertAlmostEqual(normalize_angle(-190), 170)
        self.assertAlmostEqual(normalize_angle(45), 45)


class TestPID(unittest.TestCase):
    def test_p_term(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, output_limits=(-100, 100))
        out = pid.update(10.0)
        self.assertAlmostEqual(out, 20.0, delta=1.0)

    def test_output_clamped(self):
        pid = PIDController(kp=50.0, ki=0.0, kd=0.0, output_limits=(-100, 100))
        self.assertEqual(pid.update(10.0), 100)
        self.assertEqual(pid.update(-10.0), -100)

    def test_anti_windup_integral_not_exploding(self):
        pid = PIDController(kp=1.0, ki=10.0, kd=0.0, output_limits=(-100, 100))
        for _ in range(20):
            pid.update(50.0)
        self.assertLessEqual(pid._integral, 5)

    def test_converges_with_mock_gyro(self):
        gyro = MockGyro()
        gyro.reset()
        pid = GyroPID(get_heading=gyro.yaw, target=90.0,
                      kp=1.5, ki=0.0, kd=0.0, output_limits=(-100, 100))
        for _ in range(300):
            correction = pid.update()
            gyro.set_yaw(gyro.yaw() + correction * 0.02)
        self.assertAlmostEqual(gyro.yaw(), 90.0, delta=3.0)


class TestDrift(unittest.TestCase):
    def test_compensate(self):
        dc = DriftCompensator(forward=0.95)
        self.assertAlmostEqual(dc.compensate(100, "forward"), 100 / 0.95, delta=0.1)

    def test_compute_factor(self):
        factor = DriftCompensator.compute_factor(100, 95)
        self.assertAlmostEqual(factor, 0.95, delta=0.01)


class TestManipulatorSafety(unittest.TestCase):
    def _make(self):
        from fll_lib.mocks import MockMotor
        motor = MockMotor("manip")
        manip = Manipulator(motor, min_angle=-90, max_angle=90,
                            safe_margin=10, speed=100)
        return motor, manip

    def test_target_clamped_to_safe(self):
        motor, manip = self._make()
        manip.move_to(999)
        self.assertLessEqual(motor.get_position(), 80)
        self.assertGreaterEqual(motor.get_position(), -80)

    def test_never_reaches_limit(self):
        motor, manip = self._make()
        manip.move_to(90)
        manip.move_to(-90)
        self.assertLess(abs(motor.get_position()), 80 + 2)

    def test_move_reaches_position(self):
        motor, manip = self._make()
        ok = manip.move_to(45)
        self.assertTrue(ok)
        self.assertLessEqual(abs(motor.get_position() - 45), manip.tolerance)

    def test_move_holds_after_goal(self):
        motor, manip = self._make()
        manip.move_to(30)
        self.assertIsNone(motor._goal)

    def test_move_aborts_when_start_outside_limits(self):
        motor, manip = self._make()
        motor.reset_position(95)
        ok = manip.move_to(0)
        self.assertFalse(ok)
        self.assertGreaterEqual(motor.get_position(), 90)

    def test_reset_zero_realigns_zero(self):
        motor, manip = self._make()
        motor.reset_position(150)
        manip.reset_zero()
        self.assertEqual(manip.angle(), 0)
        self.assertEqual(motor.get_position(), 150)

    def test_global_zero_independent_of_boot_position(self):
        motor, manip = self._make()

        motor.reset_position(123)
        manip.calibrate_zero(0)
        ok = manip.move_to(45)
        self.assertTrue(ok)
        self.assertLessEqual(abs(manip.angle() - 45), manip.tolerance)
        self.assertLessEqual(abs(motor.get_position() - (45 + 123)),
                             manip.tolerance)

    def test_global_calibration_arbitrary_reference(self):
        motor, manip = self._make()
        motor.reset_position(45)
        manip.calibrate_zero(90)
        ok = manip.move_to(0)
        self.assertTrue(ok)
        self.assertLessEqual(abs(manip.angle() - 0), manip.tolerance)

        self.assertLessEqual(abs(motor.get_position() - (-45)),
                             manip.tolerance)


class TestMissions(unittest.TestCase):
    def test_best_plan(self):
        plan = MissionPlan(available_time_s=25)
        plan.add("deliver", 30, 10)
        plan.add("collect", 20, 10)
        plan.add("boost", 15, 5)
        selected, points = plan.best_plan()
        self.assertGreater(len(selected), 0)
        self.assertGreater(points, 0)

    def test_scoring(self):
        s = ScoringSystem()
        s.add("t1", 20)
        s.mark("t1", True)
        self.assertEqual(s.total_score(), 20)
        self.assertEqual(s.remaining(), [])


class TestChassis(unittest.TestCase):
    def _make_robot(self):
        from fll_lib.mocks import MockHub
        hub = MockHub()
        gyro = hub.gyro
        gyro.reset()
        cfg = {
            "wheel_circumference_cm": 5.6 * math.pi,
            "max_speed": 100,
            "gyro_pid": {"kp": 2.0, "ki": 0.0, "kd": 0.5},
            "drive_speed": 60, "turn_speed": 40,
            "manipulator_speed": 50, "manipulator_safe_margin": 10,
            "manipulator_tolerance": 1.0,

            "turn_sign": 1,
        }
        chassis = DifferentialDrive(hub.left_motor, hub.right_motor, gyro,
                                    DriftCompensator(), cfg)
        return hub, gyro, chassis, cfg

    def test_forward_tracks_heading(self):
        hub, gyro, chassis, _ = self._make_robot()
        gyro.set_yaw(90)
        chassis.forward(20, 60)
        self.assertAlmostEqual(gyro.yaw(), 90, delta=1.0)

    def test_turn_changes_heading(self):
        hub, gyro, chassis, _ = self._make_robot()
        gyro.set_yaw(0)

        chassis.turn(180, 40)
        self.assertAlmostEqual(gyro.yaw() % 360, 180, delta=5.0)

    def test_turn_below_stiction_stalls(self):
        hub, gyro, chassis, _ = self._make_robot()
        gyro.set_yaw(0)

        self.assertFalse(chassis.turn(180, 20))

    def test_turn_error_compensation_learns(self):
        hub, gyro, chassis, cfg = self._make_robot()

        cfg["turn_error_compensation"] = {"enabled": True,
                                          "min_sample_deg": 0.0}
        gyro.set_yaw(0)
        report = chassis.turn_error_report()
        self.assertAlmostEqual(report["ccw"], 0.0, delta=0.01)
        chassis.turn(90, 40)
        chassis.turn(90, 40)
        after = chassis.turn_error_report()
        self.assertEqual(after["ccw"], 0.0)
        self.assertEqual(after["cw"], 0.0)
        chassis.reset_turn_comp()
        self.assertEqual(chassis.turn_error_report()["ccw"], 0.0)

    def test_turn_comp_ema_equation(self):
        _, gyro, chassis, cfg = self._make_robot()
        cfg["turn_error_compensation"] = {"enabled": True,
                                          "min_sample_deg": 0.0}
        a = chassis._comp_alpha
        self.assertAlmostEqual(a, 0.1, delta=1e-9)

        chassis._turn_comp = {1: 0.0, -1: 0.0}
        chassis._turn_comp[1] = (1 - a) * chassis._turn_comp[1] + a * 2.0
        chassis._turn_comp[1] = (1 - a) * chassis._turn_comp[1] + a * -1.0
        self.assertAlmostEqual(chassis._turn_comp[1], 0.08, delta=1e-9)
        self.assertEqual(chassis.turn_error_report()["ccw"],
                         chassis._turn_comp[1])

    def test_pose_tracks_forward(self):
        hub, gyro, chassis, cfg = self._make_robot()
        gyro.set_yaw(0)
        chassis.forward(10, 60)

        self.assertGreater(chassis.pose.x(), 5.0)
        self.assertLess(abs(chassis.pose.y()), 2.0)
        self.assertAlmostEqual(chassis.pose.heading(), 0, delta=1.0)
        self.assertGreater(chassis.pose.distance(), 5.0)

    def test_pose_heading_follows_turn(self):
        hub, gyro, chassis, cfg = self._make_robot()
        gyro.set_yaw(0)
        chassis.turn(90, 40)

        self.assertAlmostEqual(chassis.pose.heading() % 360, 90, delta=5.0)

    def test_drive_end_luff_preloads(self):
        from fll_lib.utils import now_ms
        hub, gyro, chassis, cfg = self._make_robot()
        cfg["drive_luff_enable"] = True
        cfg["drive_luff_power"] = 22.0
        cfg["drive_luff_ms"] = 40
        chassis.forward(5, 60)

        self.assertLess(chassis.pose.distance(), 8.0)


class TestConfigurationAndAdapters(unittest.TestCase):
    def test_config_defaults_are_isolated(self):
        from fll_lib.config import load_config
        a = load_config()
        a["gyro_pid"]["kp"] = 999
        b = load_config()
        self.assertEqual(b["gyro_pid"]["kp"], 2.0)

    def test_config_partial_override_merges(self):
        from fll_lib.config import load_config
        cfg = load_config("does_not_exist_anyway.json")
        a = create_robot(config={"gyro_pid": {"kp": 12.0}})
        self.assertEqual(a.config["gyro_pid"]["kp"], 12.0)
        self.assertEqual(a.config["gyro_pid"]["ki"], 0.1)
        self.assertEqual(cfg["wheel_diameter_cm"], 5.6)

    def test_wheel_sign_is_applied_once(self):
        from fll_lib.core.motors import WheelDriver
        wheel = WheelDriver("A", sign=-1)
        wheel.set_power(25)
        self.assertEqual(wheel._driver._power, -25)
        wheel.stop()

    def test_platform_is_mock_on_desktop(self):
        self.assertEqual(detect_platform(), "mock")


class TestRobotBehaviour(unittest.TestCase):
    def test_same_manipulator_port_raises(self):
        with self.assertRaises(ValueError):
            create_robot(left_manipulator_port="C", right_manipulator_port="C")

    def test_manipulator_speed_clamped(self):
        robot = create_robot(
            left_manipulator_port="C", right_manipulator_port="D")
        robot.move_manip_l(30, speed=300)
        self.assertEqual(robot.manip_l.speed, 100.0)


class TestMissionLogic(unittest.TestCase):
    def test_scoring_passes_context_to_score_function(self):
        s = ScoringSystem()
        s.add("task", 25, lambda state: state == "done")
        self.assertEqual(s.evaluate("done"), 25)

    def test_strategy_can_unlock_prerequisite_sorted_later(self):
        plan = MissionPlan(available_time_s=20)
        plan.add("finish", 100, 10, requires=["setup"])
        plan.add("setup", 1, 10)
        selected, points = plan.best_plan()
        self.assertEqual([m.name for m in selected], ["setup", "finish"])
        self.assertEqual(points, 101)


class TestCalibration(unittest.TestCase):
    def test_calibrate_accepts_measured_distance(self):
        from fll_lib.utils.calibration import calibrate_travel

        class FakeRobot:
            def __init__(self):
                self.calls = []

                class FakeGyro:
                    def reset(self):
                        pass

                self.gyro = FakeGyro()
            def forward(self, cm, speed):
                self.calls.append(("forward", cm, speed))

        robot = FakeRobot()
        factor = calibrate_travel(robot, distance_cm=100, measured_cm=95,
                                  speed=60, logger=Logger(console=False, filepath=None))
        self.assertAlmostEqual(factor, 0.95, delta=0.01)
        self.assertIn(("forward", 100, 60), robot.calls)


class TestPybricksCompatibilitySurface(unittest.TestCase):
    def test_public_attributes_do_not_require_property(self):
        from fll_lib.core.motors import WheelDriver
        from fll_lib.core.sensors import GyroSensor
        wheel = WheelDriver("A")
        self.assertEqual(wheel.name, "A")
        wheel.stop()

        gyro = GyroSensor()
        self.assertIsNone(gyro.hub)

    def test_calibration_imports_drift_compensator(self):
        from fll_lib.utils.calibration import calibrate_travel
        self.assertTrue(callable(calibrate_travel))

    def test_missions_exports(self):
        from fll_lib.missions import Mission, MissionPlan, ScoringSystem, PathRunner
        self.assertTrue(callable(MissionPlan))
        self.assertTrue(callable(ScoringSystem))
        self.assertTrue(callable(PathRunner))


if __name__ == "__main__":
    unittest.main(verbosity=2)
