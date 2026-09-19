import os
import subprocess
import sys
import textwrap
import unittest

_REPO_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

_STUB_SCRIPT = textwrap.dedent(
    r"""
    import sys
    import types

    def _stub(name, **attrs):
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        return mod

    _stub("pybricks")

    class _Axis:
        X = 1
        Y = 2
        Z = 3

    class _Port:
        A = 0
        B = 1
        C = 2
        D = 3
        E = 4
        F = 5
        PORT_A = 0
        PORT_B = 1
        PORT_C = 2
        PORT_D = 3
        PORT_E = 4
        PORT_F = 5

    class _Motor:
        def __init__(self, port, reset_angle=False):
            self.port = port
            self._angle = 0
            self._dc = 0
            self._braked = False

        def dc(self, duty):
            self._dc = int(duty)

        def angle(self):
            return self._angle

        def reset_angle(self, angle):
            self._angle = angle

        def speed(self):
            return 0

        def brake(self):
            self._braked = True

        def hold(self):
            self._braked = True

    class _IMU:
        def __init__(self):
            self._h = 0

        def heading(self):
            return self._h

        def reset_heading(self, angle):
            self._h = angle

        def tilt(self):
            return (0, 0)

    class _Speaker:
        def beep(self, freq, duration):
            pass

    class _PrimeHub:
        def __init__(self, top_side=None, front_side=None):
            self.imu = _IMU()
            self.speaker = _Speaker()

    _stub("pybricks.hubs", PrimeHub=_PrimeHub)
    _stub("pybricks.pupdevices", Motor=_Motor)
    _stub("pybricks.parameters", Axis=_Axis, Port=_Port)

    from fll_lib.runtime import detect_platform
    from fll_lib.utils import now_ms, sleep_ms, ticks_diff

    assert detect_platform() == "mock", detect_platform()

    import fll_lib.config as config
    import fll_lib.core.motors as motors
    import fll_lib.core.sensors as sensors
    from fll_lib.core.robot import create_robot

    config.PLATFORM = "pybricks"
    motors.PLATFORM = "pybricks"
    sensors.PLATFORM = "pybricks"

    assert not config.has_filesystem()

    w = motors.WheelDriver("A")
    assert isinstance(w._driver, motors._PybricksDriver)
    w.set_power(50)
    assert w._driver._motor._dc == 50
    w.reset_position(123)
    assert w.get_position() == 123
    w.velocity()
    w.status()
    w.stop(hold=True)
    assert w._driver._motor._braked

    ws = motors.WheelDriver("B", sign=-1)
    ws.set_power(25)
    assert ws._driver._motor._dc == -25
    ws.stop()

    gyro = sensors.GyroSensor()
    assert gyro.hub is not None
    assert gyro.yaw() == 0
    gyro.reset()
    assert gyro.yaw() == 0

    robot = create_robot()
    assert robot.heading() == 0
    robot.stop()

    assert config.load_config()["gyro_pid"]["kp"] == 2.0
    assert now_ms() >= 0
    assert ticks_diff(2, 1) == 1
    sleep_ms(1)

    robot2 = create_robot(left_manipulator_port="C", right_manipulator_port="D")
    robot2.move_manip_l(45, speed=999)
    assert robot2.manip_l.speed == 100.0

    print("STUB_OK")
    """
)


class TestPybricksDriverStub(unittest.TestCase):
    def test_pybricks_backend_runs_against_stubs(self):
        code = _STUB_SCRIPT
        env = dict(os.environ)
        env["PYTHONPATH"] = _REPO_ROOT + os.pathsep + env.get("PYTHONPATH", "")
        proc = subprocess.run(
            [sys.executable, "-c", code],
            cwd=_REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=120,
        )
        output = proc.stdout + proc.stderr
        self.assertEqual(proc.returncode, 0, "subprocess failed:\n" + output)
        self.assertIn("STUB_OK", proc.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
