# fll_lib

MicroPython robot-control library for LEGO SPIKE Prime running [Pybricks](https://code.pybricks.com), with a desktop mock backend. The same code runs on the hub and on the PC.

| Platform | Backend |
| --- | --- |
| SPIKE Prime hub | Pybricks firmware (`pybricks.hubs` / `pybricks.pupdevices`) |
| Desktop | Mock simulation (`fll_lib.mocks`), no hardware |

Only MicroPython-safe features are used (methods, not `@property`). The hub has no filesystem, so `config.json` and file logging are skipped there — edit `fll_lib/config.py`; it is imported on every launch.

## Quick start

Hub (requires [Pybricks firmware](https://code.pybricks.com) + `pip install pybricksdev`):

```bash
pybricksdev run ble --name "klbot" main_pybricks.py
```

Keep the robot still ~1 s at startup so the IMU calibrates.

PC, no hardware:

```bash
python examples/simulate.py        # demo on mock motors
python -m unittest discover tests  # unit tests
```

## Wiring

```python
from fll_lib.core.robot import create_robot

robot = create_robot()  # ports, directions and limits come from fll_lib/config.py
```

| Argument | Meaning |
| --- | --- |
| `left_wheel_sign` / `right_wheel_sign` | `-1` for a mirrored motor (axle points the other way). |
| `turn_sign` | Fixed rotation sense (1 or -1). If `None`, measured by one short probe spin at startup. |
| `left_manipulator_limits` / `right_manipulator_limits` | Hard arm limits in degrees. |
| `pybricks_top_side` / `pybricks_front_side` | IMU axes for `PrimeHub(...)` (default `"Z"` / `"X"`), depends on hub mounting. |

## Robot API

Fields: `left_wheel`, `right_wheel` (`WheelDriver`), `manip_l`, `manip_r` (`Manipulator`), `gyro` (`GyroSensor`), `chassis` (`DifferentialDrive`), `pose` (`PoseTracker`), `config` (`dict`).

| Method | Purpose |
| --- | --- |
| `forward(cm, speed)` / `backward(cm, speed)` / `stop()` | Straight drive (gyro-corrected, distance from wheel angles). |
| `turn_left(deg, speed)` / `turn_right(deg, speed)` | In-place turns, return `True` if within tolerance. |
| `heading()` / `reset_gyro()` | Yaw in `[-180, 180]`, zero it. |
| `pose_x()`, `pose_y()`, `pose_position()`, `pose_heading()`, `pose_drift()`, `pose_distance()` | Dead-reckoning pose since `reset_pose()`. |
| `reset_pose(x, y, heading=None)` | Re-anchor pose (default: current gyro heading). |
| `goto(x, y, final_heading=None)` | Turn to bearing, drive, optionally final turn. Most accurate from an accurate `reset_pose()`. |
| `turn_to_heading(deg)` | Shortest-path turn to a heading. |
| `move_manip_l(deg, speed)` / `move_manip_r(deg, speed)` / `move_both_manipulators(l, r, speed)` | Move arms; return `True` if reached. |
| `manip_l_angle()` / `manip_r_angle()` | Current global (physical) arm angles. |
| `reset_manip_zero()` | "Both arms at physical 0 now" — calibrate the global angle origin. |
| `calibrate_manip_angles(l, r)` | "Left arm at l, right arm at r now". |
| `turn_error_report()` / `reset_turn_comp()` | Read/reset learned per-direction turn compensation. |
| `release_manipulators()` / `coast_manipulators()` | Release arms (brake) / fully free (coast). |

## Movement

```python
robot.forward(30)
robot.forward(50, 80)
robot.backward(25, 60)
robot.turn_right(90, 40)
robot.turn_left(180, 30)
robot.stop()
```

- Distance is odometry-based, so accuracy doesn't depend on speed.
- Straight drive holds the heading with a PID (`config["gyro_pid"]`); soft launch (`drive_launch_ms`, `drive_launch_scale`), soft finish ramp (`drive_end_ramp_deg`) and anti-backlash preload (`drive_luff_power`, below the ~28% wheel breakaway) improve parking repeatability.
- Drift is compensated via `drift_forward_factor` / `drift_backward_factor` (`DriftCompensator`).
- Turns are phase-based (fast → slow creep → latch → measured taps) with a time budget; systematic per-direction error is learned as an EMA and pre-applied (see `turn_error_compensation`).

### Pose tracking

```python
robot.reset_pose(0, 0)
robot.forward(60, 80)
robot.turn_left(90)
robot.forward(35, 80)
robot.pose_position()      # (~60, ~35)
robot.goto(90, 60)
robot.turn_to_heading(0)
```

Accuracy is limited by odometry drift — call `reset_pose()` whenever the robot passes a known reference (field border, line, door).

## Manipulators

Position control is delegated to the firmware (`Motor.run_target`); the library only sets the angle, waits, and checks the result. Limits are enforced hard: targets are clamped to `[min + safe_margin, max - safe_margin]`, motion aborts if the arm leaves `[min, max]`, and it won't start at all if the arm is already outside the limits — call `reset_manip_zero()` once at startup with the arms in a safe pose.

SPIKE motors have an absolute encoder, so angles are global: `move_manip_l(90)` means the same physical pose on every boot. Optionally declare the encoder offset in config (`left_manipulator_offset` / `right_manipulator_offset`).

```python
robot.reset_manip_zero()          # arms at physical 0 right now
ok = robot.move_manip_l(90)
robot.move_both_manipulators(100, -100, 50)
deg = robot.manip_l_angle()       # global degrees (same frame as move_*)
```

Arms are held after a move. Before any `forward/backward/turn_*` they are auto-released to brake (`release_manips_on_drive`, default `True`) to avoid convulsive ringing; set the attribute to `False` if an arm must stay rigid during a move.

Key options: `manipulator_speed` (50), `manipulator_tolerance` (1.0), `manipulator_safe_margin` (10), `manipulator_hold` (True), `manipulator_debug` (False), `left/right_manipulator_limits` ([-120, 120]).

## Gyro

```python
heading = robot.heading()       # yaw, [-180, 180]
robot.reset_gyro()
pitch, roll = robot.gyro.pitch(), robot.gyro.roll()
robot.gyro.hub                  # PrimeHub object (None on desktop)
```

## Wheel encoders / odometry

```python
deg = robot.left_wheel.get_position()
dist_cm = deg / 360.0 * robot.config["wheel_circumference_cm"]
```

`WheelDriver` methods: `get_position()`, `reset_position()`, `velocity()`, `set_power()`, `stop()`, `status()`, `advance()` (mock only). Geometry defaults: `wheel_diameter_cm = 5.6`, `track_width_cm = 14.5`.

## PathRunner — command sequences

```python
from fll_lib.missions.path_runner import PathRunner

plan = PathRunner(robot)
(plan.forward(40, 80).turn_right(90, 40).manip_l(90)
     .both_manips(100, -100, 60).backward(40, 80)
     .pause(500).reset_gyro())
plan.run()
```

Commands: `forward`, `backward`, `turn_right`, `turn_left`, `manip_l`, `manip_r`, `both_manips`, `pause`, `reset_gyro`.

## Mission planning / scoring

```python
from fll_lib.missions import MissionPlan, ScoringSystem

plan = MissionPlan(available_time_s=150)
plan.add("deliver", 30, 10)                       # name, points, duration_s
plan.add("collect", 20, 10, requires=["deliver"])
selected, total_points = plan.best_plan()

scores = ScoringSystem()
scores.add("deliver", 30, score_fn=lambda s: s["delivered"])
scores.mark("bonus", True)
scores.evaluate(state)
scores.total_score()
```

`MissionPlan.best_plan()` picks runs by points-per-second budget. `ScoringSystem.evaluate()` calls each unmarked `score_fn`.

## Configuration

Edit **`fll_lib/config.py`** to configure all launch scripts in one place:

- `DEFAULTS`: PID (`gyro_pid`: `kp`, `ki`, `kd`, `integral_limit`, `deadzone`, `deriv_filter`), ports, geometry, speeds, manipulator limits, drive ramps and turn settings.
- `PYBRICKS_OVERRIDES`: hardware-specific motor/turn directions. The desktop mock uses the directions in `DEFAULTS`.

All bundled launch scripts use `create_robot()` and automatically import these settings, including on the hub without a filesystem. Shared arm limits are now ±120° with a 10° safety margin (previous scripts used different limits). Speeds passed to individual movement commands remain per-command overrides.

```python
from fll_lib.core.robot import create_robot

robot = create_robot()
robot.forward(30)  # uses drive_speed from the shared config
```

Optional overrides remain supported: explicit `create_robot(...)` arguments override a supplied config dict; a supplied dict merges with the shared settings. Without a supplied dict, desktop launches also load legacy `config.json` from the current directory. On the hub JSON loading is skipped. For one configuration source, edit the Python config and omit JSON/launch overrides.

Key drive options: `max_speed` (100), `drive_speed` (75), `drive_stop_mode` ("hold"), `drive_launch_scale` (0.6), `drive_launch_ms` (300), `drive_end_ramp_deg` (40), `drive_end_min_power` (18), `drive_luff_enable/power/ms` (true/22/30), `drift_forward_factor`, `drift_backward_factor`.

Turn options (`turn_control`): `turn_speed` (60), `settle_tolerance` (1.5), `settle_wheel_speed` (45), `settle_reads/ms` (3/30), `fallback_tolerance` (4), `stop_mode` ("hold"), `turndown_deg` (25), `fast_ramp` (28), `creep_duty` (30, must exceed the ~28% breakaway), `creep_ramp` (8), `stop_deg` (9), `latch_wait_ms` (60), `latch_max_ms` (350), `tap_duty` (34), `tap_wheel_deg` (8), `tap_min_deg` (1.5), `tap_ratio` (1.6), `tap_max_ms` (600), `tap_max` (3), `tap_revert_deg` (10), `budget_base_ms` (650), `budget_per_deg` (9), `budget_min_ms` (800), `budget_max_ms` (2600).

Turn error compensation (`turn_error_compensation`, disabled by default): `alpha` (0.1), `min_sample_deg` (1.5), `max_correction_deg` (6), `max_sample_deg` (8). Enable only if `turn_error_report()` shows a stable one-sided bias ≥2°.

## Tests

```bash
python -m unittest discover tests -v
python manipulators_test.py        # arm angles (hub or mock)
python mission_test.py             # rectangle + out-and-back, reports |drift|
```

CI runs these on Python 3.8–3.12 for every push/PR (`.github/workflows/ci.yml`).

Tuning notes: drift on launch → lower `drive_launch_scale`/higher `drive_launch_ms`; straight-line sway → cool down `gyro_pid`; turn stalling short → raise `creep_duty`/`budget_max_ms`; turn overshoot → lower `turndown_deg`/`stop_deg`/`tap_wheel_deg`. Motors need ~28% duty to break from standstill and hold ~35–40°/s minimum, so final turn correction uses short measured taps, not slow continuous driving.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| Robot spins at startup | Auto turn calibration runs because `turn_sign` is unset. Set it explicitly. |
| Sways while driving | `gyro_pid` too aggressive. |
| Turn rings / stalls short | `creep_duty` below breakaway (~28%) or `budget_max_ms` too low. |
| Arms convulse while driving | Left in `hold`; keep `release_manips_on_drive=True` or call `release_manipulators()`. |
| Arm won't move (`move_*` → `False`) | Encoder/physics mismatch; place arms safely and call `reset_manip_zero()`. |
| `ImportError: math.hypot` on hub | Use `(x*x + y*y) ** 0.5` — MicroPython lacks `math.hypot`. |
| Arm reaches short of target | `manipulator_safe_margin` clips the request. |
| Arm won't move, config ignored | Edit ports/limits in `fll_lib/config.py`, then upload the program again. |
| Turn unsynced with gyro | Check `pybricks_top_side` / `pybricks_front_side` for the hub mount. |

## License

MIT — see [LICENSE](LICENSE).