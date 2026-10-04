from fll_lib.utils import normalize_angle, clamp, sleep_ms, now_ms, ticks_diff
from fll_lib.control.pid import GyroPID
from fll_lib.config import merge_config
from fll_lib.utils.logging import Logger
from fll_lib.core.pose import PoseTracker


class DifferentialDrive:
    def __init__(self, left, right, gyro, drift, config, pose=None):
        config = merge_config(config)
        self.left = left
        self.right = right
        self.gyro = gyro
        self.drift = drift
        self.wheel_circumference_cm = config["wheel_circumference_cm"]
        self.max_speed = clamp(config["max_speed"], 1, 100)
        self.pose = pose or PoseTracker(self.wheel_circumference_cm)
        self._launch_scale = max(0.0, config["drive_launch_scale"])
        self._launch_ms = max(0, config["drive_launch_ms"])
        self._end_ramp_deg = max(5.0, float(config["drive_end_ramp_deg"]))
        self._end_min_power = clamp(float(config["drive_end_min_power"]),
                                    0.0, 100.0) / 100.0
        self._luff_enable = bool(config["drive_luff_enable"])
        self._luff_power = clamp(float(config["drive_luff_power"]), 0.0, 100.0)
        self._luff_ms = max(0, int(config["drive_luff_ms"]))
        self.drive_stop_mode = config["drive_stop_mode"]
        self.turn_debug = bool(config["turn_debug"])
        self._dbg = Logger(console=True) if self.turn_debug else None
        self._dbg_state = None
        self._dbg_t = -1

        g_pid = dict(config["gyro_pid"])
        g_pid["output_limits"] = (-self.max_speed, self.max_speed)
        self.heading_pid = GyroPID(get_heading=self.gyro.yaw, **g_pid)
        self.turn_sign = 1
        self._turn_calibrated = False
        ts0 = config.get("turn_sign")
        if ts0 is not None:
            self.turn_sign = 1 if ts0 >= 0 else -1
            self._turn_calibrated = True
        self.turn_cfg = dict(config["turn_control"])

        comp = config["turn_error_compensation"]
        self.turn_comp_enabled = bool(comp["enabled"])
        self._comp_alpha = float(comp["alpha"])
        self._comp_min_sample = float(comp["min_sample_deg"])
        self._comp_max_corr = float(comp["max_correction_deg"])
        self._comp_max_sample = float(comp["max_sample_deg"])

        self._turn_comp = {1: 0.0, -1: 0.0}
        self._tl = 0.0
        self._tr = 0.0

    @staticmethod
    def normalize_angle_delta(angle):

        return normalize_angle(angle)

    def forward(self, distance_cm, speed=60):
        return self._drive(distance_cm, speed, direction=1)

    def backward(self, distance_cm, speed=60):
        return self._drive(distance_cm, speed, direction=-1)

    def _drive(self, distance_cm, speed, direction=1):
        dir_name = "forward" if direction == 1 else "backward"
        compensated = self.drift.compensate(distance_cm, dir_name)
        target_deg = (compensated / self.wheel_circumference_cm) * 360.0
        base_speed = clamp(abs(speed), 1, self.max_speed) * direction

        if not self._turn_calibrated:
            self.calibrate_turn_sign()
        ts = 1 if self.turn_sign >= 0 else -1

        self.left.reset_position(0)
        self.right.reset_position(0)
        self.heading_pid.reset()
        self.heading_pid.set_target(self.gyro.yaw())

        self.left.set_power(base_speed * self._launch_scale)
        self.right.set_power(base_speed * self._launch_scale)

        start_time = now_ms()
        timeout_ms = int(abs(target_deg) * 12 + 4000)
        reached = False
        dbg_last = -10000
        if self._dbg is not None:
            self._dbg.log("drive %s %.0fcm @%d hdg0=%.1f ts=%+d"
                          % (dir_name, distance_cm, base_speed,
                             self.gyro.yaw(), ts))

        last_avg = None
        while True:
            avg = (self.left.get_position() + self.right.get_position()) * 0.5
            if last_avg is None:
                last_avg = avg
            self.pose.integrate(avg - last_avg, self.gyro.yaw())
            last_avg = avg
            if abs(avg) >= abs(target_deg):
                reached = True
                break
            correction = clamp(self.heading_pid.update(), -self.max_speed, self.max_speed)
            t_run = ticks_diff(now_ms(), start_time)
            launch_f = (self._launch_scale if t_run < self._launch_ms else 1.0)

            left_deg = abs(target_deg) - abs(avg)
            if left_deg < self._end_ramp_deg:
                end_f = (self._end_min_power
                         + (1.0 - self._end_min_power)
                         * max(0.0, left_deg) / self._end_ramp_deg)
            else:
                end_f = 1.0
            eff_base = base_speed * launch_f * end_f
            lspd = clamp(eff_base + ts * correction, -self.max_speed, self.max_speed)
            rspd = clamp(eff_base - ts * correction, -self.max_speed, self.max_speed)
            self.left.set_power(lspd)
            self.right.set_power(rspd)
            if self._dbg is not None and ticks_diff(now_ms(), dbg_last) >= 500:
                dbg_last = now_ms()
                self._dbg.log("    t=%dms hdg=%+.1f err=%+.1f corr=%+.0f "
                              "L=%+.0f R=%+.0f pos=%+.0f"
                              % (ticks_diff(now_ms(), start_time),
                                 self.gyro.yaw(),
                                 self.heading_pid.error(),
                                 correction, lspd, rspd, avg))
            sleep_ms(20)
            if ticks_diff(now_ms(), start_time) > timeout_ms:
                break

        if reached and self._luff_enable and self._luff_ms > 0:
            lp = direction * self._luff_power
            self.left.set_power(lp)
            self.right.set_power(lp)
            sleep_ms(self._luff_ms)

        self.left.stop(hold=(self.drive_stop_mode == "hold"))
        self.right.stop(hold=(self.drive_stop_mode == "hold"))
        avg = (self.left.get_position() + self.right.get_position()) * 0.5
        self.pose.integrate(avg - last_avg, self.gyro.yaw())
        return reached

    def calibrate_turn_sign(self, probe_power=35, probe_ms=250):
        yaw0 = self.gyro.yaw()
        self.left.set_power(probe_power)
        self.right.set_power(-probe_power)
        sleep_ms(probe_ms)
        self.left.stop(hold=True)
        self.right.stop(hold=True)
        sleep_ms(100)
        delta = normalize_angle(self.gyro.yaw() - yaw0)
        self.turn_sign = 1 if delta >= 0 else -1
        self._turn_calibrated = True
        return self.turn_sign, delta

    def _stop_and_sync(self, hold=False):
        self._tl = 0.0
        self._tr = 0.0
        self.left.stop(hold=hold)
        self.right.stop(hold=hold)

    def _wait_wheels_stopped(self, tc, timeout_ms=None):
        if timeout_ms is None:
            timeout_ms = int(tc.get("latch_max_ms", 350))
        waited = 0
        while waited < timeout_ms:
            if max(abs(self.left.velocity()),
                   abs(self.right.velocity())) <= tc["settle_wheel_speed"]:
                break
            sleep_ms(20)
            waited += 20

    def _set_power_ramp(self, left_power, right_power, step):
        left_power = clamp(left_power, self._tl - step, self._tl + step)
        right_power = clamp(right_power, self._tr - step, self._tr + step)
        self._tl = left_power
        self._tr = right_power
        self.left.set_power(left_power)
        self.right.set_power(right_power)

    def turn(self, angle_deg, speed=40):
        if not self._turn_calibrated:
            ts0, delta = self.calibrate_turn_sign()
            if self._dbg is not None:
                self._dbg.log("turn auto-calibrated turn sign ts=%+d (delta "
                              "%+.1f deg)" % (ts0, delta))
        tc = self.turn_cfg

        comp = 0.0
        if self.turn_comp_enabled:
            comp = clamp(self._turn_comp[1 if angle_deg >= 0 else -1],
                         -self._comp_max_corr, self._comp_max_corr)
        requested = angle_deg + comp
        target = normalize_angle(self.gyro.yaw() + requested)
        cap = clamp(abs(speed), 10, 100)
        ts = 1 if self.turn_sign >= 0 else -1

        direction = 1.0 if requested >= 0.0 else -1.0
        tol = tc["settle_tolerance"]
        hold_stop = (tc["stop_mode"] == "hold")
        self._dbg_state = None
        self._dbg_t = -1
        if self._dbg is not None:
            self._dbg.log("turn angle=%+d speed=%d target=%+.1f cap=%d ts=%+d "
                          "dir=%+.0f tol=%.1f comp=%+.2f stop_mode=%s"
                          % (angle_deg, speed, target, cap, ts, direction,
                             tol, comp, tc["stop_mode"]))

        start_time = now_ms()
        budget_ms = int(max(tc["budget_min_ms"],
                            min(tc["budget_max_ms"],
                                tc["budget_base_ms"]
                                + abs(requested) * tc["budget_per_deg"])))

        def rem_now():
            return normalize_angle(target - self.gyro.yaw())

        self._stop_and_sync(hold=hold_stop)
        reached = False
        settled = 0
        taps_left = tc["tap_max"]
        state = "fast"

        while True:
            t = ticks_diff(now_ms(), start_time)
            yaw = self.gyro.yaw()
            rem = rem_now()
            wheel_spd = max(abs(self.left.velocity()),
                            abs(self.right.velocity()))

            if t > budget_ms:
                state = "budget"
                break

            if abs(rem) <= tol and wheel_spd <= tc["settle_wheel_speed"]:
                self._stop_and_sync(hold=hold_stop)
                settled += 1
                self._dbg_sample(t, state, yaw, target, rem, 0.0,
                                 wheel_spd, settled, "+settle")
                if settled >= tc["settle_reads"]:
                    state = "done"
                    reached = True
                    break
                sleep_ms(tc["settle_ms"])
                continue
            settled = 0

            if state == "fast":

                if abs(rem) > tc["turndown_deg"]:
                    cmd = direction * cap * ts
                    self._set_power_ramp(cmd, -cmd, tc["fast_ramp"])
                    self._dbg_sample(t, state, yaw, target, rem, 0.0,
                                     wheel_spd, settled)
                    sleep_ms(20)
                    continue

                state = "slow"
                sleep_ms(20)
                continue

            if state == "slow":

                if abs(rem) > tc["stop_deg"]:
                    sd = -1.0 if rem < 0.0 else 1.0
                    cmd = sd * tc["creep_duty"] * ts
                    self._set_power_ramp(cmd, -cmd, tc["creep_ramp"])
                    self._dbg_sample(t, state, yaw, target, rem, 0.0,
                                     wheel_spd, settled)
                    sleep_ms(20)
                    continue

                self._stop_and_sync(hold=hold_stop)
                if tc["latch_wait_ms"] > 0:
                    sleep_ms(tc["latch_wait_ms"])
                self._wait_wheels_stopped(tc)
                self._dbg_state = None
                self._dbg_t = -1
                state = "probe"
                continue

            if abs(rem) <= tol:
                state = "slow"
                continue
            if abs(rem) > tc["tap_revert_deg"]:
                state = "slow"
                continue
            if taps_left <= 0:
                if abs(rem) > tc["stop_deg"]:

                    state = "slow"
                else:

                    state = "budget"
                break

            sd = -1.0 if rem < 0.0 else 1.0
            cmd = sd * tc["tap_duty"] * ts
            target_tap_deg = min(tc["tap_wheel_deg"],
                                 max(tc["tap_min_deg"],
                                     abs(rem) * tc["tap_ratio"]))
            left0 = self.left.get_position()
            right0 = self.right.get_position()
            self.left.set_power(cmd)
            self.right.set_power(-cmd)
            self._dbg_sample(t, state, yaw, target, rem, 0.0,
                             wheel_spd, settled)
            tap_st = now_ms()
            while True:
                moved = (abs(self.left.get_position() - left0)
                         + abs(self.right.get_position() - right0)) * 0.5
                if moved >= target_tap_deg:
                    break
                if ticks_diff(now_ms(), tap_st) > tc["tap_max_ms"]:
                    break
                sleep_ms(10)
            self._stop_and_sync(hold=hold_stop)
            if tc["latch_wait_ms"] > 0:
                sleep_ms(tc["latch_wait_ms"])
            self._wait_wheels_stopped(tc)
            self._dbg_state = None
            self._dbg_t = -1
            taps_left -= 1
            continue

        self._stop_and_sync(hold=hold_stop)
        if not reached and abs(rem) <= tc["fallback_tolerance"]:

            reached = True
        final_yaw = self.gyro.yaw()
        residual = normalize_angle(target - final_yaw)
        self.pose.set_heading(final_yaw)
        if self.turn_comp_enabled and reached and self._comp_min_sample <= abs(residual) <= self._comp_max_sample:

            key = 1 if requested >= 0 else -1
            old = self._turn_comp[key]
            self._turn_comp[key] = clamp(
                (1.0 - self._comp_alpha) * old + self._comp_alpha * residual,
                -self._comp_max_corr, self._comp_max_corr)
            if self._dbg is not None:
                self._dbg.log("turn comp[%+d] %.3f -> %.3f (res %.2f)"
                              % (key, old, self._turn_comp[key], residual))
        if not reached and abs(residual) <= tol:
            reached = True
        if self._dbg is not None:
            self._dbg.log("turn end reached=%s err=%+.1f t=%dms comp=%+.2f"
                          % (reached, residual, t, comp))
        return reached

    def turn_error_report(self):
        return {"ccw": self._turn_comp[1], "cw": self._turn_comp[-1],
                "enabled": self.turn_comp_enabled}

    def reset_turn_comp(self):
        self._turn_comp = {1: 0.0, -1: 0.0}

    def _dbg_sample(self, t, state, yaw, target, remaining, filt_v,
                    wheel_spd, settled, note=""):
        if self._dbg is None:
            return
        if state != self._dbg_state:
            self._dbg_state = state
            force = True
        elif state == "slow":
            force = True
        else:
            force = (t - self._dbg_t) >= 250
        if not force:
            return
        self._dbg_t = t
        self._dbg.log("%4dms st=%s rem=%+.1f hdg=%+.1f tgt=%+.1f v=%+.0f "
                      "ws=%d L=%+.0f R=%+.0f s=%d%s"
                      % (t, state, remaining, yaw, target, filt_v,
                         wheel_spd, self._tl, self._tr, settled, note))

    def stop(self):
        self.left.stop()
        self.right.stop()
