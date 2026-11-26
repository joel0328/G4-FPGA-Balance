"""legged_controller.py - balance, height, roll and steering for the wheel-legged robot.

schedule() designs the balance LQR (controller.design_lqr) at several leg
lengths; LegController interpolates the gain and the balance pitch from that
table, so the C port only needs the table, not a Riccati solver.

Per 1 kHz tick LegController.step takes the wheel encoder counts, the four leg
joint angles and the IMU, plus a command (speed, turn rate, height), and returns
the two wheel duties and the four joint targets:
  balance   LQR on [x - x_ref, pitch - pitch_eq(L), x_dot - v_ref, pitch_rate, u_prev]
  height    both legs slew toward the commanded length
  roll      the leg-length difference levels the body (or leans it into a turn)
  steering  the wheel duty difference tracks the commanded yaw rate (gyro z)
Keep it free of anything that can't be ported to C.
"""
import math
from dataclasses import dataclass

import numpy as np

from controller import G, design_lqr
from legged_robot import LegBody, leg_fk, leg_ik, plant_at


def schedule(model, body: LegBody, motor, dt=1e-3, n=6, **lqr_kw):
    """Gain table over the leg-length range: lengths, K rows (5 each), balance pitch."""
    lengths = np.linspace(body.height_min, body.height_max, n)
    ks, eqs = [], []
    for L in lengths:
        p, eq = plant_at(model, body, L)
        K, _, _, _ = design_lqr(p, motor, dt, **lqr_kw)
        ks.append(K)
        eqs.append(eq)
    return lengths, np.array(ks), np.array(eqs)


@dataclass
class LegConfig:
    dt: float = 1e-3
    cpr: int = 2464
    duty_limit: float = 1.0          # a future board would set DUTY_LIMIT before arming
    tilt_tau: float = 0.5            # s, complementary filters (pitch and roll)
    vel_tau: float = 0.01            # s, encoder speed low-pass
    height_rate: float = 0.10        # m/s, leg length slew
    accel: float = 0.8               # m/s^2, speed command slew
    roll_kp: float = 0.1             # m of leg difference per rad of roll error
    roll_ki: float = 2.5             # m of leg difference per rad of roll error per s
    roll_on: bool = True             # level the body with the legs
    lean_in_turns: bool = True       # lean into turns like a cyclist
    yaw_kp: float = 0.15             # duty per rad/s of yaw-rate error
    yaw_ki: float = 0.6              # duty per rad of accumulated yaw error
    x_err_max: float = 0.3           # m, position error the balance loop may see
    fall_angle: float = 0.6          # rad, pitch or roll: stop driving


@dataclass
class Command:
    speed: float = 0.0               # m/s
    turn: float = 0.0                # rad/s, positive = left
    height: float = 0.15             # m, leg length


class LegController:
    def __init__(self, body: LegBody, table, cfg: LegConfig, gyro_bias=(0.0, 0.0, 0.0)):
        self.b, self.cfg = body, cfg
        self.lengths, self.ks, self.eqs = table
        self.gyro_bias = gyro_bias
        self.a_tilt = cfg.tilt_tau / (cfg.tilt_tau + cfg.dt)
        self.a_vel = cfg.dt / (cfg.vel_tau + cfg.dt)
        self.rad_per_count = 2 * math.pi / cfg.cpr
        self.pitch = self.roll = None
        self.prev = None
        self.wheel_vel = 0.0
        self.u_prev = 0.0
        self.x_ref = 0.0
        self.yaw_int = 0.0
        self.speed = 0.0                   # slewed speed command
        self.height = body.height_nominal
        self.dlen = 0.0                    # left minus right leg length
        self.roll_int = 0.0                # integral part of dlen
        self.fallen = False
        self.out = None

    def _gain(self, L):
        L = min(max(L, self.lengths[0]), self.lengths[-1])
        i = min(int(np.searchsorted(self.lengths, L)), len(self.lengths) - 1)
        i = max(i, 1)
        f = (L - self.lengths[i - 1]) / (self.lengths[i] - self.lengths[i - 1])
        return self.ks[i - 1] + f * (self.ks[i] - self.ks[i - 1]), self.eqs[i - 1] + f * (self.eqs[i] - self.eqs[i - 1])

    def step(self, enc_l, enc_r, joints, gyro, acc, cmd: Command):
        cfg, b = self.cfg, self.b
        gx, gy, gz = (g - bias for g, bias in zip(gyro, self.gyro_bias))
        ax, ay, az = acc
        if self.pitch is None:
            if abs(math.sqrt(ax * ax + ay * ay + az * az) - G) > 0.2 * G:
                return self._hold()
            self.pitch = math.atan2(-ax, math.hypot(ay, az))
            self.roll = math.atan2(ay, az)

        # attitude; take the turn's centripetal acceleration out of the roll reference
        ay_c = ay - self.wheel_vel * b.wheel_radius * gz
        self.pitch = self.a_tilt * (self.pitch + gy * cfg.dt) + (1 - self.a_tilt) * math.atan2(-ax, math.hypot(ay_c, az))
        self.roll = self.a_tilt * (self.roll + gx * cfg.dt) + (1 - self.a_tilt) * math.atan2(ay_c, az)

        # legs and wheels: wheel angle on the ground = encoder + shank angle + pitch
        hl, kl, hr, kr = joints
        L_l, L_r = -leg_fk(b, hl, kl)[1], -leg_fk(b, hr, kr)[1]
        L = (L_l + L_r) / 2
        wl = enc_l * self.rad_per_count + hl + kl
        wr = enc_r * self.rad_per_count + hr + kr
        if self.prev is None:
            self.prev = (wl, wr)
        v = ((wl - self.prev[0]) + (wr - self.prev[1])) / (2 * cfg.dt)
        self.prev = (wl, wr)
        self.wheel_vel += self.a_vel * (v - self.wheel_vel)

        if abs(self.pitch) > cfg.fall_angle or abs(self.roll) > cfg.fall_angle:
            self.fallen = True
        if self.fallen:
            return self._hold()

        # balance
        K, pitch_eq = self._gain(L)
        dv = cfg.accel * cfg.dt
        self.speed += max(-dv, min(dv, cmd.speed - self.speed))
        self.x_ref += self.speed * cfg.dt
        x = b.wheel_radius * ((wl + wr) / 2 + self.pitch)
        x_err = max(-cfg.x_err_max, min(cfg.x_err_max, x - self.x_ref))
        self.x_ref = x - x_err                       # don't wind up behind a slow robot
        x_dot = b.wheel_radius * (self.wheel_vel + gy)
        s = (x_err, self.pitch - pitch_eq, x_dot - self.speed, gy, self.u_prev)
        u = -sum(k * si for k, si in zip(K, s))
        u = max(-cfg.duty_limit, min(cfg.duty_limit, u))
        self.u_prev = u

        # steering: positive turn = left = right wheel faster
        err = cmd.turn - gz
        self.yaw_int = max(-0.5, min(0.5, self.yaw_int + err * cfg.dt))
        turn = cfg.yaw_kp * err + cfg.yaw_ki * self.yaw_int
        clamp = lambda d: max(-cfg.duty_limit, min(cfg.duty_limit, d))
        duty = (clamp(u - turn), clamp(u + turn))

        # height and roll
        target = min(max(cmd.height, b.height_min), b.height_max)
        step = cfg.height_rate * cfg.dt
        self.height += max(-step, min(step, target - self.height))
        roll_ref = 0.0
        if cfg.lean_in_turns:
            roll_ref = -math.atan(self.speed * cmd.turn / G)  # positive roll tips the top right
        if cfg.roll_on:
            room = min(self.height - b.height_min, b.height_max - self.height) * 2
            e = self.roll - roll_ref
            self.roll_int = max(-room, min(room, self.roll_int - cfg.roll_ki * e * cfg.dt))
            self.dlen = max(-room, min(room, self.roll_int - cfg.roll_kp * e))
        else:
            self.dlen = 0.0
        ql = leg_ik(b, self.height + self.dlen / 2)
        qr = leg_ik(b, self.height - self.dlen / 2)
        self.out = (duty, (ql[0], ql[1], qr[0], qr[1]))
        return self.out

    def _hold(self):
        self.u_prev = 0.0
        ql = leg_ik(self.b, self.height)
        return (0.0, 0.0), (ql[0], ql[1], ql[0], ql[1])
