"""controller.py - LQR balance controller and the estimator in front of it.

design_lqr() builds the linear planar model from robot.plant() and the motor's
linear gains, discretises it at the 1 kHz loop rate with the one-tick delay
between the snapshot and the duty write, and returns the gain.

Controller.step() is written the way it will run on the STM32: once per tick it
takes what the FPGA snapshot provides (integer encoder counts, raw gyro and
accelerometer readings) and returns the two duties. Keep it free of anything
that can't be ported to C.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm, solve_discrete_are

G = 9.81


def linear_model(p, motor):
    """Continuous model, state s = [x, pitch, x_dot, pitch_dot], input = duty on both wheels.
    x is the axle position, so x = r * (wheel angle relative to the body + pitch)."""
    M, l, I_b, m_w, I_w, J_m, r = (p[k] for k in ("M", "l", "I_b", "m_w", "I_w", "J_m", "r"))
    b, c = motor.linear_gains()
    mass = np.array([[2 * m_w + 2 * I_w / r**2 + M + 2 * J_m / r**2, M * l - 2 * J_m / r],
                     [M * l - 2 * J_m / r, M * l**2 + I_b + 2 * J_m]])
    # total motor torque tau = 2 b u - 2 c (x_dot / r - pitch_dot) acts +tau/r on x, -tau on pitch
    tau_s = np.array([0, 0, -2 * c / r, 2 * c])
    forces_s = np.vstack([tau_s / r, np.array([0, M * G * l, 0, 0]) - tau_s])
    forces_u = np.array([2 * b / r, -2 * b])
    minv = np.linalg.inv(mass)
    A = np.zeros((4, 4))
    A[0, 2] = A[1, 3] = 1
    A[2:] = minv @ forces_s
    B = np.zeros((4, 1))
    B[2:, 0] = minv @ forces_u
    return A, B


def design_lqr(p, motor, dt=1e-3, q=(1 / 0.05**2, 1 / 0.02**2, 1 / 0.3**2, 1 / 0.5**2), r_u=1 / 0.2**2):
    """Discrete LQR with the previous duty as a fifth state (one tick of delay).
    q weights x (m), pitch (rad), x_dot (m/s), pitch_dot (rad/s); r_u weights duty.
    Returns K (1 x 5) for u = -K [x, pitch, x_dot, pitch_dot, u_prev]."""
    A, B = linear_model(p, motor)
    n = A.shape[0]
    big = np.zeros((n + 1, n + 1))
    big[:n, :n], big[:n, n:] = A, B
    e = expm(big * dt)
    Ad, Bd = e[:n, :n], e[:n, n:]
    Az = np.block([[Ad, Bd], [np.zeros((1, n)), np.zeros((1, 1))]])
    Bz = np.vstack([np.zeros((n, 1)), np.ones((1, 1))])
    Q = np.diag(list(q) + [0.0])
    R = np.array([[r_u]])
    P = solve_discrete_are(Az, Bz, Q, R)
    K = np.linalg.solve(R + Bz.T @ P @ Bz, Bz.T @ P @ Az)
    poles = np.linalg.eigvals(Az - Bz @ K)
    return K.ravel(), A, B, poles


@dataclass
class Config:
    dt: float = 1e-3              # loop tick
    cpr: int = 2464               # encoder counts per output revolution (MEASURE)
    duty_limit: float = 0.5       # FPGA DUTY_LIMIT after reset
    tilt_tau: float = 0.5         # s, complementary filter: gyro below, accelerometer above
    vel_tau: float = 0.01         # s, low-pass on the encoder speed
    yaw_kp: float = 0.5           # duty per rad of wheel-angle difference
    yaw_kd: float = 0.02          # duty per rad/s
    fall_angle: float = 0.6       # rad: beyond this, stop driving (the firmware would disarm)


class Controller:
    def __init__(self, K, wheel_radius, cfg: Config, gyro_bias=0.0):
        self.K = [float(k) for k in K]
        self.r = wheel_radius
        self.cfg = cfg
        self.gyro_bias = gyro_bias            # from a still calibration at start-up
        self.a_tilt = cfg.tilt_tau / (cfg.tilt_tau + cfg.dt)
        self.a_vel = cfg.dt / (cfg.vel_tau + cfg.dt)
        self.rad_per_count = 2 * math.pi / cfg.cpr
        self.pitch = None
        self.prev = None
        self.wheel_vel = 0.0
        self.diff_vel = 0.0
        self.u_prev = 0.0
        self.x_ref = 0.0
        self.fallen = False

    def step(self, enc_l, enc_r, gyro_y, acc_x, acc_z):
        cfg = self.cfg
        # pitch: integrate the gyro, pull slowly toward the accelerometer's gravity direction
        rate = gyro_y - self.gyro_bias
        acc_pitch = math.atan2(-acc_x, acc_z)
        if self.pitch is None:
            if abs(math.hypot(acc_x, acc_z) - G) > 0.2 * G:
                return 0.0, 0.0                 # not at rest yet: no trustworthy tilt
            self.pitch = acc_pitch
        self.pitch = self.a_tilt * (self.pitch + rate * cfg.dt) + (1 - self.a_tilt) * acc_pitch
        # wheels: angle relative to the body from the counts, speed from the per-tick delta
        al, ar = enc_l * self.rad_per_count, enc_r * self.rad_per_count
        if self.prev is None:
            self.prev = (al, ar)
        dl, dr = (al - self.prev[0]) / cfg.dt, (ar - self.prev[1]) / cfg.dt
        self.prev = (al, ar)
        self.wheel_vel += self.a_vel * ((dl + dr) / 2 - self.wheel_vel)
        self.diff_vel += self.a_vel * ((dl - dr) - self.diff_vel)

        x = self.r * ((al + ar) / 2 + self.pitch)
        x_dot = self.r * (self.wheel_vel + rate)
        if abs(self.pitch) > cfg.fall_angle:
            self.fallen = True
        if self.fallen:
            self.u_prev = 0.0
            return 0.0, 0.0
        s = (x - self.x_ref, self.pitch, x_dot, rate, self.u_prev)
        u = -sum(k * v for k, v in zip(self.K, s))
        u = max(-cfg.duty_limit, min(cfg.duty_limit, u))
        self.u_prev = u
        turn = -(cfg.yaw_kp * (al - ar) + cfg.yaw_kd * self.diff_vel)
        clamp = lambda v: max(-cfg.duty_limit, min(cfg.duty_limit, v))
        return clamp(u + turn), clamp(u - turn)
