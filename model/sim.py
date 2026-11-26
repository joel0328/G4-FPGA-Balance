"""sim.py - run the balancer in MuJoCo with the real loop timing, sensors and motor drive.

    python sim.py                      start tilted 3 deg, push forward at 1 s, back at 5 s
    python sim.py --view               same, in the 3D viewer (space = push, B = push back)
    python sim.py --push 1.5 --set battery_height=0.16 --duty-limit 1.0
    python sim.py --mode linear        ideal motor drive instead of the FPGA's coast PWM

Each 1 ms tick: sample the sensors the way the FPGA snapshot will (encoder counts
quantised to CPR, gyro/accelerometer with noise and bias), run Controller.step,
apply its duties one tick later. Physics runs at 0.5 ms. Ends with a RESULT line
and writes out/run.png.
"""
import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from controller import Config, Controller, design_lqr
from motor import Gearbox, Motor
from robot import Body, build_xml, plant

OUT = Path(__file__).resolve().parent / "out"


@dataclass
class Sensors:
    gyro_noise: float = 0.002        # rad/s rms per sample (~0.1 deg/s)
    gyro_bias: float = 0.01          # rad/s true bias (~0.6 deg/s) ...
    gyro_bias_cal: float = 0.009     # ... and what the start-up calibration found
    acc_noise: float = 0.05          # m/s^2 rms per sample


@dataclass
class Scenario:
    seconds: float = 10.0
    tilt_deg: float = 3.0
    pushes: list = field(default_factory=lambda: [(1.0, 1.0), (5.0, -1.0)])   # (time s, force N)
    push_len: float = 0.1            # s
    delay_ticks: int = 1             # snapshot -> duty write
    seed: int = 1


def make(body, motor):
    model = mujoco.MjModel.from_xml_string(build_xml(body, motor))
    return model, mujoco.MjData(model)


def settle(model, data, deg):
    """Settle the wheels onto the floor upright, then tilt by deg and let go."""
    set_tilt(model, data, 0.0)
    for _ in range(200):
        mujoco.mj_step(model, data)
    data.qvel[:] = 0
    set_tilt(model, data, deg)


def set_tilt(model, data, deg):
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([0.0, 1.0, 0.0]), math.radians(deg))
    data.qpos[3:7] = q
    mujoco.mj_forward(model, data)


def pitch_of(data, bid):
    R = data.xmat[bid].reshape(3, 3)
    return math.atan2(R[0, 2], R[2, 2])


def run(body=None, motor=None, cfg=None, sc=None, sens=None, view=False, quiet=False, plot=True,
        lqr_kw=None, design=None):
    """design = (Body, Motor) the controller is designed for; default: the simulated ones.
    montecarlo.py passes the nominal robot here while simulating a randomized one."""
    body, motor = body or Body(), motor or Motor()
    cfg, sc, sens = cfg or Config(), sc or Scenario(), sens or Sensors()
    model, data = make(body, motor)
    d_body, d_motor = design or (body, motor)
    p = plant(make(d_body, d_motor)[0], d_motor)
    K, _, _, poles = design_lqr(p, d_motor, cfg.dt, **(lqr_kw or {}))
    ctrl = Controller(K, p["r"], cfg, gyro_bias=sens.gyro_bias_cal)
    rng = np.random.default_rng(sc.seed)
    settle(model, data, sc.tilt_deg)

    ch = model.body("chassis").id
    adr = {n: model.sensor_adr[model.sensor(n).id] for n in ("gyro", "accel", "enc_l", "enc_r")}
    dof_l, dof_r = model.joint("wheel_l").dofadr[0], model.joint("wheel_r").dofadr[0]
    qa_l, qa_r = model.joint("wheel_l").qposadr[0], model.joint("wheel_r").qposadr[0]
    gears = None
    if motor.backlash_deg > 0:
        gears = (Gearbox(motor, data.qpos[qa_l]), Gearbox(motor, data.qpos[qa_r]))
    sub = round(cfg.dt / model.opt.timestep)
    ticks = round(sc.seconds / cfg.dt)
    queue = [(0.0, 0.0)] * sc.delay_ticks
    keys = []                                   # pushes requested from the viewer

    if not quiet:
        print(f"body: M {p['M']:.3f} kg, CoM {p['l'] * 100:.1f} cm above axle, wheel r {p['r'] * 1000:.1f} mm")
        print(f"motor: {motor.summary()}, backlash {motor.backlash_deg:g} deg, mode {motor.mode}")
        print(f"LQR K = [{', '.join(f'{k:.3g}' for k in K)}], slowest pole |z| {max(abs(poles)):.4f}")

    log = {k: np.zeros(ticks) for k in ("t", "pitch", "pitch_est", "x", "duty_l", "duty_r", "i_l", "i_r")}
    viewer = None
    if view:
        import mujoco.viewer as mjviewer
        viewer = mjviewer.launch_passive(
            model, data, key_callback=lambda k: keys.append(1.0 if k == 32 else -1.0 if k == 66 else 0))
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera("follow").id
        wall0 = time.perf_counter()

    fell_at = None
    key_push = (0.0, -1.0)                      # (force, end time)
    cur = (0.0, 0.0)
    for k in range(ticks):
        t = k * cfg.dt
        sd = data.sensordata
        # the encoder is on the motor shaft: with gearbox play it sees the rotor, not the wheel
        ang_l, ang_r = (gears[0].theta, gears[1].theta) if gears else (sd[adr["enc_l"]], sd[adr["enc_r"]])
        enc_l = math.floor(ang_l / (2 * math.pi) * cfg.cpr)
        enc_r = math.floor(ang_r / (2 * math.pi) * cfg.cpr)
        gyro_y = sd[adr["gyro"] + 1] + sens.gyro_bias + rng.normal(0, sens.gyro_noise)
        ax = sd[adr["accel"]] + rng.normal(0, sens.acc_noise)
        az = sd[adr["accel"] + 2] + rng.normal(0, sens.acc_noise)
        queue.append(ctrl.step(enc_l, enc_r, gyro_y, ax, az))
        duty = queue.pop(0)

        force = sum(f for t0, f in sc.pushes if t0 <= t < t0 + sc.push_len)
        while keys:
            f = keys.pop()
            if f:
                key_push = (f, t + sc.push_len)
        if t < key_push[1]:
            force += key_push[0]
        # push at the top of the frame: force at the CoM plus the matching torque
        lever = body.frame_height * math.cos(pitch_of(data, ch)) - (data.xipos[ch][2] - data.xpos[ch][2])
        data.xfrc_applied[ch] = [force, 0, 0, 0, lever * force, 0]

        for _ in range(sub):
            if gears:
                tl, il = gears[0].step(duty[0], data.qpos[qa_l], data.qvel[dof_l], model.opt.timestep)
                tr, ir = gears[1].step(duty[1], data.qpos[qa_r], data.qvel[dof_r], model.opt.timestep)
            else:
                tl, il = motor.torque(duty[0], data.qvel[dof_l])
                tr, ir = motor.torque(duty[1], data.qvel[dof_r])
            data.ctrl[:] = (tl, tr)
            mujoco.mj_step(model, data)
            cur = (il, ir)

        if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
            raise FloatingPointError(f"simulation went unstable at {t:.3f} s")
        pitch = pitch_of(data, ch)
        for key, v in (("t", t), ("pitch", pitch), ("pitch_est", ctrl.pitch), ("x", data.qpos[0]),
                       ("duty_l", duty[0]), ("duty_r", duty[1]), ("i_l", cur[0]), ("i_r", cur[1])):
            log[key][k] = v
        if fell_at is None and abs(pitch) > math.radians(45):
            fell_at = t
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            lag = wall0 + t - time.perf_counter()
            if lag > 0:
                time.sleep(lag)

    if viewer is not None:
        viewer.close()
    n = k + 1
    log = {key: v[:n] for key, v in log.items()}
    res = score(log, fell_at, cfg, sc)
    if plot:
        save_plot(log, res, OUT / "run.png")
    if not quiet:
        print(res["line"])
    return res, log


def score(log, fell_at, cfg, sc):
    settle = log["t"] > 0.5
    duty = np.maximum(abs(log["duty_l"]), abs(log["duty_r"]))
    cur = np.maximum(abs(log["i_l"]), abs(log["i_r"]))
    res = dict(
        fell_at=fell_at,
        max_tilt=math.degrees(abs(log["pitch"][settle]).max()) if settle.any() else 0.0,
        final_x=float(log["x"][-1]),
        saturated=float((duty >= cfg.duty_limit - 1e-9).mean()),
        peak_current=float(cur.max()),
        rms_current=float(np.sqrt((cur**2).mean())),
    )
    stats = (f"max tilt {res['max_tilt']:.1f} deg, ends {res['final_x'] * 100:+.1f} cm from start, "
             f"duty at limit {res['saturated'] * 100:.1f} % of ticks, "
             f"current peak {res['peak_current']:.2f} A / rms {res['rms_current']:.2f} A")
    if fell_at is not None:
        res["ok"] = False
        res["line"] = f"RESULT: FAIL - fell at {fell_at:.2f} s; {stats}"
    elif abs(res["final_x"]) > 0.05:
        res["ok"] = False
        res["line"] = f"RESULT: FAIL - stood but didn't return to its spot; {stats}"
    else:
        res["ok"] = True
        res["line"] = f"RESULT: OK - stood {sc.seconds:.0f} s and returned; {stats}"
    return res


def save_plot(log, res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    path.parent.mkdir(exist_ok=True)
    t = log["t"]
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t, np.degrees(log["pitch"]), label="true")
    ax[0].plot(t, np.degrees(log["pitch_est"]), label="estimated", lw=0.8)
    ax[0].set_ylabel("pitch (deg)")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, log["x"] * 100)
    ax[1].set_ylabel("position (cm)")
    ax[2].plot(t, log["duty_l"] * 100, label="left")
    ax[2].plot(t, log["duty_r"] * 100, label="right", lw=0.8)
    ax[2].set_ylabel("duty (%)")
    ax[2].legend(loc="upper right")
    ax[3].plot(t, log["i_l"], lw=0.6)
    ax[3].axhline(1.2, color="r", ls="--", lw=0.8)
    ax[3].axhline(-1.2, color="r", ls="--", lw=0.8)
    ax[3].set_ylabel("motor current (A)")
    ax[3].set_xlabel("time (s)")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(res["line"].replace("; ", "\n", 1), fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def apply_sets(objs, sets):
    for s in sets or []:
        name, val = s.split("=", 1)
        for o in objs:
            if hasattr(o, name):
                cur = getattr(o, name)
                setattr(o, name, type(cur)(val))
                break
        else:
            sys.exit(f"unknown parameter {name}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--view", action="store_true", help="open the MuJoCo viewer, run in real time")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--tilt", type=float, default=3.0, help="starting tilt, deg")
    ap.add_argument("--push", type=float, default=1.0, help="push force for 0.1 s, N (0 = none)")
    ap.add_argument("--duty-limit", type=float, default=0.5)
    ap.add_argument("--delay", type=int, default=1, help="ticks from snapshot to duty write")
    ap.add_argument("--mode", choices=("coast", "linear"), default="coast")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--set", action="append", metavar="NAME=VALUE",
                    help="override any Body, Motor, Config or Sensors field, e.g. battery_height=0.15")
    a = ap.parse_args()
    body, motor, cfg, sens = Body(), Motor(mode=a.mode), Config(duty_limit=a.duty_limit), Sensors()
    apply_sets([body, motor, cfg, sens], a.set)
    pushes = [(1.0, a.push), (5.0, -a.push)] if a.push else []
    sc = Scenario(seconds=a.seconds, tilt_deg=a.tilt, pushes=pushes, delay_ticks=a.delay, seed=a.seed)
    res, _ = run(body, motor, cfg, sc, sens, view=a.view)
    print(f"plot: {OUT / 'run.png'}")
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
