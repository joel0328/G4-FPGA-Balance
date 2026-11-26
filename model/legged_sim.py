"""legged_sim.py - run the wheel-legged balancer through a course, or drive it by keyboard.

    python legged_sim.py               the course below, ends with a RESULT line, writes out/legged.png
    python legged_sim.py --no-roll     same with the leg roll control off, for comparison
    python legged_sim.py --view        drive it in the 3D viewer:
        up/down    speed +/- 0.1 m/s        left/right  turn rate +/- 0.5 rad/s
        PgUp/PgDn  height +/- 1 cm          S           stop (speed and turn to 0)
        space / B  push forward / back

The course: start tilted 3 deg at 15 cm leg length; push 2 N forward at 1 s;
crouch to 10 cm at 3 s, stand to 18 cm at 4.2 s, back to 15 cm at 5.4 s; drive
at 0.4 m/s from 6 s, with the left wheel riding over a 1.5 cm plank; turn left at
1.5 rad/s from 9 s (leaning into the turn); stop at 11 s; end at 13 s.
"""
import argparse
import math
import sys
import time

import mujoco
import numpy as np

from legged_controller import Command, LegConfig, LegController, schedule
from legged_robot import LegBody, build_xml, leg_fk, set_pose
from motor import Motor
from sim import OUT, Sensors, apply_sets

JOINTS = ("hip_l", "knee_l", "hip_r", "knee_r")


def course(t):
    """Command and push force (N) at time t."""
    height = 0.15
    if 3.0 <= t < 4.2:
        height = 0.10
    elif 4.2 <= t < 5.4:
        height = 0.18
    speed = 0.4 if 6.0 <= t < 11.0 else 0.0
    turn = 1.5 if 9.0 <= t < 11.0 else 0.0
    push = 2.0 if 1.0 <= t < 1.1 else 0.0
    return Command(speed=speed, turn=turn, height=height), push


def attitude(data, bid):
    """Pitch and roll of the torso in its own frame (independent of heading)."""
    up = data.xmat[bid].reshape(3, 3)[2]          # world z seen from the torso
    return math.atan2(-up[0], math.hypot(up[1], up[2])), math.atan2(up[1], up[2])


def run(body=None, motor=None, cfg=None, sens=None, seconds=13.0, tilt_deg=3.0, delay_ticks=1,
        seed=1, view=False, quiet=False, plot=True):
    body, motor = body or LegBody(), motor or Motor()
    cfg, sens = cfg or LegConfig(), sens or Sensors()
    model = mujoco.MjModel.from_xml_string(build_xml(body, motor))
    data = mujoco.MjData(model)
    table = schedule(model, body, motor, cfg.dt)
    ctrl = LegController(body, table, cfg, gyro_bias=(sens.gyro_bias_cal,) * 3)
    rng = np.random.default_rng(seed)

    # settle onto the floor upright, then tilt and let go
    set_pose(model, data, body, body.height_nominal)
    for _ in range(400):
        mujoco.mj_step(model, data)
    data.qvel[:] = 0
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([0.0, 1.0, 0.0]), math.radians(tilt_deg))
    data.qpos[3:7] = q
    mujoco.mj_forward(model, data)

    tb = model.body("torso").id
    sadr = lambda n: model.sensor_adr[model.sensor(n).id]
    a_gyro, a_acc, a_el, a_er = sadr("gyro"), sadr("accel"), sadr("enc_l"), sadr("enc_r")
    a_joint = [sadr(j) for j in JOINTS]
    act_joint = [model.actuator(j).id for j in JOINTS]
    act_wheel = [model.actuator("wheel_l").id, model.actuator("wheel_r").id]
    dof_wheel = [model.joint("wheel_l").dofadr[0], model.joint("wheel_r").dofadr[0]]
    qadr_joint = [model.joint(j).qposadr[0] for j in JOINTS]
    sub = round(cfg.dt / model.opt.timestep)
    ticks = round(seconds / cfg.dt)
    queue = [((0.0, 0.0), None)] * delay_ticks
    torso_top = body.torso_height - 0.02

    if not quiet:
        print(f"body: {model.body_subtreemass[tb]:.2f} kg, wheels {body.wheel_radius * 2000:.0f} mm, "
              f"legs {body.height_min * 100:.0f}-{body.height_max * 100:.0f} cm")
        print(f"LQR scheduled at {len(table[0])} leg lengths; balance pitch "
              f"{math.degrees(table[2][0]):+.1f} to {math.degrees(table[2][-1]):+.1f} deg")

    keys, tele = [], Command(height=body.height_nominal)
    viewer = None
    if view:
        import mujoco.viewer as mjviewer
        viewer = mjviewer.launch_passive(model, data, key_callback=keys.append)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera("follow").id
        wall0 = time.perf_counter()
        ticks = 10**9

    names = ("t", "pitch", "roll", "height", "height_cmd", "dlen", "speed", "speed_cmd", "yaw_rate",
             "turn_cmd", "x", "duty_l", "duty_r", "i_l", "i_r", "jtau")
    log = {k: [] for k in names}
    fell_at = None
    key_push = (0.0, -1.0)
    cur = (0.0, 0.0)
    for k in range(ticks):
        t = k * cfg.dt
        if view:
            cmd, force = tele, 0.0
            while keys:
                key = keys.pop(0)
                if key == 265: tele.speed = min(0.8, tele.speed + 0.1)
                elif key == 264: tele.speed = max(-0.8, tele.speed - 0.1)
                elif key == 263: tele.turn = min(3.0, tele.turn + 0.5)
                elif key == 262: tele.turn = max(-3.0, tele.turn - 0.5)
                elif key == 266: tele.height = min(body.height_max, tele.height + 0.01)
                elif key == 267: tele.height = max(body.height_min, tele.height - 0.01)
                elif key == 83: tele.speed, tele.turn = 0.0, 0.0
                elif key in (32, 66): key_push = (2.0 if key == 32 else -2.0, t + 0.1)
            if t < key_push[1]:
                force = key_push[0]
        else:
            cmd, force = course(t)

        sd = data.sensordata
        enc = [math.floor(sd[a] / (2 * math.pi) * cfg.cpr) for a in (a_el, a_er)]
        joints = [sd[a] for a in a_joint]
        gyro = [sd[a_gyro + i] + sens.gyro_bias + rng.normal(0, sens.gyro_noise) for i in range(3)]
        acc = [sd[a_acc + i] + rng.normal(0, sens.acc_noise) for i in range(3)]
        queue.append(ctrl.step(enc[0], enc[1], joints, gyro, acc, cmd))
        duty, jt = queue.pop(0)
        if jt is not None:
            for a, q in zip(act_joint, jt):
                data.ctrl[a] = q

        R = data.xmat[tb].reshape(3, 3)
        lever = torso_top * R[2, 2] - (data.xipos[tb][2] - data.xpos[tb][2])
        data.xfrc_applied[tb] = [force * R[0, 0], force * R[1, 0], 0, 0, 0, 0]
        data.xfrc_applied[tb][3:] = np.cross([0, 0, lever], data.xfrc_applied[tb][:3])

        for _ in range(sub):
            tl, il = motor.torque(duty[0], data.qvel[dof_wheel[0]])
            tr, ir = motor.torque(duty[1], data.qvel[dof_wheel[1]])
            data.ctrl[act_wheel[0]], data.ctrl[act_wheel[1]] = tl, tr
            mujoco.mj_step(model, data)
            cur = (il, ir)

        pitch, roll = attitude(data, tb)
        hl = -leg_fk(body, data.qpos[qadr_joint[0]], data.qpos[qadr_joint[1]])[1]
        hr = -leg_fk(body, data.qpos[qadr_joint[2]], data.qpos[qadr_joint[3]])[1]
        fwd = data.qvel[0] * R[0, 0] + data.qvel[1] * R[1, 0]
        jtau = max(abs(data.actuator_force[a]) for a in act_joint)
        row = (t, pitch, roll, (hl + hr) / 2, cmd.height, hl - hr, fwd, cmd.speed, data.qvel[5],
               cmd.turn, data.qpos[0], duty[0], duty[1], cur[0], cur[1], jtau)
        for key, v in zip(names, row):
            log[key].append(v)
        if fell_at is None and (abs(pitch) > math.radians(45) or abs(roll) > math.radians(45)):
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

    log = {key: np.array(v) for key, v in log.items()}
    res = score(log, fell_at, body, cfg) if not view else {"ok": True, "line": "RESULT: OK - viewer closed"}
    if plot and not view:
        save_plot(log, res, OUT / "legged.png")
    if not quiet:
        print(res["line"])
    return res, log


def score(log, fell_at, body, cfg):
    t = log["t"]
    win = lambda a, b: (t >= a) & (t < b)
    deg = math.degrees
    plank = (log["x"] > 0.6) & (log["x"] < 0.9) & (t < 9.0)     # left wheel on the plank
    turn = win(10.0, 11.0)
    lean_target = -math.atan(0.4 * 1.5 / 9.81)
    res = dict(
        fell_at=fell_at,
        plank_roll=deg(abs(log["roll"][plank]).mean()) if plank.any() else float("nan"),
        turn_roll=deg(log["roll"][turn].mean()),
        turn_rate=float(log["yaw_rate"][turn].mean()),
        crouch_err=float(abs(log["height"][win(4.0, 4.2)] - 0.10).max()),
        stand_err=float(abs(log["height"][win(5.2, 5.4)] - 0.18).max()),
        peak_current=float(np.maximum(abs(log["i_l"]), abs(log["i_r"])).max()),
        peak_joint=float(log["jtau"].max()),
    )
    stats = (f"roll on the plank {res['plank_roll']:.1f} deg, lean in turn {res['turn_roll']:+.1f} deg "
             f"(target {deg(lean_target):+.1f}), turn rate {res['turn_rate']:.2f}/1.50 rad/s, "
             f"height errors {res['crouch_err'] * 1000:.1f}/{res['stand_err'] * 1000:.1f} mm, "
             f"wheel current peak {res['peak_current']:.2f} A, joint torque peak {res['peak_joint']:.2f} N m")
    fails = []
    if fell_at is not None:
        fails.append(f"fell at {fell_at:.2f} s")
    else:
        if cfg.roll_on and not res["plank_roll"] < 1.0:
            fails.append("body not kept level on the plank")
        if cfg.roll_on and cfg.lean_in_turns and abs(res["turn_roll"] - deg(lean_target)) > 2.0:
            fails.append("wrong lean in the turn")
        if abs(res["turn_rate"] - 1.5) > 0.25:
            fails.append("turn rate off")
        if max(res["crouch_err"], res["stand_err"]) > 0.005:
            fails.append("height not reached")
    res["ok"] = not fails
    res["line"] = (f"RESULT: OK - course done; {stats}" if not fails
                   else f"RESULT: FAIL - {', '.join(fails)}; {stats}")
    return res


def save_plot(log, res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    path.parent.mkdir(exist_ok=True)
    t = log["t"]
    fig, ax = plt.subplots(5, 1, figsize=(9, 11), sharex=True)
    ax[0].plot(t, np.degrees(log["pitch"]), label="pitch")
    ax[0].plot(t, np.degrees(log["roll"]), label="roll")
    ax[0].set_ylabel("deg")
    ax[1].plot(t, log["height"] * 100, label="leg length")
    ax[1].plot(t, log["height_cmd"] * 100, "--", lw=0.8, label="command")
    ax[1].plot(t, log["dlen"] * 100, label="left - right")
    ax[1].set_ylabel("cm")
    ax[2].plot(t, log["speed"], label="speed")
    ax[2].plot(t, log["speed_cmd"], "--", lw=0.8, label="command")
    ax[2].plot(t, log["yaw_rate"], label="yaw rate (rad/s)")
    ax[2].plot(t, log["turn_cmd"], "--", lw=0.8, label="turn command")
    ax[2].set_ylabel("m/s, rad/s")
    ax[3].plot(t, log["duty_l"] * 100, label="left")
    ax[3].plot(t, log["duty_r"] * 100, lw=0.8, label="right")
    ax[3].set_ylabel("wheel duty (%)")
    ax[4].plot(t, log["i_l"], lw=0.6, label="wheel current (A)")
    ax[4].plot(t, log["jtau"], lw=0.6, label="peak joint torque (N m)")
    ax[4].set_xlabel("time (s)")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper left", fontsize=7)
    for a, label in ((3.0, "crouch"), (4.2, "stand"), (6.0, "drive"), (9.0, "turn"), (11.0, "stop")):
        ax[0].axvline(a, color="k", lw=0.5, alpha=0.4)
        ax[0].text(a, ax[0].get_ylim()[1], label, fontsize=7, va="top")
    fig.suptitle(res["line"].replace("; ", "\n", 1), fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--view", action="store_true", help="drive it in the 3D viewer")
    ap.add_argument("--no-roll", action="store_true", help="legs don't level the body")
    ap.add_argument("--seconds", type=float, default=13.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--set", action="append", metavar="NAME=VALUE",
                    help="override any LegBody, Motor, LegConfig or Sensors field")
    a = ap.parse_args()
    body, motor, cfg, sens = LegBody(), Motor(), LegConfig(), Sensors()
    apply_sets([body, motor, cfg, sens], a.set)
    if a.no_roll:
        cfg.roll_on = False
    res, _ = run(body, motor, cfg, sens, seconds=a.seconds, seed=a.seed, view=a.view)
    if not a.view:
        print(f"plot: {OUT / 'legged.png'}")
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
