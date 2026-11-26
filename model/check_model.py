"""check_model.py - self-checks for the simulator. Ends with a RESULT line.

1. The hand-derived linear model (what the LQR is designed on) matches MuJoCo's
   accelerations for a tilt, a duty step and back-EMF braking (gearbox friction
   off, since the linear model leaves it out).
2. The motor constants reproduce the label: 178 RPM at full duty draws the
   no-load current.
3. The default body balances, recovers from 1 N pushes both ways and returns.
"""
import math
import sys

import mujoco
import numpy as np

from controller import linear_model
from motor import Motor
from robot import Body, plant
from sim import Scenario, make, run, set_tilt

failures = []


def check(name, ok, detail):
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        failures.append(name)


motor = Motor(mode="linear", backlash_deg=0)        # rigid gearbox: what the linear model assumes
model, data = make(Body(), motor)
model.dof_frictionloss[:] = 0
p = plant(model, motor)
A, B = linear_model(p, motor)
for _ in range(200):
    mujoco.mj_step(model, data)
z, r = data.qpos[2], p["r"]
dofs = [model.joint(j).dofadr[0] for j in ("wheel_l", "wheel_r")]
for name, tilt, duty, w in (("tilt 1 deg", 1.0, 0.0, 0.0), ("duty 0.2", 0.0, 0.2, 0.0),
                            ("rolling 5 rad/s", 0.0, 0.0, 5.0)):
    data.qpos[:] = 0
    data.qpos[2] = z
    data.qvel[:] = 0
    set_tilt(model, data, tilt)
    data.qvel[0] = r * w
    data.qvel[dofs] = w
    tq, _ = motor.torque(duty, w)
    data.ctrl[:] = (tq, tq)
    mujoco.mj_forward(model, data)
    sim = np.array([r * (data.qacc[dofs[0]] + data.qacc[4]), data.qacc[4]])
    lin = (A @ np.array([0, math.radians(tilt), r * w, 0]) + B[:, 0] * duty)[2:]
    err = np.abs(sim - lin).max() / np.abs(lin).max()
    check(f"linear model, {name}", err < 0.10,
          f"MuJoCo x_dd {sim[0]:+.3f} pitch_dd {sim[1]:+.3f}, linear {lin[0]:+.3f} {lin[1]:+.3f} ({err * 100:.1f} %)")

_, i = Motor(mode="linear").torque(1.0, 178 * 2 * math.pi / 60)
check("motor no-load", abs(i - 0.12) < 1e-6, f"{i:.3f} A at 178 RPM")

res, _ = run(sc=Scenario(pushes=[(1.0, 1.0), (5.0, -1.0)]), quiet=True, plot=False)
check("balance + 1 N pushes", res["ok"], res["line"][8:])

print("RESULT: OK" if not failures else f"RESULT: FAIL - {', '.join(failures)}")
sys.exit(1 if failures else 0)
