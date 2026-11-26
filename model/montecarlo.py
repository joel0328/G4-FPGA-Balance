"""montecarlo.py - does the controller still work on robots that aren't exactly the model?

    python montecarlo.py                   200 randomized robots, nominal controller
    python montecarlo.py --n 500 --set duty_limit=1.0
    python montecarlo.py --n 100 --min-pass 0.95

The controller is designed once, for the nominal Body and Motor. Each run then
simulates a different robot: masses, heights, motor constants, gearbox play and
stiffness, tyre grip, gyro bias and calibration error, sensor noise, loop delay,
starting tilt and push direction are drawn from the ranges in RANGES. A run
passes when the robot stands 10 s through the push and returns to its spot.

Prints the pass rate and which parameters matter most (failure rate in the
lowest vs the highest third of each range), writes out/montecarlo.csv, and ends
with a RESULT line (OK when the pass rate reaches --min-pass).
"""
import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

from controller import Config
from motor import Motor
from robot import Body
from sim import OUT, Scenario, Sensors, apply_sets, run

# (group, field, how, low, high): how = "scale" multiplies the nominal value,
# "add" adds to it, "set" replaces it
RANGES = [
    ("body", "frame_mass", "scale", 0.85, 1.15),
    ("body", "battery_mass", "scale", 0.85, 1.15),
    ("body", "boards_mass", "scale", 0.8, 1.2),
    ("body", "motor_mass", "scale", 0.9, 1.1),
    ("body", "wheel_mass", "scale", 0.85, 1.15),
    ("body", "battery_height", "add", -0.02, 0.02),
    ("body", "boards_height", "add", -0.02, 0.02),
    ("body", "wheel_radius", "scale", 0.97, 1.03),
    ("body", "tyre_friction", "set", 0.6, 1.2),
    ("motor", "vbus", "set", 11.5, 12.5),
    ("motor", "rpm_noload", "scale", 0.9, 1.1),
    ("motor", "i_stall", "scale", 0.75, 1.25),
    ("motor", "i_noload", "scale", 0.5, 1.5),
    ("motor", "efficiency", "set", 0.5, 0.8),
    ("motor", "rotor_inertia", "scale", 0.5, 2.0),
    ("motor", "backlash_deg", "set", 0.5, 3.0),
    ("motor", "gear_stiffness", "scale", 0.5, 2.0),
    ("sens", "gyro_bias", "set", -0.02, 0.02),
    ("sens", "gyro_cal_error", "set", -0.003, 0.003),
    ("sens", "gyro_noise", "scale", 0.5, 2.0),
    ("sens", "acc_noise", "scale", 0.5, 2.0),
    ("scenario", "delay_ticks", "int", 0, 2),
    ("scenario", "tilt_deg", "set", -4.0, 4.0),
    ("scenario", "push_sign", "sign", -1, 1),
]


def sample(rng):
    draw = {}
    for _, name, how, lo, hi in RANGES:
        if how == "int":
            draw[name] = int(rng.integers(lo, hi + 1))
        elif how == "sign":
            draw[name] = float(rng.choice([-1, 1]))
        else:
            draw[name] = float(rng.uniform(lo, hi))
    return draw


def one(args):
    i, draw, sets, push = args
    body, motor, cfg, sens = Body(), Motor(), Config(), Sensors()
    apply_sets([body, motor, cfg, sens], sets)
    design = (replace(body), replace(motor))
    groups = {"body": body, "motor": motor, "sens": sens}
    for grp, name, how, _, _ in RANGES:
        if grp not in groups or name == "gyro_cal_error":
            continue
        obj, v = groups[grp], draw[name]
        setattr(obj, name, getattr(obj, name) * v if how == "scale" else
                getattr(obj, name) + v if how == "add" else v)
    sens.gyro_bias_cal = sens.gyro_bias + draw["gyro_cal_error"]
    sc = Scenario(tilt_deg=draw["tilt_deg"], delay_ticks=draw["delay_ticks"], seed=1000 + i,
                  pushes=[(1.0, push * draw["push_sign"]), (5.0, -push * draw["push_sign"])])
    try:
        res, _ = run(body, motor, cfg, sc, sens, quiet=True, plot=False, design=design)
    except FloatingPointError:
        return dict(draw, ok=False, fell=False, unstable=True, max_tilt=float("nan"),
                    final_x=float("nan"), peak_current=float("nan"))
    return dict(draw, ok=res["ok"], fell=res["fell_at"] is not None, unstable=False,
                max_tilt=res["max_tilt"], final_x=res["final_x"], peak_current=res["peak_current"])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, default=200, help="robots to simulate")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--push", type=float, default=1.0, help="push force for 0.1 s, N")
    ap.add_argument("--min-pass", type=float, default=0.9, help="pass rate for RESULT OK")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="change the nominal robot (and controller design) first")
    ap.add_argument("--jobs", type=int, default=None)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    jobs = [(i, sample(rng), a.set, a.push) for i in range(a.n)]
    with ProcessPoolExecutor(a.jobs) as ex:
        rows = list(ex.map(one, jobs, chunksize=4))

    OUT.mkdir(exist_ok=True)
    with open(OUT / "montecarlo.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    ok = np.array([r["ok"] for r in rows])
    fell = np.array([r["fell"] for r in rows])
    bad = np.array([r["unstable"] for r in rows])
    tilt = np.array([r["max_tilt"] for r in rows])[~bad]
    print(f"{ok.sum()}/{len(rows)} passed ({ok.mean() * 100:.1f} %); fell {fell.sum()}, "
          f"stood but didn't return {(~ok & ~fell & ~bad).sum()}, simulation unstable {bad.sum()}")
    if tilt.size:
        print(f"max tilt: median {np.median(tilt):.1f} deg, 95th percentile {np.percentile(tilt, 95):.1f} deg")
    if (~ok).any():
        print("failure rate, lowest third vs highest third of each range (largest gaps first):")
        gaps = []
        for _, name, *_ in RANGES:
            v = np.array([r[name] for r in rows], dtype=float)
            lo_q, hi_q = np.quantile(v, [1 / 3, 2 / 3])
            lo, hi = ~ok[v <= lo_q], ~ok[v >= hi_q]
            if lo.size and hi.size:
                gaps.append((abs(hi.mean() - lo.mean()), name, lo.mean(), hi.mean()))
        for _, name, lo, hi in sorted(gaps, reverse=True)[:6]:
            print(f"  {name:16s} low {lo * 100:5.1f} %   high {hi * 100:5.1f} %")
    fixed = f" ({', '.join(a.set)})" if a.set else ""
    verdict = "OK" if ok.mean() >= a.min_pass and not bad.any() else "FAIL"
    print(f"RESULT: {verdict} - {ok.mean() * 100:.1f} % of {len(rows)} randomized robots passed "
          f"the 3 deg tilt + {a.push:g} N push test{fixed} (needs {a.min_pass * 100:.0f} %)")
    print(f"table: {OUT / 'montecarlo.csv'}")
    return 0 if verdict == "OK" else 1


if __name__ == "__main__":
    sys.exit(main())
