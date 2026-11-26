"""sweep.py - compare body designs by the biggest push each one survives.

    python sweep.py --vary battery_height=0.04,0.10,0.16 --vary wheel_radius=0.0325,0.04,0.05
    python sweep.py --vary frame_height=0.15,0.20,0.25 --set duty_limit=1.0

For every combination (one or two --vary parameters, any Body, Motor, Config or
Sensors field) the LQR is redesigned for that body, then a bisection finds the
largest 0.1 s push at the top of the frame it recovers from. Prints a table,
writes out/sweep.csv and ends with a RESULT line naming the best design.
"""
import argparse
import csv
import itertools
import sys
from concurrent.futures import ProcessPoolExecutor

from controller import Config
from motor import Motor
from robot import Body
from sim import OUT, Scenario, Sensors, apply_sets, run


def survives(sets, force):
    body, motor, cfg, sens = Body(), Motor(), Config(), Sensors()
    apply_sets([body, motor, cfg, sens], sets)
    sc = Scenario(seconds=4.0, tilt_deg=0.0, pushes=[(0.5, force)])
    res, _ = run(body, motor, cfg, sc, sens, quiet=True, plot=False)
    return res["fell_at"] is None


def max_push(sets, hi=12.0, steps=8):
    """Largest push force (N, 0.1 s) survived, to within hi / 2**steps."""
    if not survives(sets, 0.0):
        return 0.0
    lo = 0.0
    for _ in range(steps):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if survives(sets, mid) else (lo, mid)
    return lo


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--vary", action="append", required=True, metavar="NAME=V1,V2,...")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="fixed overrides applied to every design")
    ap.add_argument("--jobs", type=int, default=None)
    a = ap.parse_args()
    axes = []
    for v in a.vary:
        name, vals = v.split("=", 1)
        axes.append((name, vals.split(",")))
    names = [n for n, _ in axes]
    combos = list(itertools.product(*[vals for _, vals in axes]))
    designs = [a.set + [f"{n}={v}" for n, v in zip(names, combo)] for combo in combos]
    with ProcessPoolExecutor(a.jobs) as ex:
        forces = list(ex.map(max_push, designs))

    OUT.mkdir(exist_ok=True)
    with open(OUT / "sweep.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(names + ["max_push_N", "impulse_Ns"])
        for combo, force in zip(combos, forces):
            w.writerow(list(combo) + [f"{force:.2f}", f"{force * 0.1:.3f}"])
    print("  ".join(f"{n:>16s}" for n in names) + "   max push (0.1 s)")
    for combo, force in zip(combos, forces):
        print("  ".join(f"{v:>16s}" for v in combo) + f"   {force:5.2f} N  ({force * 0.1:.3f} N s)")
    best = max(range(len(combos)), key=lambda i: forces[i])
    desc = ", ".join(f"{n}={v}" for n, v in zip(names, combos[best]))
    fixed = f" with {', '.join(a.set)}" if a.set else ""
    print(f"RESULT: best {desc}: survives {forces[best]:.2f} N for 0.1 s{fixed}")
    print(f"table: {OUT / 'sweep.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
