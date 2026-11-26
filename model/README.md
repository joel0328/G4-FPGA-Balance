# Balancer simulator (MuJoCo)

A physics model of the two-wheeled balancer for trying chassis designs and
control algorithms before the hardware exists. It runs the controller the way
the STM32 will: once per 1 kHz tick, on what the FPGA snapshot provides
(integer encoder counts, gyro and accelerometer), with its duties applied one
tick later and clamped to DUTY_LIMIT.

## Setup

```
cd model
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements.txt     # Linux/macOS: .venv/bin/python
```

## Use

```
python check_model.py           # self-checks (model vs MuJoCo, motor constants, balance)
python sim.py                   # 10 s: start tilted 3°, push 1 N forward at 1 s, back at 5 s
python sim.py --view            # same in the 3D viewer, real time; space = push, B = push back
python sim.py --push 1.5 --duty-limit 1.0 --set battery_height=0.16
python sweep.py --vary battery_height=0.04,0.10,0.16 --vary wheel_radius=0.0325,0.04,0.05
```

Each script ends with a `RESULT:` line. `sim.py` writes `out/run.png` (pitch,
position, duty, motor current); `sweep.py` writes `out/sweep.csv`. `--set`
overrides any field of `Body`, `Motor`, `Config` or `Sensors` by name.

CI runs `check_model.py`, the default `sim.py` scenario and the sweep above on
every push; the plot and sweep table are attached to each run as the
`balancer-sim` artifact.

## Files

| File | Contents |
|---|---|
| `robot.py` | `Body`: every chassis dimension and mass; builds the MuJoCo model and reads its mass properties back |
| `motor.py` | `Motor`: JGB37-520 on a TB6612FNG, in the FPGA's coast-PWM mode or an ideal linear drive |
| `controller.py` | Linear model, LQR design (with the one-tick delay), and `Controller`: the per-tick estimator and control law, kept portable to C |
| `sim.py` | Runs a scenario: sensor models, pushes, logging, plot, viewer |
| `sweep.py` | Largest survivable push for each combination of one or two parameters, with the LQR redesigned for each body |
| `check_model.py` | Self-checks |

## What is modelled

- **Body:** frame, two motors on the axle, battery and boards as boxes at
  set heights, wheels with tyre friction. Pitch inertia and CoM come from the
  geometry, and the LQR is redesigned from them on every run.
- **Motor:** winding resistance, back-EMF, gearbox friction and efficiency,
  rotor inertia reflected through the 1:56 gearbox (bigger than the wheel's own
  inertia). In `coast` mode (the FPGA's sign-magnitude PWM) current flows only
  during the on-time, so zero duty coasts instead of braking.
- **Sensors:** encoder counts quantised to 2464 per output revolution, gyro
  noise and an imperfect bias calibration, accelerometer noise; a complementary
  filter estimates pitch.
- **Timing:** 1 kHz control, 0.5 ms physics, one tick from sample to duty.

Not modelled yet: gearbox backlash, motor inductance, IMU mounting
misalignment, the battery sagging under load, and yaw manoeuvres beyond
holding a heading.

## Assumptions to replace with measurements

`Motor` fields marked MEASURE in `motor.py`: stall current (sets the winding
resistance), no-load current (gearbox friction), gear ratio and encoder counts
per revolution. The 12 V / 178 RPM label is used as given. Rotor inertia and
gearbox efficiency are estimates. Once the motor runs on the bench, fit them
from a duty-step response, then rerun `check_model.py` and the sweeps.

## First findings (assumed motor constants)

- With the FPGA's reset DUTY_LIMIT of 50 % the default body recovers from a
  1 N, 0.1 s push at the top of the frame but not 1.5 N; at 100 % it recovers
  from 1.5 N with 11° of tilt.
- Coast-mode PWM recovers from larger pushes than an ideal drive, because
  partial duty doesn't brake the wheels.
- Larger wheels and a higher battery both raise the push it survives:
  2.06 N with 100 mm wheels and the battery 16 cm up, versus 1.03 N with
  65 mm wheels and the battery at 4 cm.
- Peak motor current in these pushes stays under the TB6612's 1.2 A rating.
