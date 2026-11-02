# Motors and encoders (map v5)

Two H-bridge channels for a TB6612FNG and two quadrature decoders, in
[`hbridge_pwm.v`](../rtl/hbridge_pwm.v) and
[`quad_decoder.v`](../rtl/quad_decoder.v). The registers are 0x25–0x28 and
0x34–0x37 in the [register map](register-map.md); wiring is on the
[wiring page](wiring.md#pmod-jb--tb6612fng-and-encoders-map-v5).

Status: verified in simulation; the hardware gate with the TB6612FNG and a
JGB37-520 motor is in progress.

## Hardware

| Part | Notes |
|---|---|
| TB6612FNG module | PWMA/PWMB and STBY tied to 3.3 V; the FPGA drives AIN1/AIN2 (BIN1/BIN2). VM ≤ 13.5 V, ~1.2 A continuous per channel |
| JGB37-520 gearmotor | 12 V, 178 RPM, Hall encoder on the motor shaft. Assumed 1:56 with 11 lines: 2464 counts per output revolution at x4 (to be measured) |
| Motor supply | 12 V, current-limited to about 1 A for bring-up |

## Behaviour

- **PWM:** M1_DUTY and M2_DUTY are signed Q15 (±32767 = ±100 %). Positive
  duties PWM IN1 at 20 kHz, negative ones IN2; the other pin stays low, so the
  off-time coasts. A new duty starts at the next PWM period.
- **Safety:** both pins are gated by ARMED, so a fault or disarm turns them
  off on the same clock. The duty registers are held at 0 while disarmed, so
  arming never starts a motor. The FPGA clamps |duty| to DUTY_LIMIT, which is
  50 % after reset and can't be changed while armed.
- **Encoders:** x4 decoding (4 counts per encoder line, up when A leads B)
  after a 2-flop synchronizer and a 0.64 µs glitch filter. The snapshot holds
  each position and the counts since the previous tick. A and B changing
  together counts in ENC_ERRS, not as a step.
- **Firmware:** the control step writes WDT_KICK, M1_DUTY and M2_DUTY in one
  burst and reads the encoders with the snapshot. On the console, `m25`
  followed by Enter or a space runs M1 at 25 % (`m-40` reverses), and `s`
  stops both. The app drops the duty to 0 whenever the FPGA reports disarmed.
  A `motor:` line each second shows the duty, encoder 1 count and speed, and
  the encoder errors.

## Verification

- Testbench: per-clock checks that no IN pin is high while disarmed and that
  both IN pins of a bridge are never high together; duty, clamp, reversal,
  fault-while-running, encoder counting, glitch rejection and snapshot timing.
- Mutation testing: 21 injected RTL bugs (9 safety, 12 motor/encoder) are all
  caught.
- Vivado: timing met with WNS 1.148 ns.
