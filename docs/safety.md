# Safety layer (map v4)

The FPGA owns the actuator outputs, so a firmware bug can't run them away.
Outputs (LEDs 0–13 and both motor bridges) are forced off unless **armed**.
The logic is in [`regfile.v`](../rtl/regfile.v); the registers are 0x20–0x24
in the [register map](register-map.md).

- **Arming** needs all of: SW15 up (hardware arm enable), the key
  `0x41524D21` written to ARM, no fault latched, and the loop tick running.
  Writing any other value to ARM disarms. TICK_DIV and WDT_TICKS can't change
  while armed.
- **Watchdog:** while armed, the MCU must write WDT_KICK; a loop tick that
  finds WDT_TICKS (default 3) ticks since the last kick latches a fault.
  Ordinary frames don't feed it.
- **Faults** latch in FAULT_STATUS and disarm at once: WDT (missed kicks),
  LINK (4 rejected frames in a row while armed), ESTOP (btnD, any time),
  ARM_LOST (SW15 dropped while armed). They raise IRQ bit 3 and hold FAULT_N
  low until cleared with FAULT_CLEAR; ESTOP can't be cleared while btnD is
  held. Clearing never re-arms: that takes a fresh key.
- **LEDs:** LED15 = armed, LED14 = fault.
- **Snapshot:** at every LOOP_TICK the FPGA latches TICK_COUNT, TIME_US, the
  inputs, the safety state and both encoders into SNAP_* (0x30–0x37; 0x38–0x3F
  are reserved for the IMU). If a frame is in progress it publishes them when
  CS rises, so one burst read of 0x30… is always one sample.

The firmware never arms or clears faults by itself. The demo firmware's 1 kHz
control step reads the snapshot, writes the LEDs and display, kicks the
watchdog and clears pending IRQs, all in interrupts. Console commands on the
VCP: `a` arm, `d` disarm, `c` clear faults, `k` stop/restart kicks (to test
the watchdog). btnC re-runs the self-tests, clears faults and arms.

## Verification

- Testbench: a per-clock check that armed and fault never coexist, plus
  checks of each fault cause, the locks and the snapshot timing.
- Mutation testing: injected RTL bugs (outputs not gated, watchdog off by one,
  arming without SW15 and others) are all caught by the testbench.
- Hardware: stopping kicks latched WDT and cut the outputs; SW15 down →
  ARM_LOST; btnD → ESTOP; arming refused with SW15 down; a 30-minute armed
  soak had no false trips.
