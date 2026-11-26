"""motor.py - JGB37-520 gearmotor driven by a TB6612FNG the way the FPGA drives it.

Rated: 12 V, 178 RPM no-load at the output, 1:56 gearbox. The rest are assumptions
marked MEASURE: replace them with bench numbers (stall current, no-load current,
duty vs speed) once the motor runs on hardware.

Drive modes (duty in -1..1, after the FPGA's DUTY_LIMIT clamp):
  coast   what the FPGA does: sign-magnitude PWM on IN1 or IN2, the other low,
          so the off-time coasts. Current flows only during the on-time
          (inductance ignored), so there's no braking at zero duty and the
          back-EMF damping scales with |duty|.
  linear  ideal average-voltage drive: V = duty * Vbus, with full back-EMF
          braking at zero duty. The textbook model the LQR is designed on.
"""
from dataclasses import dataclass
import math


@dataclass
class Motor:
    vbus: float = 12.0               # V, motor supply
    rpm_noload: float = 178.0        # output shaft, at vbus (label)
    gear: float = 56.0               # gearbox ratio (typical for 178 RPM; MEASURE via encoder CPR)
    i_stall: float = 1.8             # A at vbus (MEASURE) -> winding resistance
    i_noload: float = 0.12           # A at vbus (MEASURE) -> gearbox friction
    efficiency: float = 0.65         # gearbox (assumed)
    rotor_inertia: float = 1.0e-6    # kg m^2, motor rotor (assumed)
    mode: str = "coast"

    @property
    def R(self):
        return self.vbus / self.i_stall

    @property
    def ke(self):
        """Back-EMF constant at the motor shaft, V s/rad (= torque constant, N m/A)."""
        w_nl = self.rpm_noload * self.gear * 2 * math.pi / 60
        return (self.vbus - self.R * self.i_noload) / w_nl

    @property
    def friction(self):
        """Coulomb friction at the output shaft, N m (what the no-load current fights)."""
        return self.efficiency * self.gear * self.ke * self.i_noload

    @property
    def armature(self):
        """Rotor inertia reflected to the output shaft, kg m^2."""
        return self.rotor_inertia * self.gear ** 2

    def torque(self, duty, w_out):
        """Output torque (N m) and winding current (A) for a duty and output speed (rad/s)
        relative to the motor body."""
        emf = self.ke * self.gear * w_out
        if self.mode == "linear":
            i = (duty * self.vbus - emf) / self.R
        else:
            i = abs(duty) * (math.copysign(self.vbus, duty) - emf) / self.R
        return self.efficiency * self.gear * self.ke * i, i

    def linear_gains(self):
        """torque = b * duty - c * w_out in linear mode."""
        k = self.efficiency * self.gear * self.ke / self.R
        return k * self.vbus, k * self.ke * self.gear

    def summary(self):
        return (f"R {self.R:.2f} ohm, ke {self.ke * 1000:.2f} mV s/rad, "
                f"stall {self.efficiency * self.gear * self.ke * self.i_stall:.2f} N m, "
                f"friction {self.friction:.3f} N m, rotor at output {self.armature * 1000:.2f} g m^2")
