"""robot.py - parametric two-wheeled balancer, built as a MuJoCo model.

Body is every number you'd change while designing the chassis. build_xml() turns
it into MJCF; plant() reads the mass properties back from the compiled model, so
the controller design always matches the body being simulated.

Frame: x forward, y left, z up. The chassis origin sits on the wheel axle.
Positive pitch leans the top forward (+x); positive wheel angle rolls forward.
"""
from dataclasses import dataclass, fields

import mujoco
import numpy as np

from motor import Motor


@dataclass
class Body:
    # wheels
    wheel_radius: float = 0.0325     # m (65 mm wheels)
    wheel_width: float = 0.026       # m
    wheel_mass: float = 0.045        # kg each, tyre + hub
    track: float = 0.17              # m, wheel centre to wheel centre
    # printed frame: a box from just below the axle up to frame_height
    frame_height: float = 0.20       # m above the axle
    frame_depth: float = 0.06        # m, front to back
    frame_width: float = 0.12        # m, side to side
    frame_mass: float = 0.20         # kg
    # parts mounted on the frame (heights are of their centres, above the axle)
    motor_mass: float = 0.17         # kg each, JGB37-520 with encoder, on the axle
    battery_mass: float = 0.14       # kg, 3S LiPo
    battery_height: float = 0.10     # m
    boards_mass: float = 0.15        # kg, Basys 3 + Nucleo + TB6612 + wiring
    boards_height: float = 0.17      # m
    imu_height: float = 0.05         # m, where the IMU sits

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


def build_xml(b: Body, motor: Motor, timestep=0.0005) -> str:
    r, h = b.wheel_radius, b.frame_height
    hw = b.track / 2
    m_len = hw - b.wheel_width / 2 - 0.004          # motor can runs from the centre to the wheel
    fz = (h - 0.01) / 2                             # frame box centre: from 1 cm below the axle to h
    ghost = 'contype="0" conaffinity="0"'           # mass and looks only, no collisions
    wheel = lambda side, y: f"""
      <body name="wheel_{side}" pos="0 {y:.4f} 0">
        <joint name="wheel_{side}" type="hinge" axis="0 1 0" armature="{motor.armature:.6g}"
               frictionloss="{motor.friction:.6g}"/>
        <geom name="wheel_{side}" type="cylinder" size="{r} {b.wheel_width / 2}" euler="90 0 0"
              mass="{b.wheel_mass}" rgba=".15 .15 .15 1" friction="1 0.005 0.0001"/>
        <geom type="box" size="{r * 0.8} 0.001 {r * 0.12}" pos="0 {(1 if y > 0 else -1) * (b.wheel_width / 2 + 0.0005):.4f} 0"
              {ghost} mass="0" rgba=".9 .6 .1 1"/>
      </body>"""
    return f"""
<mujoco model="balancer">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".82 .84 .86" rgb2=".72 .75 .78"
             width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
  </asset>
  <worldbody>
    <light pos="0 -1 2" dir="0 0.5 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="grid"/>
    <body name="chassis" pos="0 0 {r}">
      <freejoint name="chassis"/>
      <camera name="follow" mode="trackcom" pos="0.55 -0.55 0.25" xyaxes="0.707 0.707 0 -0.2 0.2 0.96"/>
      <geom name="frame" type="box" size="{b.frame_depth / 2} {b.frame_width / 2} {(h + 0.01) / 2:.4f}"
            pos="0 0 {fz:.4f}" mass="{b.frame_mass}" rgba=".25 .45 .75 .55"/>
      <geom name="motor_l" type="cylinder" fromto="0 0.004 0 0 {m_len:.4f} 0" size="0.0185"
            {ghost} mass="{b.motor_mass}" rgba=".6 .6 .62 1"/>
      <geom name="motor_r" type="cylinder" fromto="0 -0.004 0 0 {-m_len:.4f} 0" size="0.0185"
            {ghost} mass="{b.motor_mass}" rgba=".6 .6 .62 1"/>
      <geom name="battery" type="box" size="0.0175 0.035 0.0125" pos="0 0 {b.battery_height}"
            {ghost} mass="{b.battery_mass}" rgba=".85 .25 .2 1"/>
      <geom name="boards" type="box" size="0.025 0.055 0.012" pos="0 0 {b.boards_height}"
            {ghost} mass="{b.boards_mass}" rgba=".1 .55 .3 1"/>
      <site name="imu" pos="0 0 {b.imu_height}" size="0.006" rgba="1 1 0 1"/>
      {wheel("l", hw)}
      {wheel("r", -hw)}
    </body>
  </worldbody>
  <actuator>
    <motor name="wheel_l" joint="wheel_l"/>
    <motor name="wheel_r" joint="wheel_r"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
    <jointpos name="enc_l" joint="wheel_l"/>
    <jointpos name="enc_r" joint="wheel_r"/>
  </sensor>
</mujoco>"""


def _inertia_about_y(model, body_id):
    """Moment of inertia about the body's y axis through its CoM."""
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.body_iquat[body_id])
    rot = rot.reshape(3, 3)
    return (rot @ np.diag(model.body_inertia[body_id]) @ rot.T)[1, 1]


def plant(model):
    """Mass properties the planar balance model needs, read from the compiled model."""
    ch, wl = model.body("chassis").id, model.body("wheel_l").id
    return dict(
        M=model.body_mass[ch],                       # everything that pitches with the chassis
        l=model.body_ipos[ch][2],                    # its CoM height above the axle
        I_b=_inertia_about_y(model, ch),             # its pitch inertia about that CoM
        m_w=model.body_mass[wl],                     # one wheel
        I_w=_inertia_about_y(model, wl),
        J_m=model.dof_armature[model.joint("wheel_l").dofadr[0]],   # reflected rotor, one side
        r=model.geom_size[model.geom("wheel_l").id][0],
    )
