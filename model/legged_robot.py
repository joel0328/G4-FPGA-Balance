"""legged_robot.py - wheel-legged balancer: a hip and a knee per side, a wheel at each foot.

LegBody holds every dimension and mass. Each leg is a thigh and a shank in the
sagittal plane (knee pointing forward); the wheel and its gearmotor sit at the
foot. Hip and knee are position servos with a torque limit (a geared BLDC or a
smart servo), driven from the 1 kHz loop; the knee motor sits at the hip and
drives the knee through a belt, so the leg itself stays light.

Frame: x forward, y left, z up. The torso origin is on the hip axis. Leg length
L is the hip-to-axle distance; the controller keeps each foot under its hip.
"""
import math
from dataclasses import dataclass, fields

import mujoco
import numpy as np

from motor import Motor
from robot import _inertia_about_y


@dataclass
class LegBody:
    # wheels (at the feet)
    wheel_radius: float = 0.05       # m (100 mm wheels)
    wheel_width: float = 0.03
    wheel_mass: float = 0.08         # kg each
    track: float = 0.26              # m, wheel centre to wheel centre
    wheel_motor_mass: float = 0.17   # kg each, JGB37-520 at the foot
    # legs
    hip_width: float = 0.18          # m, hip joint to hip joint
    thigh: float = 0.10              # m, hip to knee
    shank: float = 0.10              # m, knee to wheel axle
    thigh_mass: float = 0.06         # kg each
    shank_mass: float = 0.05         # kg each
    joint_torque: float = 3.0        # N m, hip and knee servo limit
    joint_kp: float = 60.0           # N m/rad, servo stiffness
    joint_kv: float = 1.5            # N m s/rad, servo damping
    joint_armature: float = 5e-4     # kg m^2, servo rotor through its gearbox
    hip_motor_mass: float = 0.10     # kg each
    knee_motor_mass: float = 0.10    # kg each, mounted at the hip (belt to the knee)
    # torso (heights are of centres, above the hip axis)
    torso_depth: float = 0.08
    torso_width: float = 0.14
    torso_height: float = 0.10
    torso_mass: float = 0.30         # printed shell
    battery_mass: float = 0.14
    battery_height: float = 0.03
    boards_mass: float = 0.15
    boards_height: float = 0.07
    imu_height: float = 0.0
    # leg length range
    height_min: float = 0.09
    height_max: float = 0.19
    height_nominal: float = 0.15

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]


def leg_ik(b: LegBody, length, x_foot=0.0):
    """Hip and knee angles that put the axle at (x_foot, -length) from the hip, knee forward."""
    l1, l2 = b.thigh, b.shank
    d = min(math.hypot(x_foot, length), l1 + l2 - 1e-6)
    qk = math.acos(max(-1.0, min(1.0, (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2))))
    gamma = math.atan2(-x_foot, length)
    qh = gamma - math.atan2(l2 * math.sin(qk), l1 + l2 * math.cos(qk))
    return qh, qk


def leg_fk(b: LegBody, qh, qk):
    """Axle position (x, z) relative to the hip in the torso frame."""
    x = -b.thigh * math.sin(qh) - b.shank * math.sin(qh + qk)
    z = -b.thigh * math.cos(qh) - b.shank * math.cos(qh + qk)
    return x, z


def build_xml(b: LegBody, motor: Motor, terrain=True, timestep=0.0005) -> str:
    r, hy = b.wheel_radius, b.hip_width / 2
    wy = (b.track - b.hip_width) / 2             # wheel outboard of the shank
    ghost = 'contype="0" conaffinity="0"'
    th = b.torso_height

    def leg(side, s):
        return f"""
      <body name="thigh_{side}" pos="0 {s * hy:.4f} 0">
        <joint name="hip_{side}" type="hinge" axis="0 1 0" range="-100 100" armature="{b.joint_armature}"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-b.thigh}" size="0.012" {ghost} mass="{b.thigh_mass}" rgba=".3 .3 .35 1"/>
        <body name="shank_{side}" pos="0 0 {-b.thigh}">
          <joint name="knee_{side}" type="hinge" axis="0 1 0" range="0 165" armature="{b.joint_armature}"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-b.shank}" size="0.010" {ghost} mass="{b.shank_mass}" rgba=".45 .45 .5 1"/>
          <geom type="cylinder" fromto="0 0 {-b.shank} 0 {s * (wy - b.wheel_width / 2 - 0.002):.4f} {-b.shank}" size="0.0185"
                {ghost} mass="{b.wheel_motor_mass}" rgba=".6 .6 .62 1"/>
          <body name="wheel_{side}" pos="0 {s * wy:.4f} {-b.shank}">
            <joint name="wheel_{side}" type="hinge" axis="0 1 0" armature="{motor.armature:.6g}"
                   frictionloss="{motor.friction:.6g}"/>
            <geom name="wheel_{side}" type="cylinder" size="{r} {b.wheel_width / 2}" euler="90 0 0"
                  mass="{b.wheel_mass}" rgba=".15 .15 .15 1" friction="1 0.005 0.0001"/>
            <geom type="box" size="{r * 0.8} 0.001 {r * 0.12}" pos="0 {s * (b.wheel_width / 2 + 0.0005):.4f} 0"
                  {ghost} mass="0" rgba=".9 .6 .1 1"/>
          </body>
        </body>
      </body>"""

    servo = lambda j: (f'<position name="{j}" joint="{j}" kp="{b.joint_kp}" kv="{b.joint_kv}" '
                       f'forcerange="{-b.joint_torque} {b.joint_torque}" forcelimited="true"/>')
    # a 1.5 cm plank under the left wheel's path only, 0.5-1.0 m ahead, with rounded edges
    terrain_xml = ""
    if terrain:
        y0, y1, hgt = b.track / 2 - 0.05, b.track / 2 + 0.06, 0.015
        yc, hy_p = (y0 + y1) / 2, (y1 - y0) / 2
        terrain_xml = f"""
    <geom name="plank" type="box" pos="0.75 {yc:.3f} {hgt / 2}" size="0.25 {hy_p:.3f} {hgt / 2}" rgba=".8 .5 .2 1"/>
    <geom type="capsule" fromto="0.5 {y0:.3f} 0 0.5 {y1:.3f} 0" size="{hgt}" rgba=".8 .5 .2 1"/>
    <geom type="capsule" fromto="1.0 {y0:.3f} 0 1.0 {y1:.3f} 0" size="{hgt}" rgba=".8 .5 .2 1"/>"""
    return f"""
<mujoco model="wheel_leg">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".82 .84 .86" rgb2=".72 .75 .78" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="40 40"/>
  </asset>
  <worldbody>
    <light pos="0 -1 2" dir="0 0.5 -1"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="grid"/>{terrain_xml}
    <body name="torso" pos="0 0 {r + b.height_nominal}">
      <freejoint name="torso"/>
      <camera name="follow" mode="trackcom" pos="0.8 -0.8 0.35" xyaxes="0.707 0.707 0 -0.2 0.2 0.96"/>
      <geom name="torso" type="box" size="{b.torso_depth / 2} {b.torso_width / 2} {th / 2}" pos="0 0 {th / 2 - 0.02:.4f}"
            mass="{b.torso_mass}" rgba=".25 .45 .75 .6"/>
      <geom type="box" size="0.0175 0.035 0.0125" pos="0 0 {b.battery_height}" {ghost} mass="{b.battery_mass}" rgba=".85 .25 .2 1"/>
      <geom type="box" size="0.025 0.055 0.012" pos="0 0 {b.boards_height}" {ghost} mass="{b.boards_mass}" rgba=".1 .55 .3 1"/>
      <geom type="cylinder" fromto="0 {hy - 0.045:.4f} 0 0 {hy - 0.005:.4f} 0" size="0.022" {ghost}
            mass="{b.hip_motor_mass + b.knee_motor_mass}" rgba=".6 .6 .62 1"/>
      <geom type="cylinder" fromto="0 {-hy + 0.045:.4f} 0 0 {-hy + 0.005:.4f} 0" size="0.022" {ghost}
            mass="{b.hip_motor_mass + b.knee_motor_mass}" rgba=".6 .6 .62 1"/>
      <site name="imu" pos="0 0 {b.imu_height}" size="0.006" rgba="1 1 0 1"/>
      {leg("l", 1)}
      {leg("r", -1)}
    </body>
  </worldbody>
  <actuator>
    {servo("hip_l")}
    {servo("knee_l")}
    {servo("hip_r")}
    {servo("knee_r")}
    <motor name="wheel_l" joint="wheel_l"/>
    <motor name="wheel_r" joint="wheel_r"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
    <jointpos name="enc_l" joint="wheel_l"/>
    <jointpos name="enc_r" joint="wheel_r"/>
    <jointpos name="hip_l" joint="hip_l"/>
    <jointpos name="knee_l" joint="knee_l"/>
    <jointpos name="hip_r" joint="hip_r"/>
    <jointpos name="knee_r" joint="knee_r"/>
  </sensor>
</mujoco>"""


def set_pose(model, data, b: LegBody, length, pitch=0.0):
    """Torso upright (or pitched), both legs at length, wheels on the floor."""
    qh, qk = leg_ik(b, length)
    for side in ("l", "r"):
        data.qpos[model.joint(f"hip_{side}").qposadr[0]] = qh
        data.qpos[model.joint(f"knee_{side}").qposadr[0]] = qk
        data.ctrl[model.actuator(f"hip_{side}").id] = qh
        data.ctrl[model.actuator(f"knee_{side}").id] = qk
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([0.0, 1.0, 0.0]), pitch)
    data.qpos[0:3] = (0, 0, b.wheel_radius + length * math.cos(pitch))
    data.qpos[3:7] = q
    mujoco.mj_forward(model, data)


def plant_at(model, b: LegBody, length):
    """Planar balance model at a leg length: everything above the axles as one rigid body.
    Also returns the pitch at which that body's CoM sits over the axle."""
    data = mujoco.MjData(model)
    set_pose(model, data, b, length)
    wheels = {model.body("wheel_l").id, model.body("wheel_r").id}
    ids = [i for i in range(1, model.nbody) if i not in wheels]
    m = model.body_mass[ids]
    pos = data.xipos[ids]
    M = m.sum()
    com = (m[:, None] * pos).sum(0) / M
    axle = (data.xpos[model.body("wheel_l").id] + data.xpos[model.body("wheel_r").id]) / 2
    I_b = 0.0
    for i, mi, pi in zip(ids, m, pos):
        R = data.ximat[i].reshape(3, 3)
        I_b += (R @ np.diag(model.body_inertia[i]) @ R.T)[1, 1] + mi * ((pi[0] - com[0]) ** 2 + (pi[2] - com[2]) ** 2)
    wl = model.body("wheel_l").id
    p = dict(M=M, l=com[2] - axle[2], I_b=I_b, m_w=model.body_mass[wl], I_w=_inertia_about_y(model, wl),
             J_m=model.dof_armature[model.joint("wheel_l").dofadr[0]], r=b.wheel_radius)
    pitch_eq = -math.atan2(com[0] - axle[0], com[2] - axle[2])
    return p, pitch_eq
