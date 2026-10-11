"""
plant_core.py — 纯 numpy 的四旋翼被控对象（**无 ROS 依赖**）。

用途
----
LAB4 官方链路的闭环是：

    trajectory_generation_node --> /desired_state --> controller_node
                                                            |
                                                     /rotor_speed_cmds
                                                            v
                                                 [ Unity(TESSE) 仿真 ]   <-- 唯一需要 GUI 的一环
                                                            |
                                                      /current_state

录 rosbag 只需要 `/current_state` 与 `/desired_state` 两个**话题**，本身不需要任何显示器。
把上面那个方框换成这个 `Plant`，闭环就能在纯命令行下成立 —— 这就是 "无头复现" 的全部原理。

模型一致性
----------
本文件的 F 矩阵、桨位、反扭矩符号、一阶电机滞后（tau=0.03）、RK4 + Rodrigues 旋转更新，
与 `controller_pkg/src/controller_node.cpp` 的 F2W 构造、以及 `lab3/sim/quad_sim.py` 的
`QuadrotorPlant` **逐行等价**；`check_headless.py` 会做数值交叉验证（应吻合到 ~1e-12）。
"""

from __future__ import annotations

import math

import numpy as np

# ---------------------------------------------------------------------------
# 物理常数（与 controller_node.cpp 构造器一致）
# ---------------------------------------------------------------------------
MASS = 1.0        # m   [kg]
G = 9.81          # g   [m/s^2]
ARM = 0.3         # d   [m]   质心 -> 桨
CF = 1e-3         # c_f 升力系数： f = c_f * w|w|
CD = 1e-5         # c_d 反扭矩系数：tau_z = (+,-,+,-) * c_d * w|w|
J = np.eye(3)     # 惯量矩阵 [kg m^2]
E3 = np.array([0.0, 0.0, 1.0])

_S = 1.0 / math.sqrt(2.0)

# ---------------------------------------------------------------------------
# F : 桨的"带符号平方" -> 力旋量 [ f_z, tau_x, tau_y, tau_z ]^T
#   桨位（俯视）  rho_1 = +d/√2 ( 1, 1)   rho_2 = +d/√2 (-1, 1)
#                rho_3 = +d/√2 (-1,-1)   rho_4 = +d/√2 ( 1,-1)
#   推力沿 +e3，大小 c_f * w|w|；反扭矩绕 e3，符号 (+,-,+,-)
# ---------------------------------------------------------------------------
F_MATRIX = np.array(
    [
        [CF, CF, CF, CF],
        [CF * ARM * _S, CF * ARM * _S, -CF * ARM * _S, -CF * ARM * _S],
        [-CF * ARM * _S, CF * ARM * _S, CF * ARM * _S, -CF * ARM * _S],
        [CD, -CD, CD, -CD],
    ]
)
F2W = np.linalg.inv(F_MATRIX)

# 悬停所需转速（单桨）： 4 * c_f * w^2 = m * g  ->  w = sqrt(m*g/(4*c_f)) ≈ 49.52 rad/s
W_HOVER = math.sqrt(MASS * G / (4.0 * CF))


def wrench_from_rotors(w):
    """桨转速 [rad/s] -> 力旋量 [f_z, tau_x, tau_y, tau_z]。"""
    w = np.asarray(w, float)
    return F_MATRIX @ (w * np.abs(w))


def rotors_from_wrench(wrench):
    """力旋量 -> 带符号的桨转速（`wrench_from_rotors` 的精确逆）。"""
    wsq = F2W @ np.asarray(wrench, float)
    return np.sign(wsq) * np.sqrt(np.abs(wsq))


# ---------------------------------------------------------------------------
# SO(3) 工具
# ---------------------------------------------------------------------------
def hat(w):
    w = np.asarray(w, float)
    return np.array(
        [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]
    )


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def expm_so3(w):
    """反对称向量的矩阵指数（Rodrigues 公式）。"""
    w = np.asarray(w, float)
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        return np.eye(3) + hat(w)
    k = w / th
    K = hat(k)
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def mat_to_quat(R):
    """旋转矩阵 -> 四元数 (x, y, z, w)（Shepperd 分支法，数值稳健）。"""
    R = np.asarray(R, float)
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], float)
    n = float(np.linalg.norm(q))
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def quat_to_mat(q):
    """四元数 (x, y, z, w) -> 旋转矩阵（与 mat_to_quat 互逆，供自检使用）。"""
    x, y, z, w = np.asarray(q, float)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def euler_zyx(R):
    """旋转矩阵 -> (roll, pitch, yaw)（ZYX 约定）。"""
    R = np.asarray(R, float)
    yaw = math.atan2(R[1, 0], R[0, 0])
    pitch = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    roll = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# 被控对象
# ---------------------------------------------------------------------------
class Plant:
    """四旋翼刚体动力学 + 一阶电机滞后。

    状态： pos(3) vel(3) R(3x3) omega(3, **机体系**) w(4, 桨转速 rad/s)
    输入： w_cmd(4) —— 即 `controller_node` 发到 `/rotor_speed_cmds` 的 `angular_velocities`

    `step(dt)` 的更新顺序与 `quad_sim.simulate()` 内层循环**完全一致**：
      先 RK4 更新 pos/vel/omega/w，再用**更新后**的 omega 做 Rodrigues 旋转更新。
    """

    def __init__(self, motor_tau=0.03, w_max=1e9, w0=None,
                 pos0=None, vel0=None, R0=None, omega0=None):
        self.motor_tau = float(motor_tau)
        self.w_max = float(w_max)
        self.w = np.zeros(4) if w0 is None else np.asarray(w0, float).reshape(4).copy()
        self.pos = np.zeros(3) if pos0 is None else np.asarray(pos0, float).copy()
        self.vel = np.zeros(3) if vel0 is None else np.asarray(vel0, float).copy()
        self.R = np.eye(3) if R0 is None else np.asarray(R0, float).copy()
        self.omega = np.zeros(3) if omega0 is None else np.asarray(omega0, float).copy()
        self.w_cmd = self.w.copy()
        self.t = 0.0

    # -- 输入 ---------------------------------------------------------------
    def set_command(self, w_cmd):
        """设置（限幅后的）桨转速指令。"""
        self.w_cmd = np.clip(np.asarray(w_cmd, float).reshape(4), -self.w_max, self.w_max)

    # -- 动力学 -------------------------------------------------------------
    def deriv(self, pos, vel, R, omega, w):
        """连续时间导数（与 quad_sim.QuadrotorPlant.deriv 一致）。"""
        wrench = wrench_from_rotors(w)
        f = wrench[0]
        M = wrench[1:]

        dpos = vel
        dvel = -G * E3 + (f / MASS) * (R @ E3)
        dR = R @ hat(omega)
        domega = np.linalg.inv(J) @ (M - np.cross(omega, J @ omega))
        dw = (self.w_cmd - w) / self.motor_tau
        return dpos, dvel, dR, domega, dw

    def step(self, dt):
        """推进 dt（RK4；旋转用 Rodrigues 精确更新）。"""
        p, v, R, om, w = self.pos, self.vel, self.R, self.omega, self.w

        dp1, dv1, _, dom1, dw1 = self.deriv(p, v, R, om, w)
        dp2, dv2, _, dom2, dw2 = self.deriv(
            p + 0.5 * dt * dp1, v + 0.5 * dt * dv1, R, om + 0.5 * dt * dom1, w + 0.5 * dt * dw1
        )
        dp3, dv3, _, dom3, dw3 = self.deriv(
            p + 0.5 * dt * dp2, v + 0.5 * dt * dv2, R, om + 0.5 * dt * dom2, w + 0.5 * dt * dw2
        )
        dp4, dv4, _, dom4, dw4 = self.deriv(
            p + dt * dp3, v + dt * dv3, R, om + dt * dom3, w + dt * dw3
        )

        self.pos = p + dt / 6.0 * (dp1 + 2 * dp2 + 2 * dp3 + dp4)
        self.vel = v + dt / 6.0 * (dv1 + 2 * dv2 + 2 * dv3 + dv4)
        self.omega = om + dt / 6.0 * (dom1 + 2 * dom2 + 2 * dom3 + dom4)
        self.w = w + dt / 6.0 * (dw1 + 2 * dw2 + 2 * dw3 + dw4)

        # 注意：用**更新后**的 omega（与 quad_sim.simulate 的行序一致）
        self.R = R @ expm_so3(dt * self.omega)

        self.t += dt
        return self

    # -- 输出 ---------------------------------------------------------------
    def omega_world(self):
        """世界系角速度。

        ⚠️ controller_node::onCurrentState 里做的是 `omega = R^T * omega_world`，
        也就是说它假定 `/current_state` 的 `twist.twist.angular` 是**世界系**的。
        发错坐标系会导致姿态环彻底跑飞 —— 这是无头复现最容易踩的坑。
        """
        return self.R @ self.omega

    def quaternion(self):
        return mat_to_quat(self.R)
