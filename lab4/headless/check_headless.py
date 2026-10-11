#!/usr/bin/env python3
"""
check_headless.py — 无 ROS 环境下的自检。

跑法（本机）：
    /Users/114514ma/Desktop/机器人实验/vnav_lab0/.venv/bin/python \
        /Users/114514ma/Desktop/机器人实验/lab4/headless/check_headless.py

它验证 4 件事：
  A. 力旋量 <-> 桨转速 往返一致（F2W 与 F 互逆）
  B. 四元数 <-> 旋转矩阵 往返一致 + 已知角度
  C. 物理自检：给悬停转速，无人机应稳住高度
  D. ★ 关键：`plant_core.Plant` 与 `lab3/sim/quad_sim.QuadrotorPlant` 的**闭环轨迹逐位一致**
     （同一条圆形轨迹 + 同一个 lab3 几何控制器，跑 5 s @1 kHz，误差应在 1e-10 量级）

第 D 项是无头复现可信度的支点：它说明"替身植物"不是另一个模型，
而是与离线仿真里那个被控对象**同一个**模型。
"""

from __future__ import annotations

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LAB3_SIM = os.path.normpath(os.path.join(HERE, "..", "..", "lab3", "sim"))
sys.path.insert(0, HERE)
sys.path.insert(0, LAB3_SIM)

import plant_core  # noqa: E402
import quad_sim  # noqa: E402

GAINS = dict(kx=16.0, kv=8.0, kr=150.0, komega=15.0)
DT = 1e-3
T_END = 5.0

_ok = True


def check(name, cond, detail=""):
    global _ok
    mark = "PASS" if cond else "FAIL"
    if not cond:
        _ok = False
    print("  [%s] %-46s %s" % (mark, name, detail))


# ---------------------------------------------------------------------------
# A. F2W 往返
# ---------------------------------------------------------------------------
def test_allocation():
    print("\nA. 桨转速分配矩阵 F / F2W 往返")
    rng = np.random.default_rng(0)
    w = rng.uniform(-80, 80, 4)
    w_back = plant_core.rotors_from_wrench(plant_core.wrench_from_rotors(w))
    check("rotors_from_wrench(wrench_from_rotors(w)) == w",
          np.allclose(w, w_back, atol=1e-12), "max|Δ|=%.2e" % np.max(np.abs(w - w_back)))

    # 悬停：4 个桨给相同转速，力旋量应为 [m*g, 0, 0, 0]
    wr = plant_core.wrench_from_rotors(np.full(4, plant_core.W_HOVER))
    check("悬停转速 -> 力旋量 = [m*g, 0, 0, 0]",
          abs(wr[0] - plant_core.MASS * plant_core.G) < 1e-9
          and np.max(np.abs(wr[1:])) < 1e-9,
          "f_z=%.6f (期望 %.6f), |tau|=%.2e"
          % (wr[0], plant_core.MASS * plant_core.G, np.max(np.abs(wr[1:]))))
    check("W_HOVER ≈ 49.52 rad/s", abs(plant_core.W_HOVER - 49.5225) < 1e-3,
          "W_HOVER=%.4f" % plant_core.W_HOVER)


# ---------------------------------------------------------------------------
# B. 四元数
# ---------------------------------------------------------------------------
def test_quaternion():
    print("\nB. 四元数 <-> 旋转矩阵")
    q = plant_core.mat_to_quat(np.eye(3))
    check("R=I -> q=(0,0,0,1)", np.allclose(q, [0, 0, 0, 1], atol=1e-12), "q=%s" % np.round(q, 12))

    th = 0.7
    Rz = plant_core.expm_so3([0, 0, th])
    q = plant_core.mat_to_quat(Rz)
    check("R=Rz(0.7) -> q=(0,0,sin(θ/2),cos(θ/2))",
          np.allclose(q, [0, 0, np.sin(th / 2), np.cos(th / 2)], atol=1e-12),
          "q=%s" % np.round(q, 9))

    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(200):
        R = plant_core.expm_so3(rng.normal(size=3) * 1.5)
        R2 = plant_core.quat_to_mat(plant_core.mat_to_quat(R))
        worst = max(worst, float(np.max(np.abs(R - R2))))
    check("随机 200 个 R: mat->quat->mat 往返", worst < 1e-12, "max|ΔR|=%.2e" % worst)

    worst = 0.0
    for _ in range(200):
        R = plant_core.expm_so3(rng.normal(size=3) * 1.5)
        roll, pitch, _ = plant_core.euler_zyx(R)
        # 用 R 的第三列能反推 roll/pitch，做一次一致性强校验
        worst = max(worst, abs(np.cos(pitch) * np.cos(roll) - R[2, 2]))
    check("euler_zyx 与 R 自洽 (cos p cos r == R22)", worst < 1e-9, "max|Δ|%.2e" % worst)


# ---------------------------------------------------------------------------
# C. 悬停物理自检
# ---------------------------------------------------------------------------
def test_hover():
    print("\nC. 物理自检")

    # C1 — 起始就在悬停转速：推力严格等于重力，高度必须一动不动。
    p = plant_core.Plant(motor_tau=0.03, pos0=np.array([0.0, 0.0, 2.0]),
                         w0=np.full(4, plant_core.W_HOVER))
    p.set_command(np.full(4, plant_core.W_HOVER))
    for _ in range(3000):          # 3 s @1 kHz
        p.step(1e-3)
    dz = float(p.pos[2] - 2.0)
    check("C1 悬停转速起跳：3 s 后 Δz < 1e-9 m", abs(dz) < 1e-9, "Δz=%.3e m" % dz)
    check("C1 悬停转速起跳：3 s 后 |v| < 1e-9 m/s",
          float(np.linalg.norm(p.vel)) < 1e-9,
          "|v|=%.3e m/s" % float(np.linalg.norm(p.vel)))

    # C2 — 电机滞后有**闭式解**，用解析公式钉死 tau。
    #   w(t) = w_h (1 - e^{-t/tau})            =>  F(t) = m g (1 - e^{-t/tau})^2
    #   a_z(t) = -g [ 2 e^{-t/tau} - e^{-2t/tau} ]
    #   v_z(inf) = -g [ 2 tau - tau/2 ] = -1.5 g tau      （开环恒给悬停转速时的净下沉率）
    tau = 0.03
    p2 = plant_core.Plant(motor_tau=tau, w0=np.zeros(4))
    p2.set_command(np.full(4, plant_core.W_HOVER))
    for _ in range(1000):          # 1 s（≫ 33 τ，瞬态已完全衰减）
        p2.step(1e-3)
    v_expect = -1.5 * plant_core.G * tau
    check("C2 阶跃到悬停 → v∞ = -1.5 g τ（解析解）",
          abs(float(p2.vel[2]) - v_expect) < 1e-4,
          "v=%.6f  期望 %.6f" % (float(p2.vel[2]), v_expect))

    # C3 — 一阶滞后的时间常数本身：1 tau 后应到 63.2%
    p3 = plant_core.Plant(motor_tau=tau, w0=np.zeros(4))
    p3.set_command(np.full(4, 100.0))
    for _ in range(30):            # 30 ms = 1 tau
        p3.step(1e-3)
    frac = float(p3.w[0] / 100.0)
    check("C3 一阶滞后 1τ 后到 ~63.2%", abs(frac - (1 - np.exp(-1))) < 5e-3,
          "%.4f (期望 %.4f)" % (frac, 1 - np.exp(-1)))


# ---------------------------------------------------------------------------
# D. ★ 与 lab3 quad_sim 的闭环等价性
# ---------------------------------------------------------------------------
def test_equivalence_with_quad_sim():
    print("\nD. ★ 与 lab3/sim/quad_sim.py 闭环轨迹逐位一致")
    print("     （圆形轨迹 + lab3 几何控制器，%.0f s @ %d Hz）" % (T_END, int(1 / DT)))

    sol = quad_sim.simulate(GAINS["kx"], GAINS["kv"], GAINS["kr"], GAINS["komega"],
                            t_end=T_END, dt=DT, controller_hz=1000.0)

    traj = quad_sim.CircleTrajectory()
    ctrl = quad_sim.GeometricController(**GAINS)
    plant = plant_core.Plant(motor_tau=0.03, w0=np.zeros(4))

    n = int(round(T_END / DT))
    err = dict(pos=0.0, vel=0.0, R=0.0, omega=0.0, w=0.0, w_cmd=0.0)
    for i in range(n + 1):
        xd, vd, ad, yawd = traj(i * DT)
        wrench, _Rd, _ex, _ev, _er = ctrl.compute(
            plant.pos, plant.vel, plant.R, plant.omega, xd, vd, ad, yawd
        )
        plant.set_command(plant_core.rotors_from_wrench(wrench))

        err["pos"] = max(err["pos"], float(np.max(np.abs(plant.pos - sol["pos"][i]))))
        err["vel"] = max(err["vel"], float(np.max(np.abs(plant.vel - sol["vel"][i]))))
        err["R"] = max(err["R"], float(np.max(np.abs(plant.R - sol["R"][i]))))
        err["omega"] = max(err["omega"], float(np.max(np.abs(plant.omega - sol["omega"][i]))))
        err["w"] = max(err["w"], float(np.max(np.abs(plant.w - sol["w"][i]))))
        err["w_cmd"] = max(err["w_cmd"], float(np.max(np.abs(plant.w_cmd - sol["w_cmd"][i]))))

        if i < n:
            plant.step(DT)

    for k in ("pos", "vel", "R", "omega", "w", "w_cmd"):
        tol = 1e-9 if k in ("w", "w_cmd") else 1e-10
        check("max|Δ%s| < %g" % (k, tol), err[k] < tol, "%.3e" % err[k])

    print("     轨迹终点: plant_core=(%.6f, %.6f, %.6f)  quad_sim=(%.6f, %.6f, %.6f)"
          % (plant.pos[0], plant.pos[1], plant.pos[2],
             sol["pos"][-1][0], sol["pos"][-1][1], sol["pos"][-1][2]))


# ---------------------------------------------------------------------------
# E. 坐标系自检（世界系角速度）
# ---------------------------------------------------------------------------
def test_frames():
    print("\nE. 坐标系自检：omega_world = R @ omega_body")
    p = plant_core.Plant()
    p.R = plant_core.expm_so3([0.3, -0.2, 1.1])
    p.omega = np.array([0.11, -0.23, 0.37])
    ow = p.omega_world()
    check("omega_world == R @ omega_body",
          np.allclose(ow, p.R @ p.omega, atol=1e-15), "ok")
    # 控制器会做 R^T * omega_world，必须能还原机体角速度
    back = p.R.T @ ow
    check("R^T @ omega_world 还原 body", np.allclose(back, p.omega, atol=1e-15),
          "max|Δ|=%.2e" % float(np.max(np.abs(back - p.omega))))


def main():
    print("=" * 78)
    print("LAB4 headless kit — 本地自检（无需 ROS）")
    print("=" * 78)
    test_allocation()
    test_quaternion()
    test_hover()
    test_equivalence_with_quad_sim()
    test_frames()
    print("\n" + "=" * 78)
    print("全部通过" if _ok else "存在失败项（见上）")
    print("=" * 78)
    return 0 if _ok else 1


if __name__ == "__main__":
    sys.exit(main())
