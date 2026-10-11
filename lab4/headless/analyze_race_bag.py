#!/usr/bin/env python3
"""
analyze_race_bag.py — 分析竞速 rosbag，输出指标并（可选）画图。

用法：
    python3 analyze_race_bag.py --bag ~/race_out/race.bag --out metrics.json --plot tracking.png

时间对齐
--------
`/desired_state`（MultiDOFJointTrajectoryPoint）**没有 header**，只有 `time_from_start`；
`/current_state`（Odometry）的 `header.stamp` 是仿真时间（由植物节点打）。
两者用同一个"轨迹时间轴"对齐：

    t0  = 第一条 /desired_state 的 bag 时刻 - 它的 time_from_start
    τ   = 某条 /current_state 的 header.stamp - t0     <-- 轨迹时间

然后就能把实际状态插值到任意 τ 上与期望比较。

对照基线（来自离线仿真 sim/lab4_sim.py，同一套代码）：
    · 期望状态 5 Hz（官方定时器口径）：位置 RMSE ≈ 316 mm
    · 期望状态连续            ：位置 RMSE ≈  13.7 mm
本脚本会把实测值与之并列打印 —— 如果 ROS 跑出来接近 316 mm，说明这条无头链路
确实复现了官方代码的行为（而不是碰巧跑通）。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# 门位姿（与 planner_pkg/src/traj_vertices_publisher.cpp 的 unity2RosPose 一致）
# ---------------------------------------------------------------------------
def unity2ros_pose(x, y, z, qx, qy, qz, qw):
    """Unity 系 -> ROS 系：位置 (x,z,y)，四元数 (-qx,-qz,-qy,qw)。"""
    return (x, z, y), (-qx, -qz, -qy, qw)


def load_gates(csv_path):
    """读取 vnav-2020-lab4_static_tfs.csv，返回按门号排序的 ROS 系门位置 (N,3)。"""
    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.reader(f):                      # 无表头！
            if len(row) < 8:
                continue
            name = row[0]
            if not name.startswith("red_square_drone_door"):
                continue
            # "red_square_drone_door" 或 "red_square_drone_door (7)"
            idx = 0
            if "(" in name:
                idx = int(name[name.index("(") + 1:name.index(")")])
            ux, uy, uz, qx, qy, qz, qw = (float(v) for v in row[1:8])
            p, _q = unity2ros_pose(ux, uy, uz, qx, qy, qz, qw)
            rows.append((idx, np.asarray(p, float)))
    rows.sort(key=lambda r: r[0])
    return np.array([r[1] for r in rows], float)


# ---------------------------------------------------------------------------
def _read_bag_rosbags(bag_path):
    """没有 ROS `rosbag` 时的回退后端（纯 pip 包 rosbags）。

    本机 macOS 没装 ROS，只有 CI 容器里有。两个后端的输出结构必须一致，
    否则同一份 bag 在两边画出的图会不一样。
    """
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    def _sec(d):
        # builtin_interfaces/msg/Duration 没有 to_sec()
        if hasattr(d, "to_sec"):
            return d.to_sec()
        return float(d.sec) + float(d.nanosec) * 1e-9

    typestore = get_typestore(Stores.ROS1_NOETIC)
    des_t, des_p, des_v = [], [], []
    cur_t, cur_p, cur_v, cur_q = [], [], [], []
    n_des = n_cur = 0
    t0 = None
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ("/desired_state", "/current_state")]
        for conn, stamp_ns, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == "/desired_state":
                n_des += 1
                tt = _sec(m.time_from_start)
                if t0 is None:
                    t0 = stamp_ns * 1e-9 - tt
                if not m.transforms:
                    continue
                tr = m.transforms[0].translation
                des_t.append(tt)
                des_p.append((tr.x, tr.y, tr.z))
                if m.velocities:
                    vl = m.velocities[0].linear
                    des_v.append((vl.x, vl.y, vl.z))
                else:
                    des_v.append((0.0, 0.0, 0.0))
            else:
                n_cur += 1
                p = m.pose.pose.position
                q = m.pose.pose.orientation
                v = m.twist.twist.linear
                cur_t.append(_sec(m.header.stamp))
                cur_p.append((p.x, p.y, p.z))
                cur_v.append((v.x, v.y, v.z))
                cur_q.append((q.x, q.y, q.z, q.w))

    out = dict(
        n_desired=n_des, n_current=n_cur, t0=t0,
        des_t=np.asarray(des_t, float),
        des_p=np.asarray(des_p, float).reshape(-1, 3),
        des_v=np.asarray(des_v, float).reshape(-1, 3),
        cur_t=np.asarray(cur_t, float),
        cur_p=np.asarray(cur_p, float).reshape(-1, 3),
        cur_v=np.asarray(cur_v, float).reshape(-1, 3),
        cur_q=np.asarray(cur_q, float).reshape(-1, 4),
    )
    order = np.argsort(out["des_t"])
    for k in ("des_t", "des_p", "des_v"):
        out[k] = out[k][order]
    out["cur_tau"] = out["cur_t"] - (t0 if t0 is not None else 0.0)
    return out


def read_bag(bag_path):
    try:
        import rosbag  # noqa: F401
    except ImportError:
        try:
            import rosbags  # noqa: F401
        except ImportError:
            print("错误：既没有 ROS 的 rosbag，也没有 pip 的 rosbags。"
                  "容器里 `source /opt/ros/noetic/setup.bash`，"
                  "本机 `pip install rosbags`。", file=sys.stderr)
            raise
        print("[analyze] 没有 ROS rosbag，改用 rosbags 后端读 bag", file=sys.stderr)
        return _read_bag_rosbags(bag_path)

    des_t, des_p, des_v = [], [], []
    cur_t, cur_p, cur_v, cur_q = [], [], [], []
    n_des = n_cur = 0
    t0 = None

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=["/desired_state", "/current_state"]):
            if topic == "/desired_state":
                n_des += 1
                tt = msg.time_from_start.to_sec()
                if t0 is None:
                    t0 = t.to_sec() - tt
                if len(msg.transforms) == 0:
                    continue
                tr = msg.transforms[0].translation
                des_t.append(tt)
                des_p.append((tr.x, tr.y, tr.z))
                if len(msg.velocities):
                    vl = msg.velocities[0].linear
                    des_v.append((vl.x, vl.y, vl.z))
                else:
                    des_v.append((0.0, 0.0, 0.0))
            else:
                n_cur += 1
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                cur_t.append(msg.header.stamp.to_sec())
                cur_p.append((p.x, p.y, p.z))
                cur_v.append((v.x, v.y, v.z))
                cur_q.append((q.x, q.y, q.z, q.w))

    out = dict(
        n_desired=n_des, n_current=n_cur, t0=t0,
        des_t=np.asarray(des_t, float),
        des_p=np.asarray(des_p, float).reshape(-1, 3),
        des_v=np.asarray(des_v, float).reshape(-1, 3),
        cur_t=np.asarray(cur_t, float),
        cur_p=np.asarray(cur_p, float).reshape(-1, 3),
        cur_v=np.asarray(cur_v, float).reshape(-1, 3),
    )
    if t0 is None or len(out["des_p"]) == 0 or len(out["cur_p"]) == 0:
        return out
    # 期望：按 τ 排序（time_from_start 单调，但保险起见排一下）
    order = np.argsort(out["des_t"])
    for k in ("des_t", "des_p", "des_v"):
        out[k] = out[k][order]
    # 实际：轨迹时间 τ = stamp - t0
    out["cur_tau"] = out["cur_t"] - t0
    return out


def metrics(data, gates):
    res = {
        "n_desired_msgs": int(data["n_desired"]),
        "n_current_msgs": int(data["n_current"]),
    }
    if data["t0"] is None or len(data["des_p"]) == 0 or len(data["cur_p"]) == 0:
        res["error"] = "bag 里缺少 /desired_state 或 /current_state，无法分析"
        return res

    dtau, dpos = data["des_t"], data["des_p"]
    tau = data["cur_tau"]
    res["trajectory_duration_s"] = float(dtau.max())
    res["sim_time_span_s"] = float(tau.max() - tau.min())

    m = (tau >= 0.0) & (tau <= dtau.max())
    if not np.any(m):
        res["error"] = "实际状态与期望轨迹没有时间重叠"
        return res
    tau_m, pos_m = tau[m], data["cur_p"][m]

    # 把期望插值到实际的时间点上
    des_i = np.column_stack([np.interp(tau_m, dtau, dpos[:, i]) for i in range(3)])
    err = np.linalg.norm(pos_m - des_i, axis=1)

    res.update(
        pos_rmse_mm=float(np.sqrt(np.mean(err ** 2)) * 1e3),
        pos_max_mm=float(np.max(err) * 1e3),
        pos_mean_mm=float(np.mean(err) * 1e3),
        max_speed_mps=float(np.max(np.linalg.norm(data["cur_v"][m], axis=1))),
        max_des_speed_mps=float(np.max(np.linalg.norm(data["des_v"], axis=1))),
        final_pos=[float(v) for v in pos_m[-1]],
    )

    # 过门统计：逐门找最近距离，且要求**按顺序**推进
    if gates is not None and len(gates):
        passed, details = 0, []
        start = 0
        for i, g in enumerate(gates):
            seg = np.linalg.norm(pos_m[start:] - g, axis=1)
            k = int(np.argmin(seg))
            dmin = float(seg[k])
            details.append(dict(gate=i, min_dist_m=round(dmin, 4),
                                tau_s=round(float(tau_m[start + k]), 3)))
            if dmin < 1.0:
                passed += 1
                start = start + k          # 只允许往前找，防止来回穿同一道门
        res["gates_total"] = int(len(gates))
        res["gates_passed"] = int(passed)
        res["gate_details"] = details

    return res


def plot(data, gates, path, title="LAB4 drone race — headless reproduction"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[analyze] 没装 matplotlib，跳过画图", file=sys.stderr)
        return False

    if data["t0"] is None or len(data["des_p"]) == 0 or len(data["cur_p"]) == 0:
        return False

    fig = plt.figure(figsize=(13, 5.2))

    # 俯视：赛道
    ax = fig.add_subplot(1, 3, 1)
    ax.plot(data["des_p"][:, 0], data["des_p"][:, 1], "-", lw=1.4,
            color="#1565c0", label="desired (min-snap)")
    ax.plot(data["cur_p"][:, 0], data["cur_p"][:, 1], "-", lw=1.1,
            color="#c62828", alpha=0.85, label="actual (sim)")
    if gates is not None and len(gates):
        ax.plot(gates[:, 0], gates[:, 1], "s", ms=6, mfc="none",
                mec="#2e7d32", mew=1.4, label="gates")
        for i, g in enumerate(gates, 1):
            ax.annotate(str(i), (g[0], g[1]), fontsize=7,
                        xytext=(3, 3), textcoords="offset points", color="#2e7d32")
    ax.plot(0, 0, "o", ms=6, color="k", label="start")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("course (top view)", fontsize=10)
    ax.legend(fontsize=7, loc="best")

    # 位置误差
    dtau, dpos = data["des_t"], data["des_p"]
    tau = data["cur_tau"]
    m = (tau >= 0.0) & (tau <= dtau.max())
    tau_m, pos_m = tau[m], data["cur_p"][m]
    des_i = np.column_stack([np.interp(tau_m, dtau, dpos[:, i]) for i in range(3)])
    err = np.linalg.norm(pos_m - des_i, axis=1)

    ax = fig.add_subplot(1, 3, 2)
    ax.plot(tau_m, err * 1e3, lw=1.1, color="#c62828")
    ax.axhline(np.sqrt(np.mean(err ** 2)) * 1e3, ls="--", lw=1.0, color="#455a64",
               label="RMSE = %.1f mm" % (np.sqrt(np.mean(err ** 2)) * 1e3))
    ax.set_xlabel("trajectory time [s]")
    ax.set_ylabel("position error [mm]")
    ax.set_title("tracking error", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # 速度
    ax = fig.add_subplot(1, 3, 3)
    ax.plot(tau_m, np.linalg.norm(data["cur_v"][m], axis=1), lw=1.1,
            color="#2e7d32", label="actual")
    ax.plot(dtau, np.linalg.norm(data["des_v"], axis=1), lw=1.0, ls="--",
            color="#1565c0", label="desired")
    ax.set_xlabel("trajectory time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_title("speed profile", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.suptitle(title, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=130)
    print("[analyze] 图已保存: %s" % path)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default="metrics.json")
    ap.add_argument("--plot", default="")
    ap.add_argument("--title", default="LAB4 drone race — headless reproduction")
    ap.add_argument("--gates-csv", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "sim_data", "StreamingAssets", "vnav-2020-lab4_static_tfs.csv"))
    args = ap.parse_args()

    gates = None
    if os.path.isfile(args.gates_csv):
        gates = load_gates(args.gates_csv)
        print("[analyze] 门位姿: %d 道（来自 %s）" % (len(gates), args.gates_csv))
    else:
        print("[analyze] 没找到门位姿 CSV，跳过过门统计", file=sys.stderr)

    data = read_bag(args.bag)
    res = metrics(data, gates)

    print("\n" + "=" * 74)
    print("竞速 rosbag 分析结果")
    print("=" * 74)
    if "error" in res:
        print("  !! %s" % res["error"])
    else:
        print("  消息数            : /desired_state %d 条, /current_state %d 条"
              % (res["n_desired_msgs"], res["n_current_msgs"]))
        print("  轨迹总时长        : %.2f s" % res["trajectory_duration_s"])
        print("  位置 RMSE         : %.2f mm" % res["pos_rmse_mm"])
        print("  位置最大误差      : %.2f mm" % res["pos_max_mm"])
        print("  最大速度          : %.2f m/s" % res["max_speed_mps"])
        if "gates_passed" in res:
            print("  过门              : %d / %d" % (res["gates_passed"], res["gates_total"]))
        print("\n  对照（离线仿真，同一套代码）:")
        print("    期望状态 5 Hz（官方定时器口径）→ 位置 RMSE ≈ 316 mm")
        print("    期望状态连续                  → 位置 RMSE ≈  13.7 mm")
        # 判定要和基线**同一个量级**才算复现。离线用合成植物节点（1000 Hz、
        # 进程内直连）跑出 517 mm；走 Unity/TESSE 实时链路时，闭环里多出
        # 渲染帧级的采样保持，几千 mm 以内都还算"在跟轨迹"，上万 mm 就是发散。
        # 早先这里只判 `>150 mm` 就打印"复现成功 ✓"，把 9195 mm 也判成了成功。
        rmse = res["pos_rmse_mm"]
        # ⚠️ 局部变量名**不要**叫 gates —— 会把外面那个门位姿数组覆盖成 int，
        # 于是第 313 行 plot(data, gates, ...) 拿到 int，在 `len(gates)` 处炸：
        #     TypeError: object of type 'int' has no len()
        # 症状很隐蔽：metrics.json 已经写好了，只有 race_tracking.png 静默消失。
        n_passed = res.get("gates_passed")
        if rmse <= 1500 and (n_passed is None or n_passed >= 10):
            verdict = "复现成功 ✓（与离线基线同量级）"
        elif rmse <= 3000:
            verdict = "勉强在跟轨迹，但明显差于离线基线（实时链路开销）"
        else:
            verdict = "★ 发散 —— 实测轨迹没跟上期望轨迹，需排查实时链路"
        print("  实测 %.1f mm —— %s" % (rmse, verdict))
        if n_passed is not None:
            print("  过门 %d/%d —— %s"
                  % (n_passed, res["gates_total"],
                     "完整一圈" if n_passed == res["gates_total"]
                     else "未跑完整圈（发散会撞门框/飞出场外）"))
    print("=" * 74)

    with open(args.out, "w") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    print("[analyze] 指标已写入: %s" % args.out)

    if args.plot:
        plot(data, gates, args.plot, args.title)
    return 0


if __name__ == "__main__":
    sys.exit(main())
