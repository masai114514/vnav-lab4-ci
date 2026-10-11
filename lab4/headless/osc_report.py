#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
osc_report.py —— 判「xyz 到底是不是在震荡」，并给出可比的量化口径

为什么单独一个脚本
------------------
竞速跑砸时的症状是「无人机斜着飞 / 翻着飞」，但这句话本身不可比。
要能判断「修好了没有」，必须落成几个能对齐基线的数：

  1. 位置误差**逐轴**的 RMS / max / 过零率 / 主频
  2. 误差的 1 s 滑动 RMS 包络（按 5 s 分档）—— 用来区分
        · **发散振荡**：包络一路涨上去
        · **稳定滞后**：包络持平
     这两者在「总 RMSE」这一个数上是分不开的。
  3. roll / pitch 自身的统计与同样口径的包络
     —— 用来区分「只是倾角指令给大了」和「姿态环本身坏了」

已归档的基线（官方 Unity 链路第 4 轮，位置 RMSE 9195 mm、12 门过 5 门）：

    | 轴 | 误差 RMS | max | 过零率 | 主频 |
    | x  |  6614.6 mm | 21090.4 mm | 0.19 Hz | 0.05 Hz |
    | y  |  4873.9 mm | 11852.8 mm | 0.19 Hz | 0.05 Hz |
    | z  |  1751.0 mm |  6417.5 mm | 0.36 Hz | 0.45 Hz |

    roll  : 均值 23.43°  RMS 95.00°  max 180.00°   过零 0.86 Hz  主频 0.30 Hz
    pitch : 均值 −7.46°  RMS 38.70°  max  88.38°   过零 0.84 Hz  主频 0.05 Hz

    roll 头 5 秒滑动 RMS 就 83.31°，全程 60–115° 从未稳定 → 姿态环从第 1 秒起就是坏的。

时间对齐（与 analyze_race_bag.py 同一套口径）
--------------------------------------------
`/desired_state`（MultiDOFJointTrajectoryPoint）**没有 header**，只有 `time_from_start`；
`/current_state`（Odometry）的 `header.stamp` 是仿真时间。于是：

    t0 = 第一条 /desired_state 的 bag 时刻 − 它的 time_from_start
    τ  = 某条 /current_state 的 header.stamp − t0

用法
----
    # 本机（无 ROS，用纯 python 的 rosbags）
    PY=/Users/114514ma/.workbuddy/binaries/python/envs/default/bin/python
    $PY osc_report.py --bag lab4/race_out/race.bag

    # 容器里（有 rosbag）
    python3 osc_report.py --bag /work/out/race.bag --out osc.txt --csv osc.csv

读 bag 用哪个库
---------------
优先 `rosbag`（ROS 自带，容器里有）；没有就退到 `rosbags`（纯 python，本机 venv 里有）。
两个后端读出来的字段完全一致，所以同一份脚本两边跑结果可比。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------
# bag 读取：两个后端
# ---------------------------------------------------------------------------
def _quat_to_rpy(x, y, z, w):
    """四元数 -> (roll, pitch, yaw)，单位弧度。"""
    # 归一化，防止数值漂移
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n > 1e-12:
        x, y, z, w = x / n, y / n, z / n, w / n
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def _dur_sec(d):
    """builtin_interfaces/Duration 兼容取秒（rosbags 的版本没有 to_sec）。"""
    if d is None:
        return 0.0
    if hasattr(d, "to_sec"):
        return float(d.to_sec())
    return float(getattr(d, "sec", 0)) + float(getattr(d, "nanosec", 0)) * 1e-9


def _read_with_rosbag(bag_path):
    import rosbag  # type: ignore

    des_t, des_p = [], []
    cur_stamp, cur_p, cur_rpy = [], [], []
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
            else:
                n_cur += 1
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                cur_stamp.append(msg.header.stamp.to_sec())
                cur_p.append((p.x, p.y, p.z))
                cur_rpy.append(_quat_to_rpy(q.x, q.y, q.z, q.w))

    return dict(n_des=n_des, n_cur=n_cur, t0=t0,
                des_t=np.asarray(des_t, float),
                des_p=np.asarray(des_p, float).reshape(-1, 3),
                cur_stamp=np.asarray(cur_stamp, float),
                cur_p=np.asarray(cur_p, float).reshape(-1, 3),
                cur_rpy=np.asarray(cur_rpy, float).reshape(-1, 2))


def _read_with_rosbags(bag_path):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    from pathlib import Path

    typestore = get_typestore(Stores.ROS1_NOETIC)

    des_t, des_p = [], []
    cur_stamp, cur_p, cur_rpy = [], [], []
    n_des = n_cur = 0
    t0 = None

    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ("/desired_state", "/current_state")]
        if not conns:
            raise RuntimeError("bag 里没有 /desired_state 或 /current_state")
        for conn, stamp_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == "/desired_state":
                n_des += 1
                tt = _dur_sec(msg.time_from_start)
                if t0 is None:
                    t0 = stamp_ns * 1e-9 - tt
                if len(msg.transforms) == 0:
                    continue
                tr = msg.transforms[0].translation
                des_t.append(tt)
                des_p.append((tr.x, tr.y, tr.z))
            else:
                n_cur += 1
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                cur_stamp.append(float(msg.header.stamp.sec)
                                 + float(msg.header.stamp.nanosec) * 1e-9)
                cur_p.append((p.x, p.y, p.z))
                cur_rpy.append(_quat_to_rpy(q.x, q.y, q.z, q.w))

    return dict(n_des=n_des, n_cur=n_cur, t0=t0,
                des_t=np.asarray(des_t, float),
                des_p=np.asarray(des_p, float).reshape(-1, 3),
                cur_stamp=np.asarray(cur_stamp, float),
                cur_p=np.asarray(cur_p, float).reshape(-1, 3),
                cur_rpy=np.asarray(cur_rpy, float).reshape(-1, 2))


def read_bag(bag_path):
    """优先 rosbag（ROS 版），退到 rosbags（纯 python 版）。"""
    try:
        return _read_with_rosbag(bag_path)
    except ImportError:
        pass
    return _read_with_rosbags(bag_path)


# ---------------------------------------------------------------------------
# 信号统计
# ---------------------------------------------------------------------------
def dominant_freq(sig, t, lo=0.05, hi=3.0, step=0.05):
    """在 [lo, hi] 上朴素扫频，返回能量最大的频率 [Hz]。信号先去掉均值。"""
    if len(sig) < 8:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return float("nan")
    n = len(sig)
    tu = t[0] + dt * np.arange(n)
    su = np.interp(tu, t, sig)
    su = su - su.mean()
    fs = [lo + step * k for k in range(int(round((hi - lo) / step)) + 1)]
    best_f, best_e = float("nan"), -1.0
    ph = 2j * np.pi * (tu - tu[0])
    for f in fs:
        e = abs(np.sum(su * np.exp(-f * ph))) ** 2
        if e > best_e:
            best_e, best_f = e, f
    return float(best_f)


def zero_cross_rate(sig, t):
    """过零率 [Hz]：s 内穿越 0 的次数 / 时长。"""
    if len(sig) < 3:
        return float("nan")
    span = float(t[-1] - t[0])
    if span <= 0:
        return float("nan")
    n = int(np.sum(np.diff(np.sign(sig)) != 0))
    return n / span


def sliding_rms(sig, t, win=1.0):
    """1 s 滑动窗的 RMS，返回与 sig 等长的数组。"""
    out = np.full(len(sig), np.nan)
    if len(sig) == 0:
        return out
    j0 = 0
    for i in range(len(sig)):
        while t[i] - t[j0] > win:
            j0 += 1
        seg = sig[j0:i + 1]
        if len(seg):
            out[i] = float(np.sqrt(np.mean(seg ** 2)))
    return out


def buckets(t, env, step=5.0):
    """把滑动 RMS 按 step 秒分档，返回 [(起, 止, 均值)]。"""
    if len(t) == 0:
        return []
    t0 = t[0]
    acc = {}
    for ti, v in zip(t, env):
        if not np.isfinite(v):
            continue
        k = int((ti - t0) // step) * int(step)
        acc.setdefault(k, []).append(v)
    return [(k, k + int(step), float(np.mean(v))) for k, v in sorted(acc.items())]


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
BASELINE_R4 = {
    "pos_rms_mm": (6614.6, 4873.9, 1751.0),
    "pos_max_mm": (21090.4, 11852.8, 6417.5),
    "roll_rms_deg": 95.00,
    "roll_max_deg": 180.00,
    "pitch_rms_deg": 38.70,
    "pos_rmse_mm": 9195.51,
}


def main():
    ap = argparse.ArgumentParser(description="xyz 振荡量化报告")
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", default="", help="报告落盘路径（默认与 bag 同目录 osc_report.txt）")
    ap.add_argument("--csv", default="", help="逐条样本 CSV（默认与 bag 同目录 osc_samples.csv）")
    ap.add_argument("--no-baseline", action="store_true", help="不打印 r4 基线对照")
    args = ap.parse_args()

    if not os.path.isfile(args.bag):
        print("找不到 bag: %s" % args.bag, file=sys.stderr)
        return 2

    d = read_bag(args.bag)
    lines = []

    def w(s=""):
        lines.append(s)

    w("=" * 78)
    w(" 无人机 xyz 振荡报告（osc_report.py）")
    w("=" * 78)
    w(" bag                : %s" % args.bag)
    w(" /desired_state 条数: %d" % d["n_des"])
    w(" /current_state 条数: %d" % d["n_cur"])

    if d["t0"] is None or len(d["des_p"]) < 2 or len(d["cur_p"]) < 2:
        w()
        w(" !! bag 内容不足（缺 /desired_state 或 /current_state），无法分析")
        print("\n".join(lines))
        return 1

    tau = d["cur_stamp"] - d["t0"]
    des_t, des_p = d["des_t"], d["des_p"]
    order = np.argsort(des_t)
    des_t, des_p = des_t[order], des_p[order]

    m = (tau >= 0.0) & (tau <= des_t.max())
    if not np.any(m):
        w()
        w(" !! 实际状态与期望轨迹没有时间重叠")
        print("\n".join(lines))
        return 1

    tau_m = tau[m]
    pos_m = d["cur_p"][m]
    rpy_m = d["cur_rpy"][m]
    des_i = np.column_stack([np.interp(tau_m, des_t, des_p[:, i]) for i in range(3)])
    err = pos_m - des_i                       # (N,3) mm 量级用 m
    err_mm = err * 1e3                        # 转 mm
    pos_rmse = float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))) * 1e3)

    w(" 轨迹总时长         : %.2f s" % des_t.max())
    w(" 参与分析样本       : %d 条（τ ∈ [%.2f, %.2f] s）"
      % (len(tau_m), tau_m[0], tau_m[-1]))
    w(" 位置 RMSE（总）    : %.2f mm" % pos_rmse)

    # ---- 一、位置误差逐轴 --------------------------------------------------
    w()
    w("一、位置误差逐轴")
    w("    %-4s %12s %12s %10s %9s" % ("轴", "RMS[mm]", "max|e|[mm]", "过零率[Hz]", "主频[Hz]"))
    for i, ax in enumerate("xyz"):
        s = err_mm[:, i]
        w("    %-4s %12.1f %12.1f %10.2f %9.2f"
          % (ax, np.sqrt(np.mean(s ** 2)), np.max(np.abs(s)),
             zero_cross_rate(s, tau_m), dominant_freq(s, tau_m)))
    w("    注：过零率 = 误差信号穿越 0 的次数 / 时长；主频 = 去均值后在 0.05–3 Hz 扫频的峰值。")

    # ---- 二、误差包络：发散 vs 稳定滞后 -----------------------------------
    w()
    w("二、误差 1 s 滑动 RMS 包络（按 5 s 分档）[mm]")
    w("    判读：包络**单调涨** = 发散振荡；**持平** = 稳定滞后。")
    w("    %10s %10s %10s %10s" % ("窗口[s]", "x", "y", "z"))
    env = [sliding_rms(err_mm[:, i], tau_m, 1.0) for i in range(3)]
    b = [buckets(tau_m, env[i], 5.0) for i in range(3)]
    for k in range(max(len(x) for x in b)):
        row = []
        for i in range(3):
            row.append("%10.0f" % b[i][k][2] if k < len(b[i]) else "%10s" % "-")
        label = "%4d-%3d" % (b[0][k][0], b[0][k][1]) if k < len(b[0]) else ""
        w("    %10s %s" % (label, " ".join(row)))
    # 趋势
    # 只用「首档 vs 末档」的比值会被近零的首档放大（实测好那趟的首档只有 5 mm，
    # 末档 109 mm → ×21.8 的"涨"，可两者都是正常的亚分米级）。
    # 所以改成：首尾各取三分之一档的**均值**，并且要求绝对增量也过得去才叫"涨"。
    w("    趋势（首 1/3 档均值 → 末 1/3 档均值）:")
    for i, ax in enumerate("xyz"):
        n = len(b[i])
        if n < 3:
            continue
        k = max(1, n // 3)
        first = float(np.mean([x[2] for x in b[i][:k]]))
        last = float(np.mean([x[2] for x in b[i][-k:]]))
        delta = last - first
        if first > 1e-9:
            ratio = last / first
        else:
            ratio = float("inf")
        if ratio > 1.6 and delta > 150.0:
            tag = "← 单调涨（发散特征）"
        elif ratio < 0.62 and delta < -150.0:
            tag = "← 降"
        else:
            tag = "← 基本持平（稳定滞后特征）"
        w("      %s: %.0f → %.0f mm（Δ=%+.0f, ×%.2f）  %s"
          % (ax, first, last, delta, ratio, tag))
    w("    阈值说明：Δ 必须超过 150 mm 且比值 > 1.6 才判「涨」——")
    w("              亚分米级的相对放大没有物理意义（好那趟首档本就只有几毫米）。")

    # ---- 三、姿态自身 ------------------------------------------------------
    w()
    w("三、姿态（roll / pitch）")
    w("    %-6s %10s %10s %10s %10s %9s" % ("姿态", "均值[°]", "RMS[°]", "max[°]", "过零率[Hz]", "主频[Hz]"))
    rpy_deg = np.degrees(rpy_m)
    for i, nm in enumerate(("roll", "pitch")):
        s = rpy_deg[:, i]
        w("    %-6s %10.2f %10.2f %10.2f %10.2f %9.2f"
          % (nm, np.mean(s), np.sqrt(np.mean(s ** 2)), np.max(np.abs(s)),
             zero_cross_rate(s, tau_m), dominant_freq(s, tau_m)))

    w()
    w("三b、姿态 1 s 滑动 RMS 包络（按 5 s 分档）[°]")
    w("    %10s %10s %10s" % ("窗口[s]", "roll", "pitch"))
    envA = [sliding_rms(rpy_deg[:, i], tau_m, 1.0) for i in range(2)]
    bA = [buckets(tau_m, envA[i], 5.0) for i in range(2)]
    for k in range(max(len(x) for x in bA)):
        row = []
        for i in range(2):
            row.append("%10.1f" % bA[i][k][2] if k < len(bA[i]) else "%10s" % "-")
        label = "%4d-%3d" % (bA[0][k][0], bA[0][k][1]) if k < len(bA[0]) else ""
        w("    %10s %s" % (label, " ".join(row)))
    if bA[0]:
        w("    roll 首档 %.1f° —— %s"
          % (bA[0][0][2],
             "头 5 秒就已经很大，姿态环从一开始就没稳"
             if bA[0][0][2] > 30 else "头 5 秒还算正常"))

    # ---- 四、与基线对照 ----------------------------------------------------
    if not args.no_baseline:
        w()
        w("四、与 r4 基线对照（官方 Unity 链路，发散那趟）")
        w("    %-10s %12s %12s %10s" % ("量", "r4 基线", "本次", "结论"))
        bref = BASELINE_R4["pos_rms_mm"]
        for i, ax in enumerate("xyz"):
            cur = float(np.sqrt(np.mean(err_mm[:, i] ** 2)))
            w("    %-10s %12.1f %12.1f %10s"
              % ("误差RMS %s" % ax, bref[i], cur,
                 "改善" if cur < bref[i] * 0.7 else ("恶化" if cur > bref[i] * 1.4 else "同量级")))
        w("    %-10s %12.1f %12.1f %10s"
          % ("位置 RMSE", BASELINE_R4["pos_rmse_mm"], pos_rmse,
             "改善" if pos_rmse < BASELINE_R4["pos_rmse_mm"] * 0.7
             else ("恶化" if pos_rmse > BASELINE_R4["pos_rmse_mm"] * 1.4 else "同量级")))
        rr = float(np.sqrt(np.mean(rpy_deg[:, 0] ** 2)))
        w("    %-10s %12.2f %12.2f %10s"
          % ("roll RMS", BASELINE_R4["roll_rms_deg"], rr,
             "改善" if rr < BASELINE_R4["roll_rms_deg"] * 0.7
             else ("恶化" if rr > BASELINE_R4["roll_rms_deg"] * 1.4 else "同量级")))
        w("    判读：位置 RMSE 与 roll RMS **同时**明显下降，才算真修好；")
        w("          只降一个，说明另一环还有问题。")

    # ---- 五、一句话结论 ----------------------------------------------------
    w()
    w("五、结论")
    verdict = []
    if pos_rmse > 3000:
        verdict.append("位置仍是发散量级（RMSE %.0f mm）" % pos_rmse)
    elif pos_rmse > 1500:
        verdict.append("位置在跟轨迹但明显差于离线基线（RMSE %.0f mm）" % pos_rmse)
    else:
        verdict.append("位置已与离线基线同量级（RMSE %.0f mm）" % pos_rmse)
    rr = float(np.sqrt(np.mean(rpy_deg[:, 0] ** 2)))
    if rr > 45:
        verdict.append("roll RMS %.0f° —— 姿态环仍在翻覆" % rr)
    elif rr > 20:
        verdict.append("roll RMS %.0f° —— 姿态偏大但未翻覆" % rr)
    else:
        verdict.append("roll RMS %.0f° —— 姿态基本受控" % rr)
    for v in verdict:
        w("    · %s" % v)

    txt = "\n".join(lines)
    print(txt)

    base = os.path.dirname(os.path.abspath(args.bag))
    out_path = args.out or os.path.join(base, "osc_report.txt")
    csv_path = args.csv or os.path.join(base, "osc_samples.csv")
    with open(out_path, "w") as f:
        f.write(txt + "\n")
    with open(csv_path, "w", newline="") as f:
        cw = csv.writer(f)
        cw.writerow(["tau_s", "err_x_mm", "err_y_mm", "err_z_mm",
                     "roll_deg", "pitch_deg", "pos_x", "pos_y", "pos_z"])
        for i in range(len(tau_m)):
            cw.writerow(["%.6f" % tau_m[i]]
                        + ["%.3f" % v for v in err_mm[i]]
                        + ["%.4f" % v for v in rpy_deg[i]]
                        + ["%.5f" % v for v in pos_m[i]])
    print("\n[osc] 报告 → %s" % out_path)
    print("[osc] 样本 → %s" % csv_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
