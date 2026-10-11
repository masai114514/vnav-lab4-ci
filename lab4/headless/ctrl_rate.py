#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 race.bag 量「控制器实际在跑多快」——用来区分「CPU 饥饿」与「控制律本身不稳」。

思路：
  · /rotor_speed_cmds 是 controller_node 每个控制周期发一次的（/hz=200 → 5 ms）。
    它的**接收间隔**分布 = 控制回路真实节拍。p95 远大于 5 ms 就是被抢了 CPU。
  · /current_state 的接收间隔 = 控制器能拿到新状态的节奏，出现长空档就是
    「开环跑」的时长（姿态环 83 ms 时间常数，空档 >83 ms 就该出事）。
"""
import sys
import numpy as np
from rosbags.rosbag1 import Reader


def recv_times(bag, topic):
    """返回该 topic 的 bag 接收时刻（epoch 秒，近似值）。

    ⚠️ rosbags 的 `messages(connections=...)`：传**空列表**等于「不过滤」，
    会把 bag 里**所有**消息都返回。所以必须先判空、自己短路。
    （曾因此把「没有 /clock」误读成「/clock 有 8005 条 / 107.8 Hz」。）
    """
    ts = []
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic == topic]
        if not conns:
            return np.asarray([], dtype=np.float64)
        for conn, t, _raw in r.messages(connections=conns):
            ts.append(t)
    return np.asarray(ts, dtype=np.float64) / 1e9


def clock_sim_times(bag):
    """返回 /clock 消息里**承载的仿真时间**（sec）。这是 Unity 真正告诉我们的时间轴。"""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    from pathlib import Path
    out = []
    ts = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(bag)], default_typestore=ts) as r:
        conns = [c for c in r.connections if c.topic == "/clock"]
        if not conns:                       # ← 空列表会被当成「不过滤」，必须自己短路
            return np.asarray([])
        for conn, _stamp, raw in r.messages(connections=conns):
            m = r.deserialize(raw, conn.msgtype)
            out.append(float(m.clock.sec) + float(m.clock.nanosec) * 1e-9)
    return np.asarray(out, dtype=np.float64)


def dist(name, dt_ms, unit="ms"):
    if dt_ms.size == 0:
        print(f"  {name:24} （无样本）")
        return
    q = np.percentile(dt_ms, [5, 50, 95, 99])
    print(f"  {name:24} n={dt_ms.size:6d}  "
          f"p05={q[0]:7.2f}  p50={q[1]:7.2f}  p95={q[2]:8.2f}  p99={q[3]:8.2f}  "
          f"max={dt_ms.max():9.2f}  [{unit}]")


def gaps_over(dt_ms, thr):
    m = dt_ms > thr
    n = int(m.sum())
    tot = float(dt_ms[m].sum())
    print(f"    · 间隔 > {thr:6.1f} ms 的有 {n:5d} 次，累计 {tot/1000.0:7.2f} s"
          f"（占时长 {100.0*tot/dt_ms.sum():5.1f}%）")
    return tot


def main():
    bag = sys.argv[1]
    print("=" * 78)
    print(" 控制回路实际节拍 / 状态送达节奏（从 bag 接收时刻量）")
    print("=" * 78)
    print(f" bag = {bag}\n")

    topics = ["/clock", "/rotor_speed_cmds", "/current_state", "/desired_state"]
    data = {}
    for tp in topics:
        t = recv_times(bag, tp)
        data[tp] = t
        if t.size:
            print(f" {tp:22} 条数 = {t.size}")
        else:
            print(f" {tp:22} （bag 里没有）")

    # ---- 【零】/clock：use_sim_time 下所有 ROS 定时器的驱动源 ----------------
    # 桥接的 spin() 是 `while not shutdown: clock_cb(None)`，每轮同步 UDP 往返
    # 向 Unity 要 metadata 再发一次 /clock ⇒ /clock 频率 = Unity 帧率。
    # 每跳多少仿真秒 = Unity 固定物理步长。控制器的 ros::Timer 挂在 ROS 时间上，
    # 只能跟着 /clock 跳 ⇒ **控制环频率上限 = /clock 频率**，与 /hz 无关。
    tc = data.get("/clock")
    if tc is not None and tc.size >= 3:
        dtc = np.diff(tc) * 1000.0
        span = tc[-1] - tc[0]
        print()
        print("【零】/clock —— 仿真时间节拍（= Unity 帧率；控制环的硬天花板）")
        print(f"  消息数 {tc.size}，墙钟跨度 {span:.2f} s → {tc.size/span:7.2f} Hz")
        cl = clock_sim_times(bag)
        if cl.size >= 3:
            dcl = np.diff(cl) * 1000.0
            u, c = np.unique(np.round(dcl, 3), return_counts=True)
            top = sorted(zip(c, u), reverse=True)[:6]
            print(f"  **仿真时间步长**（Unity 固定物理步长）：min={dcl.min():.1f}  "
                  f"p50={np.percentile(dcl,50):.1f}  max={dcl.max():.1f}  [ms]")
            print("  步长直方图（前 6）: "
                  + "  ".join(f"{v:.1f}ms×{n}" for n, v in top))
            print(f"  重复时间戳（步长为 0）占比 = {100.0*(dcl<1e-9).mean():.2f}%"
                  f"  ← 为 0 说明每跳都是新的物理步，/clock 与物理步一一对应")
            print(f"  仿真时间相对墙钟的推进率 = {cl[-1]-cl[0]:.2f}s / {span:.2f}s "
                  f"= {(cl[-1]-cl[0])/span:.4f}×  （1.0 = 实时）")
        print(f"  墙钟接收间隔：p05={np.percentile(dtc,5):.2f}  p50={np.percentile(dtc,50):.2f}"
              f"  p95={np.percentile(dtc,95):.2f}  max={dtc.max():.2f}  [ms]")
        print("  ⚠️ 接收间隔 p50≈0 且 p95 很大 = **接收端成簇**（rosbag 进程被饿后一次排空"
              "队列），不代表发布端节奏；发布端节奏看上面的平均 Hz 与仿真步长。")

    print()
    print("【一】各话题的**接收间隔**分布（ms）")
    for tp in topics:
        t = data[tp]
        if t.size < 2:
            continue
        dt = np.diff(t) * 1000.0
        dist(tp, dt)

    print()
    print("【二】/rotor_speed_cmds（控制器输出）—— 真实控制节拍")
    t = data["/rotor_speed_cmds"]
    if t.size >= 2:
        dt = np.diff(t) * 1000.0
        span = t[-1] - t[0]
        print(f"  消息数 {t.size}，跨度 {span:.2f} s → 平均 {t.size/span:7.2f} Hz"
              f"（期望 200 Hz → 5 ms）")
        dt50 = float(np.percentile(dt, 50))
        print(f"  期望周期 5.00 ms；实际中位接收间隔 = {dt50:.2f} ms"
              + (f"，中位等效 {1000.0/dt50:.1f} Hz" if dt50 > 0.5
                 else "（≈0 → 接收成簇，中位数无意义；看上面的平均 Hz）"))
        print("  长空档（控制器在这段时间没有任何输出 → 控制回路整体被饿）：")
        for thr in (10, 20, 50, 100):
            gaps_over(dt, thr)

    print()
    print("【三】/current_state 的送达空档（控制器拿不到新状态的时长）")
    t = data["/current_state"]
    if t.size >= 2:
        dt = np.diff(t) * 1000.0
        print("  空档累计（姿态环时间常数 83 ms 是参照）：")
        for thr in (50, 83, 100, 200, 400):
            gaps_over(dt, thr)
        # 最长的 10 个空档
        idx = np.argsort(dt)[-10:][::-1]
        print("  最长 10 个空档 [ms]：", " ".join(f"{dt[i]:.1f}" for i in idx))

    print()
    print("【四】判读")
    t = data["/rotor_speed_cmds"]
    if t.size >= 2:
        dt = np.diff(t) * 1000.0
        p95 = float(np.percentile(dt, 95))
        span = t[-1] - t[0]
        print(f"  · 控制回路实测平均 {t.size/span:.1f} Hz（目标 200 Hz）"
              f"，周期 p95 = {p95:.1f} ms")
        if p95 > 15.0:
            print("  · p95 周期 >> 5 ms → 控制回路被明显抢占（CPU/调度问题），"
                  "不是控制律本身的问题")
        else:
            print("  · p95 周期接近 5 ms → 控制回路节拍健康，「发散」另有原因")
    tc = data["/current_state"]
    if tc.size >= 2:
        dtc = np.diff(tc) * 1000.0
        n83 = int((dtc > 83.0).sum())
        print(f"  · 状态空档 > 83 ms（姿态环时间常数）的有 {n83} 次，"
              f"占 {100.0*n83/dtc.size:.1f}% —— 这些时段控制器在**开环**跑")


if __name__ == "__main__":
    main()
