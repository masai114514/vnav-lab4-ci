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
    """返回该 topic 的 bag 接收时刻（epoch 秒，近似值）。"""
    ts = []
    with Reader(bag) as r:
        for conn, t, _raw in r.messages(connections=[c for c in r.connections if c.topic == topic]):
            ts.append(t)
    return np.asarray(ts, dtype=np.float64) / 1e9


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

    topics = ["/rotor_speed_cmds", "/current_state", "/desired_state"]
    data = {}
    for tp in topics:
        t = recv_times(bag, tp)
        data[tp] = t
        print(f" {tp:22} 条数 = {t.size}")

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
        print(f"  期望周期 5.00 ms；实际 p50 = {np.percentile(dt,50):.2f} ms"
              f"，中位等效 {1000.0/np.percentile(dt,50):.1f} Hz")
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
