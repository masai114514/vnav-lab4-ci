#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lag_probe.py — 量「控制器看到的 /current_state 到底有多旧」

为什么需要它
------------
官方 Unity 链路发散那趟，我们量到了两件事：
  · 植物确实实时 50 Hz 步进（/clock 与状态时间戳都是 20.000 ms 一格）
  · 但 /current_state **到达**是突发式的：中位间隔 60 ms、95% 240 ms、最大 480 ms，
    3871 条里 2822 对相邻消息的**接收时刻完全相同**

第二件事是「链路把状态攒起来一次性投递」，但它**不等于**状态本身有多旧。
真正决定闭环稳定性的量是：

        lag = 收到消息那一刻的仿真时间 − 消息自带的仿真时间戳

姿态环时间常数 ≈ 1/ω ≈ 83 ms（kr=150 / kω=15 → 带宽≈12 rad/s）。
lag 一旦和这个量级相当甚至更大，相位裕度就没了 —— 这才是「斜着飞」的机理。
所以必须把这个数分布量出来，而不是靠「到达间隔」间接猜。

怎么量才准
----------
时间戳是**仿真时间**，而仿真时间与墙钟有 ~2.4% 的漂移（实测 75.05 s 墙钟 / 73.22 s 仿真）。
拿墙钟去减仿真戳，70 秒下来会累积 1.7 s 的假延迟，把真实信号淹掉。

于是本节点跑在 `use_sim_time=true` 下：此时 `rospy.Time.now()` 就是**最近一条 /clock**，
也就是「当前仿真时刻」。二者相减才是有物理意义的 lag。

顺带量三组数
------------
  A. 仿真步长分布   —— 从 header.stamp 的相邻差看植物是不是真 50 Hz
  B. 接收间隔分布   —— 墙钟口径，看突发结构（含「同一时刻到达」的碰撞率）
  C. lag 分布       —— 上式，闭环真正吃的那个延迟
  D. lag 随时间走势 —— 每 5 s 一档的均值/最大值，看它是稳定还是恶化

用法
----
    # 竞速进行中另开一个终端
    python3 lab4/headless/lag_probe.py --duration 90 --out /tmp/lag_probe

    # 或直接在 official_race.sh 里后台拉起，和竞速并行
    python3 .../lag_probe.py --duration 90 --out "$OUT/lag_probe" &

产出
----
    <out>.txt   人看的汇总（可直接贴进报告）
    <out>.csv   逐条原始样本，供事后画图

话题名可用 --state / --clock 覆盖；默认 /current_state 与 /clock。
"""

from __future__ import print_function

import argparse
import csv
import os
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------
def pct(sorted_vals, q):
    """分位数。sorted_vals 必须已排序；空列表返回 nan。"""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    pos = q * (len(sorted_vals) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = pos - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def fmt_ms(x):
    return "nan" if x != x else "%.1f" % (x * 1e3)


def describe(vals_sorted):
    return ("n=%d  min=%s  p05=%s  p50=%s  p95=%s  max=%s  [ms]"
            % (len(vals_sorted),
               fmt_ms(vals_sorted[0] if vals_sorted else float("nan")),
               fmt_ms(pct(vals_sorted, 0.05)),
               fmt_ms(pct(vals_sorted, 0.50)),
               fmt_ms(pct(vals_sorted, 0.95)),
               fmt_ms(vals_sorted[-1] if vals_sorted else float("nan"))))


# --------------------------------------------------------------------------
# 采样
# --------------------------------------------------------------------------
class LagProbe(object):
    def __init__(self, state_topic, clock_topic):
        self.samples = []          # (t_wall, sim_stamp, lag, dt_recv, dt_stamp)
        self.clock_count = 0
        self.clock_last = None
        self.last_wall = None
        self.last_stamp = None

        self.sub_state = rospy.Subscriber(state_topic, Odometry, self.on_state,
                                          queue_size=200)
        self.sub_clock = rospy.Subscriber(clock_topic, Clock, self.on_clock,
                                          queue_size=200)

    def on_clock(self, _msg):
        # use_sim_time=true 时 rospy.Time.now() 由 /clock 驱动，
        # 所以这里只需要记数，用它判断 /clock 本身是否在正常跳。
        self.clock_count += 1
        self.clock_last = time.time()

    def on_state(self, msg):
        t_wall = time.time()

        # 仿真时刻：use_sim_time 下就是最近一条 /clock
        now_sim = rospy.Time.now()
        stamp = msg.header.stamp
        sim_stamp = stamp.secs + stamp.nsecs * 1e-9

        lag = now_sim.to_sec() - sim_stamp
        dt_recv = float("nan") if self.last_wall is None else t_wall - self.last_wall
        dt_stamp = float("nan") if self.last_stamp is None else sim_stamp - self.last_stamp

        self.samples.append((t_wall, sim_stamp, lag, dt_recv, dt_stamp))
        self.last_wall = t_wall
        self.last_stamp = sim_stamp


# --------------------------------------------------------------------------
# 汇总
# --------------------------------------------------------------------------
def report(probe, out_base, realtime):
    s = probe.samples
    lines = []

    def w(x=""):
        lines.append(x)

    w("=" * 72)
    w(" /current_state 延迟实测（lag_probe.py）")
    w("=" * 72)
    w(" use_sim_time      : %s" % realtime)
    w(" /clock 消息数     : %d" % probe.clock_count)
    w(" /current_state 条数: %d" % len(s))
    if len(s) < 10:
        w()
        w(" !! 样本太少（%d 条），无法给出分布。" % len(s))
        w("    常见原因：use_sim_time 没生效、话题名不对、竞速还没开始。")
        txt = "\n".join(lines)
        with open(out_base + ".txt", "w") as f:
            f.write(txt + "\n")
        print(txt)
        return txt

    t0 = s[0][0]
    span = s[-1][0] - t0
    w(" 采样时长          : %.1f s（墙钟）" % span)

    # ---- A. 仿真步长：植物是不是真 50 Hz -------------------------------
    dts = sorted(x for x in (r[4] for r in s) if x == x and x > 1e-9)
    w()
    w("[A] 仿真步长（相邻 header.stamp 之差）")
    w("    " + describe(dts))
    if dts:
        w("    → 相当于 %.2f Hz" % (1.0 / pct(dts, 0.50)))
        zero = sum(1 for r in s if r[4] == 0.0)
        w("    时间戳重复的相邻对: %d（%.1f%%）" % (zero, 100.0 * zero / len(s)))

    # ---- B. 接收间隔：突发结构 ----------------------------------------
    recv = sorted(x for x in (r[3] for r in s) if x == x and x > 1e-9)
    w()
    w("[B] 接收间隔（墙钟，相邻两条消息到达时刻之差）")
    w("    " + describe(recv))
    if recv:
        w("    → 等效到达率 %.2f Hz（按中位间隔）" % (1.0 / pct(recv, 0.50)))
    collide = sum(1 for r in s if r[3] == 0.0)
    w("    接收时刻完全相同的相邻对: %d（%.1f%%）"
      % (collide, 100.0 * collide / len(s)))
    w("    注：到达率低 ≠ 状态旧；真实延迟看 [C]。")

    # ---- C. lag：闭环真正吃的那个数 -----------------------------------
    lags_all = [r[2] for r in s]
    lags_ok = sorted(x for x in lags_all if x == x)
    neg = sum(1 for x in lags_ok if x < 0)
    w()
    w("[C] lag = 收到时的仿真时刻 − 消息自带的仿真时间戳   ★核心指标")
    w("    " + describe(lags_ok))
    if neg:
        w("    ⚠ 负值 %d 条（%.1f%%）：时间戳比 /clock 还新，"
          "通常是 /clock 与状态不同源，需要在报告里说明" % (neg, 100.0 * neg / len(lags_ok)))
    w()
    w("    对照：姿态环时间常数 ≈ 83 ms（kr=150 / kω=15 → 带宽≈12 rad/s）")
    p95 = pct(lags_ok, 0.95)
    if p95 != p95:
        pass
    elif p95 < 0.083:
        w("    → p95 lag %.1f ms < 83 ms：环路有裕度，发散不该由延迟解释"
          % (p95 * 1e3))
    elif p95 < 0.200:
        w("    → p95 lag %.1f ms 已接近/超过时间常数：裕度被吃掉，容易低频失稳"
          % (p95 * 1e3))
    else:
        w("    → p95 lag %.1f ms 是时间常数的 %.1f 倍：★相位裕度基本归零，"
          "这条链路本身就飞不稳" % (p95 * 1e3, p95 / 0.083))

    # ---- D. lag 随时间走势 --------------------------------------------
    w()
    w("[D] lag 每 5 s 分档（看是稳定、还是随负载恶化）")
    w("      窗口(s)      n    mean[ms]    p95[ms]    max[ms]")
    bucket = {}
    for t, _, lag, _, _ in s:
        if lag != lag:
            continue
        bucket.setdefault(int((t - t0) // 5) * 5, []).append(lag)
    for k in sorted(bucket):
        v = sorted(bucket[k])
        w("      %3d-%3d   %5d   %9.1f  %9.1f  %9.1f"
          % (k, k + 5, len(v),
             sum(v) / len(v) * 1e3, pct(v, 0.95) * 1e3, v[-1] * 1e3))

    txt = "\n".join(lines)

    with open(out_base + ".txt", "w") as f:
        f.write(txt + "\n")
    with open(out_base + ".csv", "w") as f:
        cw = csv.writer(f)
        cw.writerow(["t_wall_rel", "sim_stamp", "lag", "dt_recv", "dt_stamp"])
        for r in s:
            cw.writerow(["%.6f" % (r[0] - t0), "%.6f" % r[1],
                         "%.6f" % r[2], "%.6f" % r[3], "%.6f" % r[4]])

    print(txt)
    print("\n[写盘] %s.txt / %s.csv" % (out_base, out_base))
    return txt


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="/current_state 延迟实测")
    ap.add_argument("--duration", type=float, default=90.0, help="采样时长（秒，墙钟）")
    ap.add_argument("--out", default="/tmp/lag_probe", help="输出前缀（不含扩展名）")
    ap.add_argument("--state", default="/current_state", help="状态话题")
    ap.add_argument("--clock", default="/clock", help="时钟话题")
    args, _ = ap.parse_known_args()

    rospy.init_node("lag_probe", anonymous=True)

    # 关键：必须让 rospy.Time.now() 走仿真时间，否则 lag 会被墙钟/仿真漂移淹掉
    realtime = rospy.get_param("/use_sim_time", False)
    if not realtime:
        print("[warn] /use_sim_time = False！lag 会混入墙钟与仿真的漂移（约 2.4%），"
              "结果不可用。请确认 /clock 在发且 use_sim_time 已置 true。")

    probe = LagProbe(args.state, args.clock)
    print("[lag_probe] 采样 %.0f s，state=%s clock=%s" %
          (args.duration, args.state, args.clock))

    deadline = time.time() + args.duration
    while not rospy.is_shutdown() and time.time() < deadline:
        time.sleep(0.25)

    out_base = args.out
    d = os.path.dirname(out_base)
    if d and not os.path.isdir(d):
        os.makedirs(d)

    report(probe, out_base, realtime)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(0)
