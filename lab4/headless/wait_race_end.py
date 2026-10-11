#!/usr/bin/env python3
"""
wait_race_end.py — 判断竞速跑完了没有（供 run_race_headless.sh 收尾用）。

判据（默认 plateau）
-------------------
`trajectory_generation_node` 发布的 `/desired_state` 里带 `time_from_start`，
它等于 `min(ros::Time::now() - trajectoryStartTime, trajectory.getMaxTime())`。
所以一旦采样时间撞到 `getMaxTime()`，`time_from_start` 就**停止增长** —— 不管无人机
有没有真的跟住轨迹。这使得它成为一个**与飞行成败无关**的可靠结束信号。

因此：连续收到 N 条 `time_from_start` 几乎不变的 `/desired_state`，即判定竞速结束。

备用判据（clock）：直接等 `/clock` 推进到指定仿真时刻，用于 plateau 一直不触发时的兜底。

退出码：0 = 竞速结束；1 = 墙钟超时；2 = 被 shutdown / Ctrl-C。
"""

from __future__ import annotations

import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["plateau", "clock"], default="plateau")
    ap.add_argument("--t", type=float, default=0.0, help="clock 模式的目标仿真时刻 [s]")
    ap.add_argument("--plateau-n", type=int, default=5, help="连续多少条相同即判定结束")
    ap.add_argument("--min-t", type=float, default=1.0, help="time_from_start 至少要到多少才认")
    ap.add_argument("--timeout", type=float, default=3600.0, help="墙钟超时 [s]")
    args = ap.parse_args()

    import rospy
    from rosgraph_msgs.msg import Clock
    from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

    rospy.init_node("wait_race_end", anonymous=True)

    st = {"seen": 0, "last": None, "same": 0, "tfs": 0.0, "clock": 0.0}

    def on_desired(msg):
        t = msg.time_from_start.to_sec()
        st["tfs"] = t
        if st["last"] is not None and abs(t - st["last"]) < 1e-6:
            st["same"] += 1
        else:
            st["same"] = 0
        st["last"] = t
        st["seen"] += 1

    def on_clock(msg):
        st["clock"] = msg.clock.to_sec()

    rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint, on_desired, queue_size=50)
    rospy.Subscriber("/clock", Clock, on_clock, queue_size=50)

    wall0 = time.time()
    last_report = 0.0
    try:
        while not rospy.is_shutdown():
            if args.mode == "plateau":
                if st["seen"] > 0 and st["tfs"] >= args.min_t and st["same"] >= args.plateau_n:
                    print("[wait_race_end] /desired_state 已停止推进："
                          "time_from_start=%.3f s（连续 %d 条不变），竞速结束。"
                          % (st["tfs"], st["same"]))
                    return 0
            else:
                if st["clock"] >= args.t:
                    print("[wait_race_end] /clock 到达 %.2f s，竞速结束。" % st["clock"])
                    return 0

            if time.time() - wall0 - last_report > 15.0:
                last_report = time.time() - wall0
                if st["seen"] == 0:
                    print("[wait_race_end] 等待 /desired_state ...（已等 %.0f s 墙钟，"
                          "仿真 %.1f s）。若一直没消息，检查 traj_vertices_publisher 与 CSV 路径。"
                          % (last_report, st["clock"]), flush=True)
                else:
                    print("[wait_race_end] 进行中：仿真 %.1f s，"
                          "time_from_start=%.2f s（%d 条，连续相同 %d）"
                          % (st["clock"], st["tfs"], st["seen"], st["same"]), flush=True)

            if time.time() - wall0 > args.timeout:
                print("[wait_race_end] 墙钟超时 %.0f s（仿真 %.1f s，"
                      "time_from_start=%.2f s）。" % (args.timeout, st["clock"], st["tfs"]),
                      file=sys.stderr)
                return 1

            time.sleep(0.05)
    except KeyboardInterrupt:
        return 2

    return 2


if __name__ == "__main__":
    sys.exit(main())
