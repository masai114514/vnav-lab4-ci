#!/usr/bin/env python3
"""
plant_node.py — 无头"被控对象"节点，用来替代 LAB4 官方链路里的 Unity(TESSE) 仿真。

    /rotor_speed_cmds  (mav_msgs/Actuators)   <-- controller_node
                 |
                 v
        [ 本节点 = plant_core.Plant ]
                 |
                 +--> /current_state (nav_msgs/Odometry)  --> controller_node
                 |                                        +-> trajectory_generation_node
                 +--> /clock         (rosgraph_msgs/Clock) --> 驱动 sim time

为什么需要 /clock
-----------------
`trajectory_generation_node` 是按 **ROS 时间** 采样轨迹的
（`sampling_time = ros::Time::now() - trajectoryStartTime`，定时器 `ros::Rate(5)`）。
如果节点只能跑到 300 Hz 而墙钟照走，无人机就会永远追不上"按墙钟前进"的期望轨迹。
所以本节点把仿真时间发布到 `/clock`，配合全局 `use_sim_time=true`：
**整条链路（含 rosbag 时间戳）都跑在仿真时间上**，墙钟快慢只影响录制的耗时，不影响结果。

配置（ROS 参数，均可省略）
-------------------------
  ~rate        : 积分/发布频率 [Hz]（仿真时间），默认 1000
  ~dt          : 积分步长 [s]，默认 1/rate
  ~motor_tau   : 电机一阶滞后 [s]，默认 0.03
  ~duration    : 跑多少**仿真秒**后自动退出，0 = 不限
  ~realtime    : true 时按 1:1 墙钟节流；**默认 true，别关**。
                 （曾经默认 false = "尽可能快"。那是错的：仿真时间会比节点处理能力跑得快，
                  roscpp 单线程回调队列（/current_state 1 kHz + 控制定时器 1 kHz）积压
                  持续累积 → 等效控制延迟不断增大 → 跟踪精度崩塌。
                  实测：无节流 RMSE 861 mm / 峰值速度 8.4 m/s；1:1 节流后回到 ~270 mm 量级。
                  植物跑不满 1 kHz 也没关系，仿真时间只会慢下来，结果不变。）
  ~start_z     : 初始高度 [m]，默认 0（与官方一致：无人机在地面/原点起飞）

命令行等价形式（两种都支持）：
  rosrun trajectory_generation plant_node.py --rate 1000 --duration 60
  rosparam set /plant_node/rate 1000 && rosrun ...
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

# 让 `python3 /abs/path/plant_node.py` 也能 import 同目录的 plant_core
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from plant_core import Plant  # noqa: E402


def _parse_args():
    """解析命令行参数。

    roslaunch 会注入 `__name:=...` / `__log:=...` 之类的私有参数，必须先滤掉，
    否则 argparse 会因为无法识别的 token 直接退出。
    """
    argv = [a for a in sys.argv[1:] if ":=" not in a]
    ap = argparse.ArgumentParser(description="LAB4 headless plant node")
    ap.add_argument("--rate", type=float, default=None, help="积分/发布频率 [Hz]（仿真时间）")
    ap.add_argument("--dt", type=float, default=None, help="积分步长 [s]（默认 1/rate）")
    ap.add_argument("--motor-tau", type=float, default=None, help="电机一阶滞后 [s]")
    ap.add_argument("--duration", type=float, default=None, help="运行多少仿真秒后退出（0=不限）")
    ap.add_argument("--realtime", action="store_true", help="按 1:1 墙钟节流")
    ap.add_argument("--no-hold", action="store_true",
                    help="不做地面支撑：收到首个 /desired_state 之前也积分（默认会支撑）")
    ap.add_argument("--start-z", type=float, default=None, help="初始高度 [m]")
    args, unknown = ap.parse_known_args(argv)
    if unknown:
        print("[plant_node] 忽略未识别的参数: %s" % unknown, file=sys.stderr)
    return args


def main():
    args = _parse_args()

    # rospy 在 main 里 import：这样 --help / 单元测试不需要 ROS 环境
    import rospy
    from mav_msgs.msg import Actuators
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

    rospy.init_node("plant_node", anonymous=False)

    def p(name, cli_value, default):
        """命令行优先；否则读私有参数；否则用默认值。"""
        if cli_value is not None:
            return cli_value
        if rospy.has_param("~" + name):
            return rospy.get_param("~" + name)
        return default

    rate_hz = float(p("rate", args.rate, 1000.0))
    dt = float(p("dt", args.dt, 1.0 / rate_hz))
    motor_tau = float(p("motor_tau", args.motor_tau, 0.03))
    duration = float(p("duration", args.duration, 0.0))
    realtime = bool(p("realtime", True if args.realtime else None, False))
    start_z = float(p("start_z", args.start_z, 0.0))

    plant = Plant(motor_tau=motor_tau, pos0=np.array([0.0, 0.0, start_z]))

    # 仿真时钟。**必须在创建订阅者之前定义** —— rospy 的回调跑在独立线程里，
    # on_desired_state 可能在主线程给 sim_t 赋值之前就被调用。
    sim_t = 0.0

    # --- 订阅：控制器的桨转速指令 ----------------------------------------
    n_cmd = {"n": 0, "bad": 0}

    def on_rotor_speed_cmd(msg):
        w = np.zeros(4)
        vals = np.asarray(list(msg.angular_velocities), float)
        if vals.size < 4:
            n_cmd["bad"] += 1
            if n_cmd["bad"] == 1:
                rospy.logwarn(
                    "[/rotor_speed_cmds] 只收到 %d 个转速（期望 4），其余按 0 处理", vals.size
                )
        else:
            w[:] = vals[:4]
        plant.set_command(w)
        n_cmd["n"] += 1

    rospy.Subscriber("/rotor_speed_cmds", Actuators, on_rotor_speed_cmd, queue_size=1)

    # --- 订阅：/desired_state —— 用来判断"任务是否已开始" --------------------
    #
    # 为什么需要它（**这是个必须处理的坑**）
    # ------------------------------------
    # controller_node 的 xd / vd / ad 是**未初始化的 Eigen 向量**，而它的控制定时器
    # 从节点构造那一刻就开始跑。在第一个 /desired_state 到达之前（启动空窗期），
    # 控制器会拿垃圾参考算出疯狂的桨转速。
    #
    # 官方 Unity(TESSE) 里这不成问题：无人机**停在地面上**，按 W 才解锁，启动空窗期的
    # 指令根本作用不到机体上。
    #
    # 所以替身植物也必须照做：在收到第一个 /desired_state 之前**保持地面支撑状态**
    # —— 不积分、不响应任何桨转速指令（位置/速度/姿态都停在初值上）。
    # 收到第一个 /desired_state 后立刻释放，从 (0, 0, start_z) 干净起飞。
    #
    # 注意：**仿真时间在此期间照常推进并发布 /clock**，否则编排脚本等待 /clock、
    # 而 /desired_state 又要等 launch 起来，会互相死锁。
    hold_until_desired = bool(p("hold_until_desired",
                                False if args.no_hold else None, True))
    gate = {"released": not hold_until_desired, "t_release": None}

    def on_desired_state(_msg):
        if not gate["released"]:
            gate["released"] = True
            gate["t_release"] = sim_t if sim_t > 0 else 0.0
            rospy.loginfo("[plant_node] 收到首个 /desired_state —— 释放地面支撑，开始积分")

    rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint,
                     on_desired_state, queue_size=1)

    # --- 发布：状态 + 时钟 -------------------------------------------------
    state_pub = rospy.Publisher("/current_state", Odometry, queue_size=10)
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)

    rospy.loginfo(
        "[plant_node] rate=%.0f Hz  dt=%.6f s  motor_tau=%.3f s  duration=%.1f sim-s  "
        "realtime=%s  start_z=%.2f  hold_until_desired=%s",
        rate_hz, dt, motor_tau, duration, realtime, start_z, hold_until_desired,
    )
    if not rospy.get_param("/use_sim_time", False):
        rospy.logwarn(
            "[plant_node] /use_sim_time=false —— 本节点仍然会推 /clock，但轨迹节点会按墙钟采样。"
            "建议 `rosparam set /use_sim_time true` 后再启动其余节点。"
        )

    t_end = duration if duration > 0 else None
    # sim_t 已在创建订阅者之前初始化（见上），这里不再重置。
    wall0 = time.time()
    warned_no_cmd = False
    n_held = 0

    while not rospy.is_shutdown():
        # 1) 推进被控对象
        #
        # 地面支撑阶段：不积分、不响应桨转速指令（等价于无人机停在地面等待解锁）。
        # 仿真时间仍然推进并发布 /clock —— 否则编排脚本等不到 /clock 而死锁。
        if gate["released"]:
            plant.step(dt)
        else:
            n_held += 1
        sim_t += dt

        # 2) 发布 /current_state
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(sim_t)
        odom.header.frame_id = "world"
        odom.child_frame_id = "body"

        odom.pose.pose.position.x = float(plant.pos[0])
        odom.pose.pose.position.y = float(plant.pos[1])
        odom.pose.pose.position.z = float(plant.pos[2])

        q = plant.quaternion()
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        odom.twist.twist.linear.x = float(plant.vel[0])
        odom.twist.twist.linear.y = float(plant.vel[1])
        odom.twist.twist.linear.z = float(plant.vel[2])

        # ⚠️ 世界系角速度！（controller_node 自己做 R^T * omega_world）
        ow = plant.omega_world()
        odom.twist.twist.angular.x = float(ow[0])
        odom.twist.twist.angular.y = float(ow[1])
        odom.twist.twist.angular.z = float(ow[2])

        state_pub.publish(odom)

        # 3) 推进仿真时钟
        clk = Clock()
        clk.clock = rospy.Time.from_sec(sim_t)
        clock_pub.publish(clk)

        if not warned_no_cmd and sim_t > 2.0 and n_cmd["n"] == 0:
            rospy.logwarn(
                "[/rotor_speed_cmds] 启动 2 个仿真秒仍没收到任何桨转速 —— "
                "检查 controller_node 是否在跑、话题是否是 /rotor_speed_cmds。"
            )
            warned_no_cmd = True

        # 4) 结束条件 / 节流
        if t_end is not None and sim_t >= t_end:
            rospy.loginfo("[plant_node] 到达 duration=%.1f sim-s，正常退出。", duration)
            break

        if realtime:
            d = (wall0 + sim_t) - time.time()
            if d > 0:
                time.sleep(d)

    rospy.loginfo(
        "[plant_node] 退出。sim_t=%.2f s  地面支撑 %d 步（%.1f sim-s）  "
        "释放于 sim_t=%.2f s  收到转速指令 %d 条  最终位置=(%.3f, %.3f, %.3f)",
        sim_t, n_held, n_held * dt,
        gate["t_release"] if gate["t_release"] is not None else -1.0,
        n_cmd["n"], plant.pos[0], plant.pos[1], plant.pos[2],
    )


if __name__ == "__main__":
    main()
