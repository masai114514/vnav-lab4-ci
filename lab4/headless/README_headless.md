# LAB4 无头复现 kit（headless/）

> **目的**：在没有 Unity 录屏、没有桌面 Linux 的条件下，产出一份**真实的 ROS rosbag**
> （`/current_state` + `/desired_state`），满足 LAB4 团队交付物里的"最快那趟的 rosbag"。
>
> 代价：被控对象是**替身植物节点**，不是官方 Unity(TESSE)。除此之外，跑的就是你提交的那套
> C++ 代码（`trajectory_generation_node` + lab3 `controller_node`），由它们真实驱动。

---

## 1. 原理：为什么 rosbag 不需要 GUI

官方 rosbag 录的只是两个**话题**：

| 话题 | 类型 | 谁发布 |
|---|---|---|
| `/current_state` | `nav_msgs/Odometry` | 被控对象（官方 = Unity(TESSE)） |
| `/desired_state` | `trajectory_msgs/MultiDOFJointTrajectoryPoint` | `trajectory_generation_node` |

整条竞速链路里，**唯一需要显示器的是 Unity 里那份动力学渲染**。把它换掉，闭环就完整了：

```
traj_vertices_publisher ──/desired_traj_vertices──► trajectory_generation_node ──/desired_state──┐
        ▲                                     （PoseArray, 12 道门）                                │
        │ CSV 门位姿                                                                                 ▼
        └────────────────────  controller_node ──/rotor_speed_cmds──►  plant_node（本 kit）
                                                                             │
                                                        /current_state ◄─────┘
                                                                             │
                                                                    /clock ◄─┘（驱动仿真时间）

        rosbag record /current_state /desired_state
```

接口全部来自源码，不是猜的：

| 节点 | 订阅 | 发布 |
|---|---|---|
| `traj_vertices_publisher` | — | `/desired_traj_vertices`（`PoseArray`，5 Hz） |
| `trajectory_generation_node` | `/current_state`、`/desired_traj_vertices` | `/desired_state`（**5 Hz** 定时器） |
| `controller_node`（lab3） | `/desired_state`、`/current_state` | `/rotor_speed_cmds`（`mav_msgs/Actuators`） |
| **`plant_node.py`**（本 kit） | `/rotor_speed_cmds` | `/current_state`、`/clock` |

---

## 2. 文件清单

| 文件 | 作用 |
|---|---|
| `plant_core.py` | 被控对象**纯数学**部分（无 ROS）：F 矩阵、SO(3) 工具、RK4 动力学 + 电机一阶滞后 |
| `plant_node.py` | ROS 节点壳：`/rotor_speed_cmds` → 积分 → `/current_state` + `/clock` |
| `wait_race_end.py` | 竞速结束判据：`/desired_state.time_from_start` 停止推进即结束 |
| `analyze_race_bag.py` | 自动分析 bag：时长 / RMSE / 最大速度 / 过门数，并与离线基线对照 |
| `check_headless.py` | **本地自检**（不需要 ROS），含与 `lab3/sim/quad_sim.py` 的闭环等价性验证 |
| `setup_autodl.sh` | AutoDL 一键装 ROS Noetic + 依赖 + `catkin build` |
| `run_race_headless.sh` | 一键跑竞速 + 录 bag + 分析 |
| `sim_data/StreamingAssets/vnav-2020-lab4_static_tfs.csv` | 官方 12 道门位姿（`traj_vertices_publisher` 要读它） |

---

## 3. 快速开始

> **⚠️ 实测结论先说**：**AutoDL 无卡模式跑不了**。它的容器被 cgroup 限制在
> **2 GiB 内存 / 0.5 核**（`free` 显示的 755 GB 是宿主机，骗人的），
> `conda` 与 `micromamba` 解算 RoboStack **都被 OOM 杀（exit 137）**，
> `swapon` 也加不上（容器无 `CAP_SYS_ADMIN`）。
>
> **实际跑通的是本机 Docker + 官方 `ros:noetic-ros-base` 镜像**
> （Ubuntu 20.04.6 focal + 原版 Noetic，Apple Silicon 上经 colima `vz` + Rosetta 跑 amd64）：
>
> ```bash
> # Docker Hub 被墙，换国内镜像源
> docker pull --platform linux/amd64 docker.1ms.run/library/ros:noetic-ros-base
> docker tag docker.1ms.run/library/ros:noetic-ros-base ros:noetic-ros-base
> docker run -d --name vnav --platform linux/amd64 \
>   -v "$PWD/lab4:/work/lab4" -w /work/lab4 ros:noetic-ros-base sleep infinity
>
> # 一键：装依赖 → catkin build → 跑竞速 → 录 bag → 分析
> docker exec vnav bash /work/lab4/headless/docker_in_container.sh
> # 产物在 lab4/race_out/
> ```
>
> 这条路比 RoboStack 更好：就是原版 focal + Noetic，不掺 conda 的 ABI 风险。
> 下面的 AutoDL 流程保留作为参考（换 x86 原生 Ubuntu 20.04 机器同样适用）。

### 3.1 AutoDL 路线（不推荐，会 OOM）

```bash
# ── 一次性：把 机器人实验/ 整个目录传到 AutoDL（含 lab3/ 与 lab4/）──
#    AutoDL 无卡模式（¥0.1/小时）就能装；但编译阶段内存/核数吃紧，
#    建议先用带卡小实例（如 3060）构建，跑通后再关机切无卡。

source /etc/network_turbo                 # AutoDL 学术加速（依赖包要连 github，必须开）

# ① 装环境 + 编译（约 15–40 分钟，取决于核数）
bash lab4/headless/setup_autodl.sh

# ② 每个新终端都要 source
source /opt/ros/noetic/setup.bash
source ~/vnav_ws/devel/setup.bash

# ③ 跑竞速 + 录 bag + 分析
bash lab4/headless/run_race_headless.sh

# ④ 看结果
cat ~/race_out/metrics.json
rosbag info ~/race_out/race.bag

# ⑤ 收工（别白花钱）
/usr/bin/shutdown
```

产出默认落在 `~/race_out/`：

| 文件 | 说明 |
|---|---|
| `race.bag` | ★ 提交用 rosbag |
| `metrics.json` | 指标 |
| `race_tracking.png` | 实际 vs 期望轨迹 |
| `plant.log` / `traj_following.log` / `traj_gen.log` / `rosbag.log` | 排错先看这四个 |

---

## 4. 已经在本机验证过的部分

本机没有 ROS，所以**ROS 相关的部分只能靠"接口对齐 + 数学等价"来保证**。已验证：

```bash
# 无需 ROS 的完整自检
python3 lab4/headless/check_headless.py
```

| 组 | 内容 | 结果 |
|---|---|---|
| A | `F` / `F2W` 往返；悬停转速 → 力旋量 `= [mg,0,0,0]`（`w_hover=49.5227 rad/s`） | ✅ |
| B | 四元数 ↔ 旋转矩阵往返（随机 200 组），`Rz(0.7)` 的解析解 | ✅ max\|ΔR\|=8.9e-16 |
| C | 物理自检：悬停转速起跳高度零漂移；阶跃后 `v∞ = -1.5gτ`（闭式解，τ=0.03） | ✅ 精确命中 |
| **D** | ★ `plant_core.Plant` 与 `lab3/sim/quad_sim.QuadrotorPlant` 的**闭环轨迹逐位一致**（圆轨迹 + lab3 控制器，5 s @1 kHz） | ✅ **误差 0.000e+00** |
| E | `omega_world = R @ omega_body`，且 `R^T @ omega_world` 能还原机体角速度 | ✅ |

**D 组是这套 kit 可信度的支点**：它证明"替身植物"不是另写了一个模型，而是与离线仿真里
那个被控对象**完全同一个**模型（逐位一致，不是"差不多"）。

另外 `analyze_race_bag.py` 的门位姿解析也在本机验证过：读出的 12 道门与离线仿真一致
（G0 `(0.42, 3.17, 2.67)` … G11 `(-39.62, 14.97, 2.51)`）。

---

## 5. 跑完怎么判断成功了

`run_race_headless.sh` 最后会自动打印分析结果。

**★★ 最有用的判据：`max_speed_mps` 应接近 `max_des_speed_mps`。**
实跑时期望峰值约 3.25 m/s。如果实测峰值跑到 8–12 m/s，说明控制环在颤振
（→ 去查 `PLANT_REALTIME` 是否 =1、地面支撑是否生效，见 §6 的第 5、6 条）。
这个判据比 RMSE 灵敏得多。

**RMSE 参考区间**（同一台机器、同一条期望轨迹）：

| 口径 | 位置 RMSE |
|---|---|
| 把 bag 里的期望轨迹回放给**离线**控制器（`sim/replay_bag.py`） | ≈ **271 mm** ← 这条链路的"理想值" |
| 实跑（容器内，1:1 节流 + 地面支撑） | ≈ **517 mm** |
| 实跑（无节流 / 无地面支撑，两个 bug 都在） | ≈ **901 mm** ❌ |

- 实测 RMSE 在 **300–600 mm** 区间且 12/12 过门 → **复现成功** ✓
- 实测 < 150 mm → 期望状态大概没有真的跑在 5 Hz，检查 `traj_following.log`
- 实测 > 700 mm → 两个 bug 至少中了一个，见 §6 第 5、6 条

> ⚠️ **别把几百 mm 当成 bug**：官方代码的期望状态就是 5 Hz（`ros::Rate(5)`），
> 在 3 m/s 的竞速速度下这本身就会产生百毫米级的零阶保持误差。
> 想冲**额外加分**（最快无人机，现纪录 10.3 s），两个最直接的发力点：
> ① 把 `trajectory_generation_node.cpp` 里的 `ros::Rate(5)` 调大（如 50–100 Hz）；
> ② 缩短传给 `setupFromVertices` 的 `segment_times`（在线 x86 机器上才建议动，容器里余量不多）。

---

## 6. 设计决策（都是踩出来的）

1. **仿真时间 (`/use_sim_time=true` + 植物节点推 `/clock`)**
   `trajectory_generation_node` 按 **ROS 时间**采样轨迹
   （`sampling_time = now() - trajectoryStartTime`）。纯 Python 植物节点很难稳定跑满实时 1 kHz，
   若不接管时间，"按墙钟前进的期望轨迹"会把无人机永久甩开，结果完全不可信。
   推 `/clock` 后整条链路（**含 bag 时间戳**）都跑在仿真时间上：机器慢只是录得久，结果不变。

2. **`/current_state` 的角速度必须是世界系**
   `controller_node::onCurrentState` 里做的是 `omega = R^T * omega_world` ——
   也就是它**假定** `twist.twist.angular` 是世界系的。发成机体系会让姿态环直接跑飞。
   本 kit 在 `plant_core.omega_world()` 里显式转换，并在自检 E 组盯着这一条。

3. **启动顺序**：植物 → `traj_following.launch` → **开始录制** → `traj_gen.launch`。
   门位姿一发布，竞速立即开始（`generateOptimizedTrajectory` 用当时的 `/current_state` 做起点，
   `trajectoryStartTime = now()`）。所以录制必须在发布门位姿**之前**开始，否则漏掉起飞段；
   而植物必须先于轨迹节点启动，否则起点会取到未初始化的 `Eigen::Vector3d x`。

4. **竞速结束判据用 `/desired_state` 而非计时**
   `time_from_start = min(now - start, getMaxTime())`，撞到 `getMaxTime()` 后就不再增长 ——
   **与飞行成败无关**，所以它是个可靠的结束信号（`wait_race_end.py`）。

5. **植物必须按 1:1 墙钟节流（`--realtime`，现在是默认）** —— ★ 实测踩出来的
   曾经默认"尽可能快"。那是错的：仿真时间会比节点处理能力跑得快（实测 ~1.2× 墙钟），
   而 roscpp 单线程回调队列要处理 `/current_state` 1 kHz + 控制定时器 1 kHz，
   消化不完 → **积压持续累积** → 控制动作的等效延迟不断增大 → 跟踪精度崩塌。
   实测（同一条期望轨迹，离线复算）：

   | 控制延迟 | 位置 RMSE | 峰值速度 |
   |---|---|---|
   | 0–10 ms | ~270 mm | 2.4 m/s |
   | 30 ms | 611 mm | 7.7 m/s |
   | 无节流实跑 | **911 mm** | **12.2 m/s** |
   | 1:1 节流后实跑 | **517 mm** | 10.7 m/s |

   植物跑不满 1 kHz 也没关系 —— 仿真时间只会慢下来，结果不变。

6. **必须做"地面支撑"：收到首个 `/desired_state` 之前不要积分** —— ★ 另一个实测踩出来的坑
   `controller_node` 的 `xd / vd / ad` 是**未初始化的 Eigen 向量**，而它的控制定时器
   **从节点构造那一刻就开始跑**。在第一个 `/desired_state` 到达之前（启动空窗期十几秒），
   控制器会拿垃圾参考算出疯狂的桨转速。
   官方 Unity 里这不成问题 —— **无人机停在地面上，按 W 才解锁**。植物节点原先没有这个约束：
   实测 bag 首条 `/current_state` 是 `p = (0, 0, -14.71)`，无人机已经自由落体到地下 14.7 m，
   还以 38 m/s 往上猛拉（`w_max=1e9` 无推力上限）。
   `plant_node.py` 的 `hold_until_desired`（默认开）修正了它：释放前不积分、不响应指令，
   但**照常推进并发布 `/clock`**（否则编排脚本等 `/clock`、`/desired_state` 又要等 launch，死锁）。
   实测地面支撑 9.2 仿真秒后在 `sim_t = 9.17 s` 干净释放。

7. **电机一阶滞后 τ=0.03 s、限幅、RK4 + Rodrigues** 全部照搬 `quad_sim.py`，不做"简化"。

---

## 7. 故障排查

| 现象 | 原因 / 处理 |
|---|---|
| `catkin build` 失败，卡在 `glog_src` / `nlopt_src` | 依赖包全是**下载源码**构建，必须能连 github。`source /etc/network_turbo` 后重试 |
| `glog_src` 下载卡在 0 字节 | **`github.com` 的网页域名被卡，但 `codeload.github.com` 通**。`docker_in_container.sh` 会自动探测镜像（`ghfast.top` 等）并改写 URL |
| `nlopt` 报 `make[5]: *** write jobserver: Bad file descriptor` | nlopt 的 `BUILD_COMMAND` 是**裸 `make`**，继承了 catkin `-j4` 的 jobserver FD 而那 FD 已失效。GNU make 只有命令行显式带 `-j` 时才忽略它（所以 glog 的 `make -j 8` 没事）。脚本已改成 `env -u MAKEFLAGS -u MFLAGS make -j$JOBS` |
| `catkin build` 被 OOM 杀掉 | 2 GB 内存不够（`glog_catkin` 里硬编码了 `make -j 8`）。换带卡实例，或 `JOBS=1` |
| `setup_autodl.sh` 报 Ubuntu 版本不对 | ROS Noetic 只发 focal(20.04) 的 deb。AutoDL 是 22.04 → 走 `setup_robostack.sh`，或直接用官方 `ros:noetic-ros-base` 容器 |
| 植物节点没 `/clock` | 看 `plant.log`；确认 `python3 -c "import numpy"` 可用 |
| `/desired_state` 一直没消息 | 看 `traj_gen.log` 是否有 `Finished loading trajectory csv: 12`。没有 → `sim_data/StreamingAssets/...csv` 路径不对 |
| 无人机原地打转 / 直接坠地 | 检查是不是忘了 `rosparam set /use_sim_time true`（`run_race_headless.sh` 已代劳） |
| 跟踪 RMSE 明显偏大（>700 mm）、`max_speed` 远超期望 | 先查 `PLANT_REALTIME` 是否为 1；再查 bag 首条 `/current_state` 的 z（应为 ~0，若是几十米深说明地面支撑没生效） |
| 过门数 < 12 | 正常现象：官方 `ros::Rate(5)` 的 5 Hz 期望状态本身就跟不准；提高发布频率再跑 |

---

## 8. 局限与提交声明（重要）

- **这份 bag 是"无头复现"，不是官方 Unity 那趟。** 被控对象是替换过的同模型植物节点，
  与 Unity 内置动力学在数值上不会逐位相同。
- 跑的是**你提交的** `trajectory_generation_node` 与 lab3 `controller_node`（未经改动的 C++），
  这一点与官方链路一致。
- 建议在 `rviz_drone_race.txt` 里如实注明这一点（已预留说明位置）。
- 「官方 Unity 录屏」这条路本 kit **不覆盖** —— 那需要一台有桌面的 Linux x86_64，
  官方播放器可直接下载：<https://vnav.mit.edu/material/lab4.zip>（见 `../README.md` §8）。
