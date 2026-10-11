#!/usr/bin/env bash
# =============================================================================
# run_race_headless.sh — 一条命令跑完 LAB4 竞速（无头）并录出官方要求的 rosbag
#
#   bash run_race_headless.sh              # 产出到 ~/race_out/
#   OUT=/tmp/run1 bash run_race_headless.sh
#
# 干的事：
#   1) 设 /use_sim_time=true
#   2) 起替身植物节点（推 /clock + /current_state）—— 替代 Unity(TESSE)
#   3) 起官方 traj_following.launch（trajectory_generation_node + controller_node）
#   4) 开始 rosbag record（**必须在门位姿发布之前**，否则漏掉起飞段）
#   5) 起官方 traj_gen.launch（traj_vertices_publisher 发门位姿 → 竞速开始）
#   6) 等 /desired_state 停止推进 → 停录 → 自动分析
#
# 为什么用仿真时间
#   trajectory_generation_node 按 ROS 时间采样轨迹。植物节点在纯 Python 里很难跑满
#   实时 1 kHz，若不接管时间，"按墙钟前进的期望轨迹"会把无人机甩开，结果不可信。
#   推 /clock 后整条链路（含 bag 时间戳）都跑在仿真时间上，墙钟快慢只影响录制耗时。
# =============================================================================
set -o pipefail

HEADLESS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$HOME/vnav_ws}"
OUT="${OUT:-$HOME/race_out}"
BAG="$OUT/race.bag"

PLANT_RATE="${PLANT_RATE:-1000}"     # 积分/发布频率 [Hz]（仿真时间），与离线仿真 dt=1ms 对齐
PLANT_REALTIME="${PLANT_REALTIME:-1}" # 1 = 按 1:1 墙钟节流（**别关！**）
MOTOR_TAU="${MOTOR_TAU:-0.03}"
RACE_TIMEOUT="${RACE_TIMEOUT:-2400}" # 墙钟兜底超时 [s]

# 为什么必须 realtime=1
# ---------------------
# 植物节点若以"尽可能快"推进仿真时间（realtime=0），仿真时间会比节点实际处理能力跑得
# 快（实测 ~1.2× 墙钟）。roscpp 单线程回调队列（/current_state 1 kHz + 控制定时器
# 1 kHz）消化不完，积压会**持续累积**，表现为控制动作的等效延迟不断增大。
# 实测对照（同一条期望轨迹，离线复算）：
#     延迟 0–10 ms → 位置 RMSE ~270 mm，峰值速度 2.4 m/s
#     延迟    30 ms → 位置 RMSE  611 mm，峰值速度 7.7 m/s
#     实跑（无节流）→ 位置 RMSE  861 mm，峰值速度 8.4 m/s   ← 明显被延迟主导
# 按 1:1 墙钟节流后，仿真时间最多与墙钟同速，回调负载被钉在 1 kHz 以内，就不会积压。
# 植物跑不满 1 kHz 也不要紧 —— 仿真时间只会变慢，结果不变（墙钟快慢不影响仿真结果）。

say()  { printf '\033[1;36m[race]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[race]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[race] %s\033[0m\n' "$*" >&2; exit 1; }

mkdir -p "$OUT"

PIDS=()
kill_pid() { [ -n "${1:-}" ] && kill "$1" 2>/dev/null || true; }

cleanup() {
  warn "清理节点 ..."
  for p in "${PIDS[@]:-}"; do kill_pid "$p"; done
  pkill -f plant_node.py               2>/dev/null || true
  pkill -f trajectory_generation_node  2>/dev/null || true
  pkill -f controller_node             2>/dev/null || true
  pkill -f traj_vertices_publisher     2>/dev/null || true
  pkill -f "rosbag record"             2>/dev/null || true
}
trap cleanup EXIT

# -----------------------------------------------------------------------------
say "HEADLESS_DIR = $HEADLESS_DIR"
say "WS           = $WS"
say "OUT          = $OUT"

# ROS 环境：优先用 $ROS_SETUP 指定的；否则依次探测 /opt/ros/noetic、报错。
# （Ubuntu 22.04 上没有官方 Noetic deb，用 RoboStack 时 ROS 在 conda 环境里：
#   export ROS_SETUP=/root/miniconda3/envs/ros/setup.bash）
if [ -z "${ROS_SETUP:-}" ]; then
  for cand in /opt/ros/noetic/setup.bash "$HOME/miniconda3/envs/ros/setup.bash" \
              "$CONDA_PREFIX/setup.bash"; do
    [ -f "$cand" ] && { ROS_SETUP="$cand"; break; }
  done
fi
[ -n "${ROS_SETUP:-}" ] && [ -f "$ROS_SETUP" ] || \
  die "找不到 ROS 的 setup.bash。官方 deb 装法请先跑 setup_autodl.sh；RoboStack 装法请 export ROS_SETUP=<conda env>/setup.bash"
say "ROS_SETUP    = $ROS_SETUP"
# shellcheck disable=SC1091
source "$ROS_SETUP"
[ -f "$WS/devel/setup.bash" ] || die "找不到 $WS/devel/setup.bash —— 先 catkin build"
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

# 用哪个 python（RoboStack 下要显式指向 conda env 的 python，才有 rospy/numpy）
PY="${PYTHON:-python3}"
say "PYTHON       = $PY  ($("$PY" -c 'import sys;print(sys.version.split()[0])' 2>/dev/null || echo '不可用'))"

# -----------------------------------------------------------------------------
say "清理上次残留 ..."
cleanup
sleep 1

# -----------------------------------------------------------------------------
if rostopic list >/dev/null 2>&1; then
  say "已有 ROS master，复用"
else
  say "启动 roscore"
  roscore -p 11311 > "$OUT/roscore.log" 2>&1 &
  PIDS+=("$!")
  for _ in $(seq 1 40); do
    rostopic list >/dev/null 2>&1 && break
    sleep 0.5
  done
  rostopic list >/dev/null 2>&1 || die "roscore 起不来，见 $OUT/roscore.log"
fi

# -----------------------------------------------------------------------------
say "设 /use_sim_time = true"
rosparam set /use_sim_time true

# -----------------------------------------------------------------------------
say "启动替身植物节点 (rate=$PLANT_RATE Hz, motor_tau=$MOTOR_TAU, realtime=$PLANT_REALTIME)"
if [ "$PLANT_REALTIME" = "1" ]; then RT_FLAG="--realtime"; else RT_FLAG=""; fi
# shellcheck disable=SC2086
"$PY" "$HEADLESS_DIR/plant_node.py" --rate "$PLANT_RATE" --motor-tau "$MOTOR_TAU" $RT_FLAG \
  > "$OUT/plant.log" 2>&1 &
PIDS+=("$!")

say "等待 /clock ..."
CLOCK_OK=0
for _ in $(seq 1 40); do
  if timeout 3 rostopic echo -n1 /clock >/dev/null 2>&1; then CLOCK_OK=1; break; fi
  sleep 0.5
done
[ "$CLOCK_OK" = 1 ] || die "植物节点没发出 /clock —— 看 $OUT/plant.log"
say "/clock 正常，仿真时间已由植物节点接管"

# 再确认 /current_state 已经有消息（轨迹节点的起点依赖它，否则起点是未初始化的垃圾值）
say "等待 /current_state ..."
for _ in $(seq 1 40); do
  if timeout 3 rostopic echo -n1 /current_state >/dev/null 2>&1; then break; fi
  sleep 0.5
done

# -----------------------------------------------------------------------------
say "启动 traj_following.launch（trajectory_generation_node + controller_node）"
roslaunch trajectory_generation traj_following.launch \
  > "$OUT/traj_following.log" 2>&1 &
PIDS+=("$!")
sleep 3

# -----------------------------------------------------------------------------
say "开始录制 rosbag → $BAG"
rosbag record -O "$BAG" /current_state /desired_state \
  > "$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
PIDS+=("$BAG_PID")
sleep 2

# -----------------------------------------------------------------------------
say "启动 traj_gen.launch（发布 /desired_traj_vertices，竞速开始）"
roslaunch planner_pkg traj_gen.launch \
  simulator_data_directory:="$HEADLESS_DIR/sim_data" \
  > "$OUT/traj_gen.log" 2>&1 &
PIDS+=("$!")

# -----------------------------------------------------------------------------
say "等待竞速结束（监视 /desired_state 是否停止推进）..."
"$PY" "$HEADLESS_DIR/wait_race_end.py" --mode plateau --timeout "$RACE_TIMEOUT"
WAIT_RC=$?
[ "$WAIT_RC" = 0 ] || warn "竞速结束判定返回 $WAIT_RC（1=超时 2=中断），仍会收尾并尝试分析。"

sleep 2
say "停止录制"
kill -INT "$BAG_PID" 2>/dev/null || true
sleep 3
kill_pid "$BAG_PID"
sleep 1

BAGF="$BAG"
[ -f "$BAGF" ] || BAGF="$BAG.active"
if [ ! -f "$BAGF" ]; then
  die "没有生成 bag —— 看 $OUT/rosbag.log"
fi
say "bag: $BAGF  ($(du -h "$BAGF" | cut -f1))"

# -----------------------------------------------------------------------------
say "分析 rosbag ..."
if "$PY" "$HEADLESS_DIR/analyze_race_bag.py" --bag "$BAGF" \
     --out "$OUT/metrics.json" --plot "$OUT/race_tracking.png" \
     > "$OUT/analyze.log" 2>&1; then
  cat "$OUT/analyze.log"
else
  warn "分析失败，见 $OUT/analyze.log（bag 本身是好的，可以自己看）"
fi

# -----------------------------------------------------------------------------
cat <<EOF

$(printf '\033[1;32m')============================================================$(printf '\033[0m')
 完成。产出目录：$OUT

   race.bag            提交用 rosbag（可用 rosbag info 查看）
   metrics.json        指标（竞速时长 / RMSE / 最大速度 / 过门数）
   race_tracking.png   实际 vs 期望轨迹
   plant.log           植物节点日志
   traj_following.log  轨迹生成 + 控制器日志（"Traversed x%" 在这里）
   traj_gen.log        门位姿发布日志（"Finished loading trajectory csv: 12" 在这里）
   rosbag.log          录制日志

 提交前把 bag 拷回本机，填进 lab4/rviz_drone_race.txt。
 跑完记得关机：/usr/bin/shutdown
$(printf '\033[1;32m')============================================================$(printf '\033[0m')
EOF
