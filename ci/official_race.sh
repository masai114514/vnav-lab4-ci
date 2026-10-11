#!/usr/bin/env bash
# =============================================================================
# official_race.sh —— 在**原生 x86_64 Linux** 上一条龙跑通官方 Unity(TESSE) 竞速链路
#
# 产出（$OUT/）：
#   race_video.mp4     Unity 窗口录屏
#   race.bag           竞速 rosbag                    ← 交付物 [2]
#   metrics.json       位置 RMSE / 过门数 / 最大速度
#   race_tracking.png  实际 vs 期望轨迹
#   lag_probe.txt      ★核心诊断：控制器看到的 /current_state 到底有多旧
#   lag_probe.csv      同上，逐条原始样本
#   unity.log / bridge.log / traj_following.log / traj_gen.log / roscore.log
#   fps_*.png          左上角帧时浮层的定时截图（Unity 开发构建自带，用于核对帧率）
#
# 用法（容器内，root）：
#   OUT=$PWD/out BUNDLE=$PWD/lab4 bash ci/official_race.sh
#
# -----------------------------------------------------------------------------
# 三条设计取舍（都是上一轮踩出来的，改之前先读）
# -----------------------------------------------------------------------------
# 1) **不复用** headless/docker_in_container.sh 里那套 GitHub 镜像改写。
#    那套是为了绕过「macOS + colima 容器出网被墙」；GitHub Actions 的 runner
#    直连 github.com，镜像改写纯属多余的失败点（三个镜像探针全挂就会 die）。
#    但 nlopt 的 **jobserver 补丁必须保留** —— 它与网络无关：nlopt 的
#    ExternalProject 里 BUILD_COMMAND 是裸 `make`，会继承 catkin `make -j$JOBS`
#    留在 MAKEFLAGS 里的 jobserver FD，而 EP 是经 `cmake -P` 调起的、FD 已失效 →
#    `make[5]: *** write jobserver: Bad file descriptor. Stop.`
#
# 2) Unity 就绪用「**端口能连上**」判定，不用「睡 N 秒」。
#    上一轮写死 UNITY_BOOT_WAIT=45，Unity 还停在 `loading new scene...`，
#    桥接空等 → 整趟没有 bag、没有视频。这是那次失败的直接原因。
#
# 3) 失败**不静默**：关键门（Unity 端口、/current_state、bag 落盘）一律 die 非零退出。
#    上一轮用 `bash x.sh | tee log` 且脚本没有 -e，退出码被 tee 吃掉 →
#    空产物也判 success。（本仓的 workflow 已加 `set -o pipefail` 双保险。）
# =============================================================================

set -uo pipefail

# -----------------------------------------------------------------------------
# 参数（全部可用环境变量覆盖；workflow_dispatch 的 inputs 就是灌这些）
# -----------------------------------------------------------------------------
OUT="${OUT:-$PWD/out}"
BUNDLE="${BUNDLE:-$PWD/lab4}"

DISP="${DISP:-:99}"

# 渲染分辨率。llvmpipe 是纯软件光栅化，帧时名义上正比于像素数 —— 但上一轮实测
# 960x600 → 640x400（像素只剩 44%）帧时**几乎没变**（248 ms / 381 ms），说明瓶颈
# 是每帧固定开销而不是光栅化。所以这里保持 640x400 只为少占点 CPU，别指望提帧率。
RES="${RES:-640x400}"
XVFB_RES="${XVFB_RES:-$RES}"
CAPFPS="${CAPFPS:-10}"
UPSCALE="${UPSCALE:-}"

# 控制回路频率。官方 lab3 起点代码写死 1000 Hz，但 Unity/TESSE 的物理只有 ~50 Hz，
# 且每条命令都要过一次 socket 桥接。在 CPU 吃紧的 runner 上 1000 Hz 只是把 CPU
# 从渲染器手里抢走。controller_node 已把它开成参数（/hz）。
CTRL_HZ="${CTRL_HZ:-200}"

# 等 Unity 监听端口的超时 [s]。开发构建首次加载场景很慢，给足。
UNITY_BOOT_WAIT="${UNITY_BOOT_WAIT:-240}"
# 等桥接出第一条 /current_state 的超时 [s]
BRIDGE_WAIT="${BRIDGE_WAIT:-120}"
# 竞速墙钟兜底超时 [s]
RACE_TIMEOUT="${RACE_TIMEOUT:-900}"

# Unity 里无人机停在地面上，**按 W 才上电**（README §8 第 344 行的注释）。
# 注意用 `${ARM_KEY-w}` 而不是 `${ARM_KEY:-w}` —— 显式传空串要能真的表示「不发键」。
ARM_KEY="${ARM_KEY-w}"
ARM_DELAY="${ARM_DELAY:-5}"
UNITY_WIN_NAME="${UNITY_WIN_NAME:-lab4}"

# lag_probe 采样时长 [s]；<=0 表示不跑
LAG_DURATION="${LAG_DURATION:-120}"

# 录屏开关：BAG_ONLY=1 只出 bag（省 CPU，用于纯算法迭代）
BAG_ONLY="${BAG_ONLY:-0}"

LAB4_ZIP_URL="${LAB4_ZIP_URL:-https://vnav.mit.edu/material/lab4.zip}"

JOBS="${JOBS:-$(nproc)}"
SKIP_APT="${SKIP_APT:-0}"

# -----------------------------------------------------------------------------
# 日志
# -----------------------------------------------------------------------------
say()  { printf '\033[1;36m[race]\033[0m %s\n' "$*"; }
note() { printf '\033[1;35m[race]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[race]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[race] %s\033[0m\n' "$*" >&2; exit 1; }
indent() { sed 's/^/    /'; }

mkdir -p "$OUT"
[ -d "$BUNDLE/dependencies" ] || die "BUNDLE=$BUNDLE 下没有 dependencies/ —— 仓库布局不对（应为 <repo>/lab4/dependencies）"

# 后台进程登记在册，退出统一收尸
PIDS=()
killall_bg() {
  local p
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null || true; done
  pkill -f "lab4.x86_64" 2>/dev/null || true
  pkill -f "Xvfb $DISP"  2>/dev/null || true
  return 0
}
trap 'killall_bg' EXIT

# =============================================================================
# 1. 系统依赖
# =============================================================================
if [ "$SKIP_APT" != "1" ]; then
  say "apt：构建工具"
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq || warn "apt-get update 有告警"
  apt-get install -y -qq --no-install-recommends \
    build-essential cmake pkg-config git wget curl patch unzip ca-certificates \
    autoconf automake libtool \
    libeigen3-dev libyaml-cpp-dev \
    python3-numpy python3-matplotlib python3-pip python3-yaml python3-dev \
    || die "基础构建包装不上"

  say "apt：ROS 组件（ros-base 变体里没有 tf / tf2 / eigen_conversions）"
  apt-get install -y -qq --no-install-recommends \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs \
    ros-noetic-eigen-conversions ros-noetic-cmake-modules ros-noetic-roslib \
    ros-noetic-roslaunch ros-noetic-rosbag ros-noetic-roslint \
    || warn "补装 ROS 组件有告警，继续（缺什么下面 catkin 会报）"

  say "apt：桥接依赖 cv_bridge"
  apt-get install -y -qq --no-install-recommends \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    || warn "cv_bridge 装不上；桥接本身是纯 python 节点，多数情况仍能跑"

  say "apt：图形栈（Xvfb + Mesa llvmpipe + 截图/录屏 + 发键）"
  apt-get install -y -qq --no-install-recommends \
    xvfb x11-utils xdotool imagemagick ffmpeg \
    libgl1-mesa-dri libglx-mesa0 libglu1-mesa mesa-utils \
    libxcursor1 libxrandr2 libxinerama1 libxi6 libxss1 libasound2 \
    libnss3 libgbm1 libgconf-2-4 \
    || warn "部分图形包装不上 —— Unity 缺 .so 会体现在 unity.log 里，先继续"

  # 容器里没有声卡，给 ALSA 一个 null 设备，免得 Unity 一直刷音频报错
  printf 'pcm.!default { type null }\nctl.!default { type null }\n' > /root/.asoundrc

  command -v catkin >/dev/null 2>&1 || pip3 install -q catkin_tools || die "catkin_tools 装不上"
fi

# =============================================================================
# 2. catkin 工作区
#    布局照 headless/docker_in_container.sh：三个自有包 + dependencies 里的 6 个
#    catkin 包直接摊平进 src/；mav_comm / mav_trajectory_generation 的子目录
#    本身就是包，按目录拷。
# =============================================================================
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
WS="${WS:-$HOME/vnav_ws}"
say "重建工作区 $WS"
rm -rf "$WS"; mkdir -p "$WS/src"

for p in planner_pkg trajectory_generation_pkg controller_pkg; do
  cp -r "$BUNDLE/$p" "$WS/src/"
done
for p in catkin_simple eigen_catkin eigen_checks glog_catkin nlopt yaml_cpp_catkin; do
  cp -r "$BUNDLE/dependencies/$p" "$WS/src/"
done
for sub in "$BUNDLE"/dependencies/mav_comm/*/; do
  [ -f "$sub/package.xml" ] && cp -r "$sub" "$WS/src/"
done
for sub in "$BUNDLE"/dependencies/mav_trajectory_generation/*/; do
  [ -f "$sub/package.xml" ] || continue
  # mav_trajectory_generation_example 顶层的 CMakeLists 是 WARP 专用的，不带进来
  case "$(basename "$sub")" in mav_trajectory_generation_example) continue ;; esac
  cp -r "$sub" "$WS/src/"
done
# yaml_cpp_catkin 的 PATCH_COMMAND 用的是 ${CMAKE_SOURCE_DIR}/extra_version.patch：
# catkin_tools 下 CMAKE_SOURCE_DIR = 包目录（正确），catkin_make 下 = ws/src。
# 多放一份，两种构建器都过。
cp -n "$WS/src/yaml_cpp_catkin/extra_version.patch" "$WS/src/" 2>/dev/null || true

say "src/ 内容：" ; ls -1 "$WS/src" | indent

# --- nlopt jobserver 补丁（见文件头「设计取舍 1」） --------------------------
if [ -f "$WS/src/nlopt/CMakeLists.txt" ]; then
  sed -i "s#^  BUILD_COMMAND make\$#  BUILD_COMMAND env -u MAKEFLAGS -u MFLAGS make -j${JOBS}#" \
      "$WS/src/nlopt/CMakeLists.txt"
  sed -i "s#^DESTDIR=@NLOPT_INSTALL_PREFIX@ make install\$#DESTDIR=@NLOPT_INSTALL_PREFIX@ env -u MAKEFLAGS -u MFLAGS make -j1 install#" \
      "$WS/src/nlopt/make_install_nlopt.sh.in"
  note "nlopt jobserver 补丁："
  grep -nE "BUILD_COMMAND" "$WS/src/nlopt/CMakeLists.txt" | indent
fi

# --- 官方 TESSE 桥接 ---------------------------------------------------------
# 仓名是 MIT-TESSE/tesse-ros-bridge（不是 MIT-SPARK，一次就找错的那个）。
# 纯 python 节点 + 少量 srv，是 catkin 包，依赖 catkin_simple（上面已拷）与 cv_bridge。
say "clone tesse-ros-bridge"
git clone -q --depth 1 https://github.com/MIT-TESSE/tesse-ros-bridge.git \
    "$WS/src/tesse_ros_bridge" || die "clone tesse-ros-bridge 失败"
# 少数 tag 会把 catkin 包放在子目录里，兜一下
if [ ! -f "$WS/src/tesse_ros_bridge/package.xml" ]; then
  found="$(find "$WS/src/tesse_ros_bridge" -maxdepth 3 -name package.xml | head -1)"
  if [ -n "$found" ]; then
    warn "catkin 包不在仓根，提升 $(dirname "$found")"
    mv "$WS/src/tesse_ros_bridge" "$WS/src/_tesse_ros_bridge_tmp"
    mv "$(dirname "$found")" "$WS/src/tesse_ros_bridge"
    mv "$WS/src/_tesse_ros_bridge_tmp" "$WS/src/tesse_ros_bridge_src" 2>/dev/null || true
  fi
fi

# tesse-interface 提供 `tesse` 客户端库，是 setup.py 包，不进 catkin
say "clone tesse-interface"
if [ ! -d "$HOME/tesse-interface/.git" ]; then
  git clone -q --depth 1 https://github.com/MIT-TESSE/tesse-interface.git \
      "$HOME/tesse-interface" || warn "clone tesse-interface 失败（若桥接不需要它可忽略）"
fi
if [ -f "$HOME/tesse-interface/setup.py" ]; then
  ( cd "$HOME/tesse-interface" && pip3 install -q -e . ) || warn "pip install tesse-interface 失败"
fi

# --- 构建 --------------------------------------------------------------------
cd "$WS" || die "cd $WS 失败"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release || die "catkin config 失败"
say "catkin build -j$JOBS（重点看 glog 0.3.5 的 autoreconf 与 nlopt 的下载）"
if ! catkin build -j"$JOBS" --no-status; then
  warn "catkin build 失败。常见原因："
  warn "  · glog：缺 autoconf/automake/libtool/patch"
  warn "  · nlopt：下载被限流（重跑一次通常就好）"
  warn "  · 单包重来：catkin build <pkg> -j1 --verbose"
  die "构建未通过"
fi
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"
rospack find trajectory_generation >/dev/null || die "找不到 trajectory_generation 包"
rospack find planner_pkg          >/dev/null || die "找不到 planner_pkg 包"
rospack find controller_pkg       >/dev/null || die "找不到 controller_pkg 包"
rospack find tesse_ros_bridge     >/dev/null || warn "找不到 tesse_ros_bridge 包 —— 桥接启动会失败"
note "工作区就绪：$WS"

# =============================================================================
# 3. 官方 Unity 播放器
# =============================================================================
if [ -z "${UNITY_BIN:-}" ]; then
  if [ ! -s "$OUT/lab4.zip" ]; then
    say "下载官方 lab4.zip（$LAB4_ZIP_URL）"
    curl -fL --retry 3 --retry-delay 5 --max-time 600 \
         -o "$OUT/lab4.zip" "$LAB4_ZIP_URL" || die "下载 lab4.zip 失败"
  fi
  say "解包 → $OUT/unity"
  rm -rf "$OUT/unity"; mkdir -p "$OUT/unity"
  unzip -q -o "$OUT/lab4.zip" -d "$OUT/unity" || die "解包 lab4.zip 失败（不是 zip？）"
  UNITY_BIN="$(find "$OUT/unity" -maxdepth 3 -type f -name '*.x86_64' | head -1)"
fi
[ -n "$UNITY_BIN" ] || die "解包后找不到 *.x86_64"
chmod +x "$UNITY_BIN" 2>/dev/null || true
[ -x "$UNITY_BIN" ] || die "$UNITY_BIN 不可执行"
UNITY_DIR="$(dirname "$UNITY_BIN")"
UNITY_NAME="$(basename "$UNITY_BIN")"
UNITY_DATA="$(find "$UNITY_DIR" -maxdepth 1 -type d -name '*_Data' | head -1)"
say "Unity  : $UNITY_BIN"
say "Data   : $UNITY_DATA"
find "$UNITY_DIR" -maxdepth 1 | indent

# 门位姿 CSV 住在构建的 _Data/StreamingAssets/ 里（planner_pkg 就是去那儿读）
SIM_DATA="${SIM_DATA:-$UNITY_DATA}"
if [ ! -f "$SIM_DATA/StreamingAssets/vnav-2020-lab4_static_tfs.csv" ]; then
  warn "$SIM_DATA/StreamingAssets 下没有 vnav-2020-lab4_static_tfs.csv"
  SIM_DATA="$BUNDLE/headless/sim_data"
  warn "回落到仓内副本：$SIM_DATA"
fi
say "门位姿目录（simulator_data_directory）: $SIM_DATA"

# =============================================================================
# 4. 虚拟显示 + 启动 Unity
# =============================================================================
export DISPLAY="$DISP"
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export LP_NUM_THREADS="$JOBS"

pkill -f "Xvfb $DISP" 2>/dev/null || true
say "启动 Xvfb $DISP（${XVFB_RES}x24）"
Xvfb "$DISP" -screen 0 "${XVFB_RES}x24" -ac +extension GLX +render -noreset \
    > "$OUT/xvfb.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 30); do
  xdpyinfo -display "$DISP" >/dev/null 2>&1 && break
  sleep 1
done
xdpyinfo -display "$DISP" >/dev/null 2>&1 || die "Xvfb $DISP 起不来，见 $OUT/xvfb.log"
note "Xvfb 就绪"

# 留一条「GL 上下文确实是软件渲染」的书面证据
if command -v glxinfo >/dev/null 2>&1; then
  glxinfo -display "$DISP" -B 2>/dev/null \
    | grep -iE "OpenGL (renderer|version)" | indent || true
fi

# 容器默认不带 CAP_SYS_NICE，`nice -n -5` 会 EPERM。先探一下再决定带不带。
NICE=()
if nice -n -5 true 2>/dev/null; then
  NICE=(nice -n -5)
else
  warn "无法降 niceness（容器缺 CAP_SYS_NICE）—— Unity 不加优先级"
fi

say "启动 Unity（窗口 ${RES}，force-glcore）"
( cd "$UNITY_DIR" && exec "${NICE[@]}" "./$UNITY_NAME" \
    -logFile "$OUT/unity.log" \
    -screen-fullscreen 0 -screen-width "${RES%x*}" -screen-height "${RES#*x}" \
    -force-glcore \
    < /dev/null > "$OUT/unity_stdout.log" 2>&1 ) &
PIDS+=("$!")

# 就绪判据：Unity 的服务端端口能连上（而不是「睡 N 秒」）
wait_port() {
  local port="$1" timeout="$2"
  local end=$((SECONDS + timeout))
  while [ "$SECONDS" -lt "$end" ]; do
    if (exec 3<>"/dev/tcp/127.0.0.1/$port") 2>/dev/null; then return 0; fi
    pgrep -f "$UNITY_NAME" >/dev/null 2>&1 || return 2   # 进程没了就别等
    sleep 2
  done
  return 1
}

say "等 Unity 监听 9001（服务端元数据口，最长 ${UNITY_BOOT_WAIT}s）"
wait_port 9001 "$UNITY_BOOT_WAIT"
UP_RC=$?
if [ "$UP_RC" = 2 ]; then
  warn "Unity 进程已退出。unity.log 尾部："
  tail -40 "$OUT/unity.log" 2>/dev/null | indent || true
  warn "unity_stdout.log 尾部："
  tail -20 "$OUT/unity_stdout.log" 2>/dev/null | indent || true
  die "Unity 启动失败"
fi
if [ "$UP_RC" != 0 ]; then
  warn "等满 ${UNITY_BOOT_WAIT}s 仍未监听 9001。unity.log 尾部："
  tail -40 "$OUT/unity.log" 2>/dev/null | indent || true
  die "Unity 没起来（可调大 UNITY_BOOT_WAIT 重试）"
fi
note "Unity 已就绪 —— 帧率取证行（事件里的 Display / OPENGL LOG）："
grep -aoE "Display [0-9]+ '[^']*': [0-9]+x[0-9]+|Creating OpenGL [0-9.]+ graphics device" \
     "$OUT/unity.log" 2>/dev/null | head -4 | indent || true

# =============================================================================
# 5. ROS 桥接
# =============================================================================
say "启动 roscore"
roscore -p 11311 > "$OUT/roscore.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 40); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
rostopic list >/dev/null 2>&1 || die "roscore 起不来，见 $OUT/roscore.log"

# Unity 发 /clock，整条链路必须走仿真时间（控制器与轨迹节点都按 ROS 时间采样）；
# lag_probe 也只有在 use_sim_time=true 下才能把「仿真时间戳」和「当前仿真时刻」相减。
rosparam set /use_sim_time true
say "控制回路频率 /hz = $CTRL_HZ（官方起点是 1000，这里按 runner 的 CPU 余量降）"
rosparam set /hz "$CTRL_HZ"

say "roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch"
# use_broadcast 保持官方默认 true。
# tesse_ros_node.py 里写的是 `if self.use_broadcast: udp_host = '<broadcast>'` ——
# 改成 false 会把 UDP 监听绑到 self_ip:9004 的**单播**地址，而 Unity 侧是往广播
# 地址发 metadata 的 → 直接丢 /current_state，整条链路废掉。
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch \
    > "$OUT/bridge.log" 2>&1 &
PIDS+=("$!")

say "等第一条 /current_state（最长 ${BRIDGE_WAIT}s）"
ok=0
for _ in $(seq 1 $((BRIDGE_WAIT / 2))); do
  if timeout 3 rostopic echo -n1 /current_state >/dev/null 2>&1; then ok=1; break; fi
  sleep 2
done
if [ "$ok" != 1 ]; then
  warn "桥接没发出 /current_state。诊断："
  { echo "--- rosnode list";   rosnode list  2>&1 | indent
    echo "--- 相关话题";       rostopic list 2>&1 | grep -E "current|clock|rotor|imu" | indent
    echo "--- bridge.log 尾部"; tail -30 "$OUT/bridge.log" 2>/dev/null | indent
    echo "--- unity.log 尾部";  tail -20 "$OUT/unity.log"  2>/dev/null | indent; } || true
  die "桥接未连通"
fi
note "/current_state 已到 —— 官方链路通了"

# =============================================================================
# 6. 上电（README §8：「按 W 上电」）
# =============================================================================
if [ -n "$ARM_KEY" ]; then
  sleep "$ARM_DELAY"
  say "向 Unity 发上电键 '$ARM_KEY'"
  WID="$(xdotool search --name "$UNITY_WIN_NAME" 2>/dev/null | head -1)"
  if [ -n "$WID" ]; then
    xdotool windowfocus "$WID" 2>/dev/null || true
    note "  窗口 id = $WID"
  else
    warn "  xdotool 没找到名字含 '$UNITY_WIN_NAME' 的窗口，直接对焦点窗口发键"
  fi
  xdotool key --clearmodifiers "$ARM_KEY" 2>/dev/null || warn "  xdotool 发键失败"
  sleep 2
else
  note "ARM_KEY 为空 —— 不发上电键"
fi

# =============================================================================
# 7. 起控制器 → 开录 → 发门位姿（竞速开始）
# =============================================================================
say "roslaunch trajectory_generation traj_following.launch（trajectory_generation_node + controller_node）"
roslaunch trajectory_generation traj_following.launch > "$OUT/traj_following.log" 2>&1 &
PIDS+=("$!")
sleep 5

say "开始录 rosbag → $OUT/race.bag"
rosbag record -O "$OUT/race.bag" /current_state /desired_state /rotor_speed_cmds \
    > "$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
PIDS+=("$BAG_PID")
sleep 3

FFMPEG_PID=""
if [ "$BAG_ONLY" = "1" ]; then
  note "BAG_ONLY=1 —— 跳过录屏（省 CPU）"
else
  say "开始录屏 → $OUT/race_video.mp4（${CAPFPS} fps @ $RES）"
  ffmpeg -y -loglevel warning -f x11grab -framerate "$CAPFPS" \
      -video_size "$RES" -i "$DISP" \
      -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -an \
      "$OUT/race_video.mp4" > "$OUT/ffmpeg.log" 2>&1 &
  FFMPEG_PID=$!
  PIDS+=("$FFMPEG_PID")
fi

# Unity 开发构建左上角固定画着帧时浮层 —— 定时截图留证（事后可核对真实帧率）
say "启动帧时浮层定时截图（每 ${SHOT_GAP:-20}s 一张）"
( i=0
  while [ "$i" -lt "${SHOT_COUNT:-48}" ]; do
    sleep "${SHOT_GAP:-20}"; i=$((i + 1))
    import -window root "$OUT/shot_t$(printf '%03d' "$i").png" 2>/dev/null || continue
    convert "$OUT/shot_t$(printf '%03d' "$i").png" -crop 320x44+0+0 +repage \
            -filter point -resize 300% "$OUT/fps_$(printf '%03d' "$i").png" 2>/dev/null || true
  done ) &
PIDS+=("$!")

if [ "${LAG_DURATION%.*}" -gt 0 ] 2>/dev/null; then
  say "并行跑 lag_probe（${LAG_DURATION}s）—— 量 /current_state 真实滞后"
  python3 "$BUNDLE/headless/lag_probe.py" --duration "$LAG_DURATION" \
      --out "$OUT/lag_probe" > "$OUT/lag_probe.log" 2>&1 &
  PIDS+=("$!")
fi

say "roslaunch planner_pkg traj_gen.launch（发门位姿 → 竞速开始）"
roslaunch planner_pkg traj_gen.launch simulator_data_directory:="$SIM_DATA" \
    > "$OUT/traj_gen.log" 2>&1 &
PIDS+=("$!")

say "等竞速结束（plateau 判据：/desired_state 停止推进；墙钟兜底 ${RACE_TIMEOUT}s）"
python3 "$BUNDLE/headless/wait_race_end.py" --mode plateau --timeout "$RACE_TIMEOUT"
WAIT_RC=$?
[ "$WAIT_RC" = 0 ] || warn "竞速结束判定返回 $WAIT_RC（1=超时 2=中断），仍收尾并分析"

# =============================================================================
# 8. 收尾
# =============================================================================
sleep 2
say "停止录 bag / 录屏"
[ -n "${BAG_PID:-}" ]    && kill -INT "$BAG_PID"    2>/dev/null || true
[ -n "${FFMPEG_PID:-}" ] && kill -INT "$FFMPEG_PID" 2>/dev/null || true
sleep 4
[ -n "${BAG_PID:-}" ]    && kill "$BAG_PID"    2>/dev/null || true
[ -n "${FFMPEG_PID:-}" ] && kill "$FFMPEG_PID" 2>/dev/null || true
sleep 1

BAGF="$OUT/race.bag"
[ -f "$BAGF" ] || BAGF="$OUT/race.bag.active"
if [ -f "$BAGF" ]; then
  say "bag: $BAGF ($(du -h "$BAGF" | cut -f1))"
else
  warn "没有生成 bag —— 看 $OUT/rosbag.log"
fi

# =============================================================================
# 9. 分析
# =============================================================================
if [ -f "$BAGF" ]; then
  rosbag info "$BAGF" > "$OUT/bag_info.txt" 2>&1 || true
  say "rosbag info"
  head -20 "$OUT/bag_info.txt" 2>/dev/null | indent || true

  say "analyze_race_bag.py（位置 RMSE / 过门数 / 最大速度）"
  if python3 "$BUNDLE/headless/analyze_race_bag.py" --bag "$BAGF" \
        --out "$OUT/metrics.json" --plot "$OUT/race_tracking.png" \
        > "$OUT/analyze.log" 2>&1; then
    cat "$OUT/analyze.log" | indent
  else
    warn "分析失败，见 $OUT/analyze.log（bag 本身是好的，可以本地再分析）"
  fi

  say "osc_report.py（xyz 逐轴误差 / 滑动 RMS 包络 / roll·pitch）"
  if python3 "$BUNDLE/headless/osc_report.py" --bag "$BAGF" \
        --out "$OUT/osc_report.txt" --csv "$OUT/osc_samples.csv" \
        > "$OUT/osc.log" 2>&1; then
    cat "$OUT/osc_report.txt" | indent
  else
    warn "osc_report 失败，见 $OUT/osc.log"
  fi
fi

if [ -s "$OUT/lag_probe.txt" ]; then
  say "★ /current_state 滞后实测（lag_probe）"
  cat "$OUT/lag_probe.txt" | indent
else
  warn "没有 lag_probe.txt —— 延迟这一条结论会缺"
fi

# =============================================================================
# 10. 交付：上采样 + 清单
# =============================================================================
if [ -n "$UPSCALE" ] && [ -f "$OUT/race_video.mp4" ] && [ "$UPSCALE" != "$RES" ]; then
  say "上采样视频 $RES → $UPSCALE"
  if ffmpeg -y -loglevel warning -i "$OUT/race_video.mp4" \
       -vf "scale=${UPSCALE}:flags=lanczos" -c:v libx264 -preset fast -crf 20 \
       -pix_fmt yuv420p -an "$OUT/race_video_full.mp4"; then
    say "上采样版: $OUT/race_video_full.mp4 ($(du -h "$OUT/race_video_full.mp4" | cut -f1))"
  else
    warn "上采样失败，保留原片"
  fi
fi

say "产出清单 $OUT"
ls -la "$OUT" | indent

MISSING=0
for f in race.bag metrics.json; do
  if [ ! -f "$OUT/$f" ]; then warn "缺 $f"; MISSING=1; fi
done
[ -f "$OUT/race_video.mp4" ] || warn "缺 race_video.mp4（交付视频需要它）"

if [ "$MISSING" != 0 ]; then
  die "关键产物缺失 —— 这趟不算成功"
fi
printf '\033[1;32m[race] 全部完成。产物在 %s\033[0m\n' "$OUT"
