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
# 2) Unity 就绪读 **Unity 自己日志里的监听器标记**，既不是「睡 N 秒」，
#    也**不是**「TCP 探端口」——后者是 2026-10-11 Run 3 那次假故障的元凶：
#    官方 launch 的端口分配是 position=9000 / metadata=9001 / image=9002 /
#    udp(控制)=9004 / step=9005 / lidar=9006 / lidar_udp=9007，而
#    **metadata 与 position 在 TESSE 里都是 UDP**，UDP 没有 listen 状态，
#    `/dev/tcp/127.0.0.1/9001` 永远连不上 → Unity 早已 ready（它自己的日志里
#    "Image request listener started" 与 "Metadata request listener started..."
#    都打出来了）却被判成「没起来」，整趟 die 在起跑线上。
#    （更早一轮是写死 UNITY_BOOT_WAIT=45，Unity 还停在 `loading new scene...`，
#      桥接空等 → 没有 bag 没有视频。两次都不是「等得不够久」，是判据错。）
#
# 3) 失败**不静默**：关键门（Unity 就绪、/current_state、bag 落盘）一律 die 非零退出。
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

# 等 Unity 的 TESSE 监听器就绪（读 unity.log 标记）的超时 [s]。开发构建首次加载场景很慢，给足。
UNITY_BOOT_WAIT="${UNITY_BOOT_WAIT:-240}"
# 监听器起了 ≠ 场景已经在跑。给物理/渲染一小段起步时间再上桥接。
UNITY_SETTLE="${UNITY_SETTLE:-8}"
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

# 桥接是否发 IMU。launch 里默认 true（200 Hz）。我们的 controller_node 只订阅
# /current_state 与 /desired_state，**从不读 IMU**，所以默认关掉：
# 200 Hz 的 UDP 收包 + 反序列化 + 话题发布在 4 核机器上是白烧 CPU。
# 想完全照官方原样跑就传 PUBLISH_IMU=true。
PUBLISH_IMU="${PUBLISH_IMU:-false}"

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

# 本脚本跑在 `set -u`（nounset）下，但 ROS 的 setup.bash 会连带 source 一批
# /opt/ros/noetic/etc/catkin/profile.d/*.sh，里面**有对未定义变量的引用**：
#     .../profile.d/10.roslaunch.sh: line 3: ROS_MASTER_URI: unbound variable
# 于是 source 当场中止、脚本一步都没跑。实测就是这么挂的。
# 所以凡 source ROS 的环境，一律临时关掉 nounset 再开回来。
ros_source() {
  set +u
  # shellcheck disable=SC1090,SC1091
  . "$1"
  set -u
}

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

  say "apt：桥接依赖 cv_bridge / ackermann_msgs / message_generation / tf2_ros"
  # 逐个对着 tesse_ros_node.py 的 import 核过：
  #   rospy tf tf2_ros std_msgs sensor_msgs nav_msgs geometry_msgs
  #   ackermann_msgs rosgraph_msgs cv_bridge tesse + 自生成的 msg/srv
  # 注意 tf2 与 tf2_ros 是**两个包**，只装前者会 import tf2_ros 失败。
  apt-get install -y -qq --no-install-recommends \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-ackermann_msgs ros-noetic-message-generation ros-noetic-message-runtime \
    || warn "桥接依赖有告警；缺哪个 catkin build / 启动时都会点出来"

  say "apt：桥接 python 侧的 scipy / opencv / empy"
  # ROS/setup.py 的 install_requires 声明了 tesse / empy / opencv-python / scipy，
  # 但 catkin 的 setup.py 不会真的去 pip 装它们，得我们自己备齐。
  apt-get install -y -qq --no-install-recommends \
    python3-scipy python3-opencv python3-empy python3-defusedxml \
    || warn "部分 python 包装不上，继续"

  say "apt：图形栈（Xvfb + Mesa llvmpipe + 截图/录屏 + 发键）"
  apt-get install -y -qq --no-install-recommends \
    xvfb x11-utils xdotool imagemagick ffmpeg \
    libgl1-mesa-dri libglx-mesa0 libglu1-mesa mesa-utils \
    libxcursor1 libxrandr2 libxinerama1 libxi6 libxss1 libasound2 \
    libnss3 libgbm1 libgconf-2-4 \
    || warn "部分图形包装不上 —— Unity 缺 .so 会体现在 unity.log 里，先继续"

  # 容器里没有声卡，给 ALSA 一个 null 设备，免得 Unity 一直刷音频报错。
  # 注意用 $HOME —— GitHub 的容器 job 把 HOME 设成 /github/home，不是 /root。
  # （上一步写死 /root/.asoundrc 就是错的。）
  printf 'pcm.!default { type null }\nctl.!default { type null }\n' > "$HOME/.asoundrc"

  # ---------------------------------------------------------------------------
  # `python` 解释器 —— Run 5（38126738778）就是死在这，exit code 127
  # ---------------------------------------------------------------------------
  # 桥接的 py 脚本 shebang 是 `#!/usr/bin/env python`（Py2 时代的写法）。
  # ROS Noetic 本体是 py3，但**这两个脚本的 shebang 没人改过**；而 Ubuntu 20.04
  # 默认只装 python3，`python` 这个命令**根本不存在**，于是 roslaunch 起节点时：
  #     /usr/bin/env: 'python': No such file or directory
  #     [tesse/tesse_ros_bridge-1] process has died [pid -1, exit code 127]
  # 桥接两个节点当场全灭 → /current_state 永远不来 → 整趟没有任何产物。
  # `python-is-python3` 是 ROS Noetic 的官方配套，提供 /usr/bin/python → python3。
  say "apt：python-is-python3（/usr/bin/python → python3 的链接）"
  apt-get install -y -qq --no-install-recommends python-is-python3 \
    || warn "python-is-python3 装不上 —— 下面手动建链接兜底"

  # 双保险：个别镜像没这个包，或被 --no-install-recommends 掉。
  if ! command -v python >/dev/null 2>&1; then
    warn "系统里没有 python —— 手动链接到 python3（Noetic 本体就是 py3）"
    ln -sf "$(command -v python3)" /usr/local/bin/python \
      || die "建 python 链接失败"
  fi
  command -v python >/dev/null 2>&1 || die "没有 python 解释器，桥接必然起不来"
  note "python → $(command -v python)  [$(python -V 2>&1)]"

  command -v catkin >/dev/null 2>&1 || pip3 install -q catkin_tools || die "catkin_tools 装不上"
fi

# =============================================================================
# 2. catkin 工作区
#    布局照 headless/docker_in_container.sh：三个自有包 + dependencies 里的 6 个
#    catkin 包直接摊平进 src/；mav_comm / mav_trajectory_generation 的子目录
#    本身就是包，按目录拷。
# =============================================================================
# shellcheck disable=SC1091
ros_source /opt/ros/noetic/setup.bash
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

# --- 官方 TESSE 桥接（ROS1 版）------------------------------------------------
# 来源是 MIT-SPARK/VNAV-labs 的 **tag `VNAV2021`**，不是 MIT-TESSE/tesse-ros-bridge。
# 为什么（2026-10-11 实测查清，别再改回去）：
#   · MIT-TESSE/tesse-ros-bridge 的 master 已经是 **ROS 2** 分支 ——
#     launch 用 `exec:` / `find-pkg-share` / `remap: from..to`，与我们 ROS1 的
#     catkin/roslaunch 完全不兼容，而且里面已经没有 quadrotor_control_interface 了。
#   · 课程 ROS1 那一版（2021 年）躺在 VNAV-labs 的 `VNAV2021` tag 里：
#         lab3/tesse-ros-bridge/ROS/             ← catkin 包 `tesse_ros_bridge`
#         lab3/tesse-ros-bridge/tesse-interface/ ← `tesse` python 客户端库
#     我们的 Unity 构建叫 `vnav-2021-lab4.x86_64`，正是这一代。
#   · 顺带核过：该 tag 下 `lab4/dependencies/` 的 8 个包与我们本地**逐项一致**。
VNAV_TAG="${VNAV_TAG:-VNAV2021}"
VNAV_URL="${VNAV_URL:-https://github.com/MIT-SPARK/VNAV-labs.git}"
VNAV_DIR="${VNAV_DIR:-$HOME/vnav-labs}"
BRIDGE_SRC="$VNAV_DIR/lab3/tesse-ros-bridge"

if [ ! -d "$VNAV_DIR/.git" ]; then
  say "clone VNAV-labs @ $VNAV_TAG（约 2.5 MB）"
  git clone -q --depth 1 --branch "$VNAV_TAG" "$VNAV_URL" "$VNAV_DIR" \
    || die "clone VNAV-labs 失败（tag $VNAV_TAG 不存在？）"
fi
[ -f "$BRIDGE_SRC/ROS/package.xml" ] || \
  die "在 $BRIDGE_SRC/ROS 下找不到 package.xml —— tag 内容变了？"

say "拷贝 catkin 包 tesse_ros_bridge"
cp -r "$BRIDGE_SRC/ROS" "$WS/src/tesse_ros_bridge"
# 清掉 py2 时代的 .pyc（对 py3 无用，还会干扰 rospack 的可执行文件搜索）
find "$WS/src/tesse_ros_bridge" -name '*.pyc' -delete 2>/dev/null || true
# ROS/setup.py 里 packages=['tesse_ros_bridge', 'key_teleop']，但上游的
# src/key_teleop/ 只有 LICENSE 与 key_teleop.py，**没有 __init__.py**。
# setuptools 对「显式列出的包目录缺 __init__.py」会告警（新版可能直接失败），
# 补一个空文件最省事。
KEYINIT="$WS/src/tesse_ros_bridge/src/key_teleop/__init__.py"
[ -f "$KEYINIT" ] || : > "$KEYINIT"
# launch 用的是 type="tesse_ros_node.py" —— 脚本名。ROS1 靠 rospack 在包目录里
# **递归找可执行文件**来解析，所以这两个 .py 的可执行位必须留着（上游是 100755）。
ls -l "$WS/src/tesse_ros_bridge/src/tesse_ros_bridge/"*.py | indent

# `tesse` python 客户端库。它的 setup.py 里 install_requires =
# ['numpy >= 1.16.0', 'defusedxml >= 0.6.0', 'Pillow >= 5.0.0', 'enum34 >= 1.1.10']，
# 而 **enum34 只有 py2 能装**（py3 上 pip 直接报错）。所以：先手动备齐前三个，
# 再用 --no-deps 装本体。
say "装 tesse-interface（跳过 py2-only 的 enum34）"
pip3 install -q numpy defusedxml Pillow || warn "numpy/defusedxml/Pillow 有告警"
pip3 install -q -e "$BRIDGE_SRC/tesse-interface" --no-deps \
  || die "pip install tesse-interface 失败"
python3 -c "import tesse" 2>/dev/null \
  && note "tesse 模块导入 OK" \
  || warn "tesse 模块导入失败 —— tesse_ros_node 会起不来"

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
ros_source "$WS/devel/setup.bash"
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

# ---------------------------------------------------------------------------
# 就绪判据：读 Unity 自己的日志标记（**不要**改回 TCP 探端口，也不是睡 N 秒）
# ---------------------------------------------------------------------------
# TESSE 的 C# 各 interface 一进 Start() 就把自己的 listener 线程拉起来并打印
#   "Image request listener started" / "Metadata request listener started..."
# 这就是「服务端已可被访问」的权威信号。而端口本身探不得：
#   metadata(9001) / position(9000) 都是 UDP，没有 listen 状态，
#   /dev/tcp 连它必然失败（Run 3 就是这么被误判掉一整趟的）。
readiness_marker() {
  grep -aqE "Metadata request listener started|Image request listener started" \
       "$OUT/unity.log" 2>/dev/null
}

unity_ready() {
  # ⚠️ 千万别把这两行合写成 `local wait_s="$1" end=$((SECONDS + wait_s))` ——
  # `local` 命令的右侧展开在声明**生效之前**完成，`$wait_s` 会去外层作用域找，
  # 在 `set -u` 下直接 `wait_s: unbound variable` 崩掉。
  # 这不是理论：Run 4（38126371856）就是这么在 248 s 处一步崩掉的。
  local wait_s="$1"
  local end=$((SECONDS + wait_s))
  while [ "$SECONDS" -lt "$end" ]; do
    pgrep -f "$UNITY_NAME" >/dev/null 2>&1 || return 2   # 进程没了就别等
    readiness_marker && return 0
    sleep 2
  done
  return 1
}

say "等 Unity 的 TESSE 监听器就绪（读 unity.log 标记，最长 ${UNITY_BOOT_WAIT}s）"
unity_ready "$UNITY_BOOT_WAIT"
UP_RC=$?

# 无论成败都落一份监听表：下次再遇到「端口/协议」类问题可以直接看，不用猜。
{
  echo "--- ss -lntup（TCP+UDP 全部监听/绑定）"
  ss -lntup 2>/dev/null || netstat -lntup 2>/dev/null || echo "(既没有 ss 也没有 netstat)"
} > "$OUT/sockets.txt" 2>&1 || true
cat "$OUT/sockets.txt" | indent || true

if [ "$UP_RC" = 2 ]; then
  warn "Unity 进程已退出。unity.log 尾部："
  tail -40 "$OUT/unity.log" 2>/dev/null | indent || true
  warn "unity_stdout.log 尾部："
  tail -20 "$OUT/unity_stdout.log" 2>/dev/null | indent || true
  die "Unity 启动失败"
fi
if [ "$UP_RC" != 0 ]; then
  warn "等满 ${UNITY_BOOT_WAIT}s 仍未见「监听器就绪」标记。unity.log 尾部："
  tail -40 "$OUT/unity.log" 2>/dev/null | indent || true
  warn "unity_stdout.log 尾部："
  tail -20 "$OUT/unity_stdout.log" 2>/dev/null | indent || true
  die "Unity 没起来（可调大 UNITY_BOOT_WAIT 重试）"
fi
note "Unity 已就绪（TESSE 监听器已起）—— 帧率取证行（Unity 事件里的 Display / OPENGL LOG）："
grep -aoE "Display [0-9]+ '[^']*': [0-9]+x[0-9]+|Creating OpenGL [0-9.]+ graphics device" \
     "$OUT/unity.log" 2>/dev/null | head -4 | indent || true
note "再等 ${UNITY_SETTLE}s 让场景跑起来，然后上桥接"
sleep "$UNITY_SETTLE"

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

say "roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch（publish_imu=$PUBLISH_IMU）"
# use_broadcast 保持官方默认 true。
# tesse_ros_node.py 里写的是 `if self.use_broadcast: udp_host = '<broadcast>'` ——
# 改成 false 会把 UDP 监听绑到 self_ip:9004 的**单播**地址，而 Unity 侧是往广播
# 地址发 metadata 的 → 直接丢 /current_state，整条链路废掉。
#
# 其余参数一律用 launch 默认，只动 publish_imu（理由见文件头参数区）。
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch \
    publish_imu:="$PUBLISH_IMU" \
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

  # 先把窗口清单落盘。Unity Linux 独立构建的窗口标题是 ProductName，
  # **未必**等于 $UNITY_WIN_NAME；万一名字对不上，日志里至少能看到真实标题。
  # （只做诊断，不改变行为 —— 每次只动一个变量。）
  {
    echo "--- xdotool search --onlyvisible --name '.*'  →  id / 标题 / 几何"
    for w in $(xdotool search --onlyvisible --name '.*' 2>/dev/null); do
      printf '%s  |  %s  |  %s\n' "$w" \
        "$(xdotool getwindowname "$w" 2>/dev/null)" \
        "$(xdotool getwindowgeometry --shell "$w" 2>/dev/null | tr '\n' ' ')"
    done
  } > "$OUT/windows.txt" 2>&1 || true
  cat "$OUT/windows.txt" | indent || true

  WID="$(xdotool search --name "$UNITY_WIN_NAME" 2>/dev/null | head -1)"
  if [ -n "$WID" ]; then
    xdotool windowfocus "$WID" 2>/dev/null || true
    note "  窗口 id = $WID（匹配名 '$UNITY_WIN_NAME'）"
  else
    warn "  没有名字含 '$UNITY_WIN_NAME' 的窗口 —— 见上面的窗口清单"
    warn "  退化为对当前焦点窗口发键（Xvfb 无 WM，focus=PointerRoot）"
  fi
  # 只按一次：W 很可能是 arm 的 toggle，连按两次会把它又关掉。
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
