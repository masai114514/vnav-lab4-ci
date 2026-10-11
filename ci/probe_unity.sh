#!/usr/bin/env bash
# =============================================================================
# probe_unity.sh —— 快探针：只验「Unity 能不能起来 + 桥接能不能连上」
#
# 不进竞速、不录 bag、不跑控制器。用在改渲染参数 / 改等待时间时的快速迭代，
# 避免每次都烧一整趟 official_race.sh。
#
# 用法（容器内，root）：
#   OUT=$PWD/out BUNDLE=$PWD/lab4 bash ci/probe_unity.sh
#
# 它做的事就是 official_race.sh 的 §1~§5，外加：
#   · 抓到 Unity 的帧时浮层（左上角）放大后存成 png，用于读真实帧率
#   · 连发 3 次 /current_state 的到达间隔，粗看链路健康度
# =============================================================================
set -uo pipefail

OUT="${OUT:-$PWD/out}"
BUNDLE="${BUNDLE:-$PWD/lab4}"
DISP="${DISP:-:99}"
RES="${RES:-640x400}"
XVFB_RES="${XVFB_RES:-$RES}"
UNITY_BOOT_WAIT="${UNITY_BOOT_WAIT:-240}"
# 监听器起了 ≠ 场景已经在跑，给一小段起步时间（与 official_race.sh 同口径）
UNITY_SETTLE="${UNITY_SETTLE:-8}"
BRIDGE_WAIT="${BRIDGE_WAIT:-120}"
JOBS="${JOBS:-$(nproc)}"
SKIP_APT="${SKIP_APT:-0}"
PLACEHOLDER_ONLY="${PLACEHOLDER_ONLY:-0}"
LAB4_ZIP_URL="${LAB4_ZIP_URL:-https://vnav.mit.edu/material/lab4.zip}"

say()  { printf '\033[1;36m[probe]\033[0m %s\n' "$*"; }
note() { printf '\033[1;35m[probe]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[probe]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[probe] %s\033[0m\n' "$*" >&2; exit 1; }
indent() { sed 's/^/    /'; }

# 见 official_race.sh 里的同名函数：ROS 的 setup.bash 会在 profile.d 里引用未定义
# 变量（10.roslaunch.sh: ROS_MASTER_URI: unbound variable），在 set -u 下直接中止。
ros_source() {
  set +u
  # shellcheck disable=SC1090,SC1091
  . "$1"
  set -u
}

mkdir -p "$OUT"
PIDS=()
killall_bg() {
  local p
  for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null || true; done
  pkill -f "lab4.x86_64" 2>/dev/null || true
  pkill -f "Xvfb $DISP"  2>/dev/null || true
  return 0
}
trap 'killall_bg' EXIT

# ---- 依赖（与 official_race.sh 同一套，改一处要改两处） --------------------
if [ "$SKIP_APT" != "1" ]; then
  export DEBIAN_FRONTEND=noninteractive
  say "apt 装图形栈与 ROS 组件"
  apt-get update -qq || warn "apt-get update 有告警"
  apt-get install -y -qq --no-install-recommends \
    xvfb x11-utils xdotool imagemagick ffmpeg curl unzip ca-certificates git \
    libgl1-mesa-dri libglx-mesa0 libglu1-mesa mesa-utils \
    libxcursor1 libxrandr2 libxinerama1 libxi6 libxss1 libasound2 \
    libnss3 libgbm1 libgconf-2-4 \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-eigen-conversions \
    ros-noetic-cv-bridge ros-noetic-roslaunch ros-noetic-rosbag ros-noetic-tf2-ros \
    ros-noetic-ackermann_msgs ros-noetic-message-generation ros-noetic-message-runtime \
    python3-pip python3-scipy python3-opencv python3-empy python3-defusedxml \
    python-is-python3 \
    || warn "部分包装不上，继续"
  # 桥接脚本 shebang 是 `#!/usr/bin/env python`，而 focal 默认只有 python3 →
  # roslaunch 起节点时 `/usr/bin/env: 'python': No such file or directory`，exit 127。
  # 见 official_race.sh 里同名段落（Run 5 就死在这）。
  if ! command -v python >/dev/null 2>&1; then
    warn "没有 python —— 手动链接到 python3"
    ln -sf "$(command -v python3)" /usr/local/bin/python || die "建 python 链接失败"
  fi
  command -v python >/dev/null 2>&1 || die "没有 python 解释器，桥接必然起不来"
  command -v catkin >/dev/null 2>&1 || pip3 install -q catkin_tools || die "catkin_tools 装不上"
  # $HOME 在容器 job 里是 /github/home，不是 /root
  printf 'pcm.!default { type null }\nctl.!default { type null }\n' > "$HOME/.asoundrc"
fi

ros_source /opt/ros/noetic/setup.bash

# 桥接：有现成工作区就用（official_race.sh 建的），没有就现场拉一份最小工作区。
# 桥接来源见 official_race.sh 的说明：必须用 MIT-SPARK/VNAV-labs 的 tag VNAV2021，
# MIT-TESSE/tesse-ros-bridge 的 master 已经是 ROS2 分支，不兼容。
VNAV_TAG="${VNAV_TAG:-VNAV2021}"
VNAV_DIR="${VNAV_DIR:-$HOME/vnav-labs}"
BRIDGE_SRC="$VNAV_DIR/lab3/tesse-ros-bridge"
WS="${WS:-$HOME/vnav_ws}"

if [ -f "$WS/devel/setup.bash" ]; then
  note "复用已有工作区 $WS"
  ros_source "$WS/devel/setup.bash"
else
  say "没有现成工作区，只拉桥接做最小构建"
  [ -d "$VNAV_DIR/.git" ] || \
    git clone -q --depth 1 --branch "$VNAV_TAG" \
      "${VNAV_URL:-https://github.com/MIT-SPARK/VNAV-labs.git}" "$VNAV_DIR" \
    || die "clone VNAV-labs @ $VNAV_TAG 失败"
  mkdir -p "$WS/src"
  [ -d "$WS/src/tesse_ros_bridge" ] || cp -r "$BRIDGE_SRC/ROS" "$WS/src/tesse_ros_bridge"
  find "$WS/src/tesse_ros_bridge" -name '*.pyc' -delete 2>/dev/null || true
  # setup.py 声明了 packages=['tesse_ros_bridge','key_teleop']，而 key_teleop/ 没有 __init__.py
  [ -f "$WS/src/tesse_ros_bridge/src/key_teleop/__init__.py" ] || \
    : > "$WS/src/tesse_ros_bridge/src/key_teleop/__init__.py"
  [ -d "$WS/src/catkin_simple" ] || cp -r "$BUNDLE/dependencies/catkin_simple" "$WS/src/" 2>/dev/null || true
  [ -d "$WS/src/mav_comm" ] || { for sub in "$BUNDLE"/dependencies/mav_comm/*/; do
      [ -f "$sub/package.xml" ] && cp -r "$sub" "$WS/src/"; done; }
  pip3 install -q numpy defusedxml Pillow 2>/dev/null || true
  pip3 install -q -e "$BRIDGE_SRC/tesse-interface" --no-deps || warn "tesse-interface 装不上"
  ( cd "$WS" && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release >/dev/null \
      && catkin build tesse_ros_bridge -j"$JOBS" --no-status ) || die "桥接构建失败"
  ros_source "$WS/devel/setup.bash"
fi

# ---- 下 Unity ---------------------------------------------------------------
if [ -z "${UNITY_BIN:-}" ]; then
  [ -s "$OUT/lab4.zip" ] || { say "下载 lab4.zip"; curl -fL --retry 3 --max-time 600 -o "$OUT/lab4.zip" "$LAB4_ZIP_URL" || die "下载失败"; }
  rm -rf "$OUT/unity"; mkdir -p "$OUT/unity"
  unzip -q -o "$OUT/lab4.zip" -d "$OUT/unity" || die "解包失败"
  UNITY_BIN="$(find "$OUT/unity" -maxdepth 3 -type f -name '*.x86_64' | head -1)"
fi
[ -n "$UNITY_BIN" ] || die "找不到 *.x86_64"
chmod +x "$UNITY_BIN" 2>/dev/null || true
UNITY_DIR="$(dirname "$UNITY_BIN")"; UNITY_NAME="$(basename "$UNITY_BIN")"

# ---- Xvfb + Unity ----------------------------------------------------------
export DISPLAY="$DISP" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS="$JOBS"
say "启动 Xvfb $DISP （${XVFB_RES}x24）"
Xvfb "$DISP" -screen 0 "${XVFB_RES}x24" -ac +extension GLX +render -noreset > "$OUT/xvfb.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 30); do xdpyinfo -display "$DISP" >/dev/null 2>&1 && break; sleep 1; done
xdpyinfo -display "$DISP" >/dev/null 2>&1 || die "Xvfb 起不来"

NICE=()
if nice -n -5 true 2>/dev/null; then NICE=(nice -n -5); else warn "无 CAP_SYS_NICE，不加优先级"; fi

say "启动 Unity"
( cd "$UNITY_DIR" && exec "${NICE[@]}" "./$UNITY_NAME" \
    -logFile "$OUT/unity.log" -screen-fullscreen 0 \
    -screen-width "${RES%x*}" -screen-height "${RES#*x}" -force-glcore \
    < /dev/null > "$OUT/unity_stdout.log" 2>&1 ) &
PIDS+=("$!")

# 就绪判据与 official_race.sh 完全一致：读 Unity 日志里的监听器标记。
# **不要**改回 /dev/tcp 探端口 —— metadata(9001)/position(9000) 都是 UDP，
# 没有 listen 状态，TCP 探它必然失败（Run 3 一整趟就是这么被误判掉的）。
readiness_marker() {
  grep -aqE "Metadata request listener started|Image request listener started" \
       "$OUT/unity.log" 2>/dev/null
}

unity_ready() {
  # 同 official_race.sh：两个 local 必须分开写（`local a=.. b=$a` 的 $a 是外层作用域）
  local wait_s="$1"
  local end=$((SECONDS + wait_s))
  while [ "$SECONDS" -lt "$end" ]; do
    pgrep -f "$UNITY_NAME" >/dev/null 2>&1 || return 2
    readiness_marker && return 0
    sleep 2
  done
  return 1
}

say "等 Unity 的 TESSE 监听器就绪（读 unity.log 标记，最长 ${UNITY_BOOT_WAIT}s）"
unity_ready "$UNITY_BOOT_WAIT"; UP_RC=$?
{
  echo "--- ss -lntup（TCP+UDP 全部监听/绑定）"
  ss -lntup 2>/dev/null || netstat -lntup 2>/dev/null || echo "(既没有 ss 也没有 netstat)"
} > "$OUT/sockets.txt" 2>&1 || true
cat "$OUT/sockets.txt" | indent || true
if [ "$UP_RC" = 2 ]; then
  tail -40 "$OUT/unity.log" 2>/dev/null | indent || true
  die "Unity 进程死了"
elif [ "$UP_RC" != 0 ]; then
  tail -40 "$OUT/unity.log" 2>/dev/null | indent || true
  die "Unity 在 ${UNITY_BOOT_WAIT}s 内没打出监听器就绪标记"
fi
note "Unity 就绪（TESSE 监听器已起）："
grep -aoE "Display [0-9]+ '[^']*': [0-9]+x[0-9]+|Creating OpenGL [0-9.]+ graphics device" \
     "$OUT/unity.log" 2>/dev/null | head -4 | indent || true
note "再等 ${UNITY_SETTLE}s 让场景跑起来"
sleep "$UNITY_SETTLE"

# 帧时浮层连拍 3 张（间隔 8 s），用于读真实帧率
say "抓帧时浮层（3 张，间隔 8 s）"
for i in 1 2 3; do
  sleep 8
  if import -window root "$OUT/probe_shot_$i.png" 2>/dev/null; then
    convert "$OUT/probe_shot_$i.png" -crop 320x44+0+0 +repage -filter point -resize 300% \
            "$OUT/probe_fps_$i.png" 2>/dev/null || true
    note "  probe_fps_$i.png"
  fi
done

if [ "$PLACEHOLDER_ONLY" = "1" ]; then
  say "PLACEHOLDER_ONLY=1 —— 到此为止"
  exit 0
fi

# ---- 桥接 ------------------------------------------------------------------
say "启动 roscore + 桥接"
roscore -p 11311 > "$OUT/roscore.log" 2>&1 &
PIDS+=("$!")
for _ in $(seq 1 40); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
rostopic list >/dev/null 2>&1 || die "roscore 起不来"
rosparam set /use_sim_time true

roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch > "$OUT/bridge.log" 2>&1 &
PIDS+=("$!")

say "等 /current_state（最长 ${BRIDGE_WAIT}s）"
ok=0
for _ in $(seq 1 $((BRIDGE_WAIT / 2))); do
  if timeout 3 rostopic echo -n1 /current_state >/dev/null 2>&1; then ok=1; break; fi
  sleep 2
done
if [ "$ok" != 1 ]; then
  warn "rosnode list:"; rosnode list 2>&1 | indent || true
  warn "话题:";        rostopic list 2>&1 | grep -E "current|clock|rotor" | indent || true
  warn "bridge.log 尾部:"; tail -30 "$OUT/bridge.log" 2>/dev/null | indent || true
  die "桥接没连上"
fi
note "/current_state 通了"

say "/current_state 到达率（8 s 采样）"
timeout 9 rostopic hz /current_state 2>&1 | tail -3 | indent || true
say "/clock 到达率（8 s 采样）"
timeout 9 rostopic hz /clock 2>&1 | tail -3 | indent || true

say "探针完成。日志与截图在 $OUT"
ls -la "$OUT" | indent
