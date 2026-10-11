#!/usr/bin/env bash
# =============================================================================
# docker_in_container.sh — 在 `ros:noetic-ros-base`（linux/amd64）容器里跑完
#                          LAB4 无头复现：装依赖 → 建 catkin 工作区 → 竞速 → 出 rosbag
#
# 为什么用容器而不是直接装
# ------------------------
# · ROS Noetic 官方 deb 只有 Ubuntu 20.04；本机是 macOS，AutoDL 无卡机是 22.04 且只有
#   2 GiB 内存（conda 解算就被 OOM 杀）。容器里就是**原版 focal + Noetic**，
#   保真度最高，也不受宿主机发行版限制。
# · Apple Silicon 上靠 colima(vz + Rosetta binfmt) 跑 amd64 镜像，接近原生速度。
#
# 用法（容器内）：
#   bash /work/lab4/headless/docker_in_container.sh
#   可用环境变量覆盖：WS / OUT / JOBS / BUNDLE
# =============================================================================
set -o pipefail

BUNDLE="${BUNDLE:-/work/lab4}"
WS="${WS:-$HOME/vnav_ws}"
OUT="${OUT:-/work/lab4/race_out}"
JOBS="${JOBS:-4}"

say()  { printf '\033[1;36m[docker]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[docker]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[docker] %s\033[0m\n' "$*" >&2; exit 1; }

[ -d "$BUNDLE/dependencies" ] || die "找不到 $BUNDLE/dependencies —— 目录挂载对了吗？"

# -----------------------------------------------------------------------------
# 1. 系统依赖
# -----------------------------------------------------------------------------
say "apt 装构建工具与系统库"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq || warn "apt-get update 有告警"
apt-get install -y -qq --no-install-recommends \
  build-essential cmake pkg-config git wget curl patch unzip \
  autoconf automake libtool \
  libeigen3-dev libyaml-cpp-dev \
  python3-numpy python3-matplotlib python3-pip python3-yaml \
  || warn "部分 apt 包失败，继续"

# ros-base 变体里没有 tf / tf2 / eigen_conversions，单独补
say "补装 ROS 组件（tf / tf2 / eigen_conversions / cmake_modules / roslib）"
apt-get install -y -qq --no-install-recommends \
  ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs \
  ros-noetic-eigen-conversions ros-noetic-cmake-modules ros-noetic-roslib \
  ros-noetic-roslaunch ros-noetic-rosbag ros-noetic-roslint \
  || warn "补装 ROS 组件有告警，继续（缺什么后面 catkin 会报）"

if ! command -v catkin >/dev/null 2>&1; then
  say "pip 装 catkin_tools"
  pip3 install -q catkin_tools || die "catkin_tools 装不上"
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
say "ROS_DISTRO=$ROS_DISTRO"

# -----------------------------------------------------------------------------
# 2. 建工作区（与 setup_autodl.sh 同构）
# -----------------------------------------------------------------------------
say "构建工作区 $WS"
rm -rf "$WS"; mkdir -p "$WS/src"
cd "$BUNDLE" || die "cd $BUNDLE 失败"

for p in planner_pkg trajectory_generation_pkg controller_pkg; do
  cp -r "$p" "$WS/src/"
done
for p in catkin_simple eigen_catkin eigen_checks glog_catkin nlopt yaml_cpp_catkin; do
  cp -r "dependencies/$p" "$WS/src/"
done
# 容器目录摊平；mav_trajectory_generation 顶层的 WARP 专用 CMakeLists 不带进来
for sub in dependencies/mav_comm/*/; do
  [ -f "$sub/package.xml" ] && cp -r "$sub" "$WS/src/"
done
for sub in dependencies/mav_trajectory_generation/*/; do
  [ -f "$sub/package.xml" ] || continue
  case "$(basename "$sub")" in mav_trajectory_generation_example) continue;; esac
  cp -r "$sub" "$WS/src/"
done
# yaml_cpp_catkin 的 PATCH_COMMAND 用了 ${CMAKE_SOURCE_DIR}/extra_version.patch
# （catkin_tools 下=包目录，正确；catkin_make 下=ws/src）。多放一份两边都过。
cp -n "$WS/src/yaml_cpp_catkin/extra_version.patch" "$WS/src/" 2>/dev/null || true

say "src/：" ; ls -1 "$WS/src"

# -----------------------------------------------------------------------------
# 2b. 依赖里的 GitHub 直连 → 镜像
#
#     实测：从容器（经 macOS 出网）连 github.com 的网页域名会被卡死
#     （v0.3.5.zip 下到 0 字节就不动了），但 CDN / 代理域名是通的。
#     三个包要联网拿源码：
#       glog_catkin        URL  https://github.com/google/glog/archive/v0.3.5.zip
#       nlopt              DOWNLOAD_COMMAND wget https://github.com/ethz-asl/.../nlopt-2.4.2.tar.gz
#       yaml_cpp_catkin    GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp
#     统一改写成 "<mirror>/https://github.com/..."（代理透明转发，文件名不变）。
# -----------------------------------------------------------------------------
GH_MIRROR="${GH_MIRROR:-}"
if [ -z "$GH_MIRROR" ]; then
  for m in https://ghfast.top https://gh-proxy.com https://ghproxy.net; do
    if curl -fsI --max-time 25 \
        "$m/https://github.com/google/glog/archive/v0.3.5.zip" >/dev/null 2>&1; then
      GH_MIRROR="$m"; break
    fi
  done
fi
[ -n "$GH_MIRROR" ] || die "找不到可用的 GitHub 镜像（试过 ghfast.top / gh-proxy.com / ghproxy.net）"
say "GitHub 镜像：$GH_MIRROR"

sed -i "s#URL https://github.com/#URL ${GH_MIRROR}/https://github.com/#" \
    "$WS/src/glog_catkin/CMakeLists.txt"
sed -i "s#wget https://github.com/#wget ${GH_MIRROR}/https://github.com/#" \
    "$WS/src/nlopt/CMakeLists.txt"
sed -i "s#GIT_REPOSITORY  https://github.com/#GIT_REPOSITORY  ${GH_MIRROR}/https://github.com/#" \
    "$WS/src/yaml_cpp_catkin/CMakeLists.txt"

say "改写结果："
grep -hnE "URL .*glog|wget .*nlopt|GIT_REPOSITORY" \
  "$WS/src/glog_catkin/CMakeLists.txt" \
  "$WS/src/nlopt/CMakeLists.txt" \
  "$WS/src/yaml_cpp_catkin/CMakeLists.txt" | sed 's/^/    /'

# 漏网的直连（注释里的不算）
leftover=$(grep -rn "github.com/" "$WS/src" --include=CMakeLists.txt \
           | grep -v "^\S*:[0-9]*: *#" | grep -v "$GH_MIRROR" || true)
[ -n "$leftover" ] && { warn "仍有未改写的 github 直连："; echo "$leftover" | sed 's/^/    /'; } || true

# -----------------------------------------------------------------------------
# 2c. nlopt 的 jobserver 坑
#
#     nlopt 的 ExternalProject 里 BUILD_COMMAND 是**裸 make**（无 -j）。它会从
#     catkin 的 `make -j$JOBS` 继承 MAKEFLAGS 里那对 jobserver FD，而 EP 是经
#     `cmake -P` 脚本调起来的、FD 已然失效 →
#         make[5]: *** write jobserver: Bad file descriptor.  Stop.
#     （对照：glog 的 BUILD_COMMAND 是 `make -j 8`，命令行显式带 -j，GNU make
#      就会忽略继承的 jobserver，所以它没事。）
#     修法：给 nlopt 的 make 显式加 -j，并清掉 MAKEFLAGS/MFLAGS；
#     `make install`（在 make_install_nlopt.sh 里）同样处理。
# -----------------------------------------------------------------------------
if [ -f "$WS/src/nlopt/CMakeLists.txt" ]; then
  sed -i "s#^  BUILD_COMMAND make\$#  BUILD_COMMAND env -u MAKEFLAGS -u MFLAGS make -j${JOBS}#" \
      "$WS/src/nlopt/CMakeLists.txt"
  sed -i "s#^DESTDIR=@NLOPT_INSTALL_PREFIX@ make install\$#DESTDIR=@NLOPT_INSTALL_PREFIX@ env -u MAKEFLAGS -u MFLAGS make -j1 install#" \
      "$WS/src/nlopt/make_install_nlopt.sh.in"
  say "nlopt jobserver 补丁："
  grep -nE "BUILD_COMMAND" "$WS/src/nlopt/CMakeLists.txt" | sed 's/^/    /'
  grep -nE "make install" "$WS/src/nlopt/make_install_nlopt.sh.in" | sed 's/^/    /'
fi

cd "$WS" || die "cd $WS 失败"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release || die "catkin config 失败"

# -----------------------------------------------------------------------------
# 3. 编译
# -----------------------------------------------------------------------------
say "catkin build -j$JOBS（重点看 glog 0.3.5 的 autoreconf 与 nlopt 的 wget）"
if ! catkin build -j"$JOBS" --no-status; then
  warn "catkin build 失败。排查："
  warn "  · glog：需 autoconf/automake/libtool/patch"
  warn "  · nlopt：需要能访问 github（wget 下 tar.gz）"
  warn "  · 单包重来：catkin build <pkg> -j1 --verbose"
  exit 1
fi

say "自检："
rospack find trajectory_generation && rospack find planner_pkg && \
  rospack find controller_pkg && rospack find mav_trajectory_generation || true

# -----------------------------------------------------------------------------
# 3b. CI 用：只构建不跑赛（SKIP_RACE=1）
# -----------------------------------------------------------------------------
if [ "${SKIP_RACE:-0}" = "1" ]; then
  say "SKIP_RACE=1 —— 只构建，不跑竞速。工作区就绪：$WS"
  exit 0
fi

# -----------------------------------------------------------------------------
# 4. 跑竞速 + 录 bag
# -----------------------------------------------------------------------------
mkdir -p "$OUT"
say "开始竞速，产出到 $OUT"
WS="$WS" OUT="$OUT" bash "$BUNDLE/headless/run_race_headless.sh" || \
  warn "竞速脚本非零退出，仍检查产物"

say "产物："
ls -la "$OUT" || true
