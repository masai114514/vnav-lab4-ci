#!/usr/bin/env bash
# =============================================================================
# setup_robostack.sh — 在 **Ubuntu 22.04 (jammy)** 上用 RoboStack 装 ROS Noetic
#
# 为什么需要这个脚本
# ------------------
# ROS Noetic 官方 deb 只发布到 Ubuntu **20.04 (focal)**。AutoDL 现在的默认镜像
# 是 22.04，`packages.ros.org/ros/ubuntu focal` 在 jammy 上装不了（硬塞 focal 源
# 还会污染系统库）。RoboStack 把 ROS 重新打包成 conda 包，**与发行版解耦**，
# 所以是 jammy 上唯一省事的路子。
#
# 与 setup_autodl.sh 的分工
#   · setup_autodl.sh   → Ubuntu 20.04 机器（官方 deb，ROS 在 /opt/ros/noetic）
#   · setup_robostack.sh→ Ubuntu 22.04 机器（conda，ROS 在 $CONDA_PREFIX）
#   两者建出来的工作区布局相同，run_race_headless.sh 都能跑（靠 ROS_SETUP 变量区分）。
#
# 用法：
#   bash setup_robostack.sh
#   ENV_NAME=ros2 JOBS=16 bash setup_robostack.sh
# =============================================================================
set -o pipefail

HEADLESS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB4_DIR="$(cd "$HEADLESS_DIR/.." && pwd)"
WS="${WS:-$HOME/vnav_ws}"
ENV_NAME="${ENV_NAME:-ros}"
JOBS="${JOBS:-8}"
CONDA_ROOT="${CONDA_ROOT:-$HOME/miniconda3}"

say()  { printf '\033[1;36m[setup-rb]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[setup-rb]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[setup-rb] %s\033[0m\n' "$*" >&2; exit 1; }

[ -f "$CONDA_ROOT/etc/profile.d/conda.sh" ] || die "找不到 $CONDA_ROOT/etc/profile.d/conda.sh（没有 conda？）"
[ -d "$LAB4_DIR/dependencies" ] || die "找不到 $LAB4_DIR/dependencies"

# -----------------------------------------------------------------------------
# 1. 系统构建工具（依赖包全是「下载源码 + 构建」）
#    · glog_catkin     : autoreconf -fi && ./configure  → 需 autoconf/automake/libtool
#    · nlopt           : wget 下一个 tar.gz 再 configure
#    · yaml_cpp_catkin : git clone + patch
#    · eigen_catkin    : 用系统 Eigen（≥3.3.4；jammy 的 libeigen3-dev 是 3.4.0）
# -----------------------------------------------------------------------------
say "安装系统构建工具"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq || warn "apt-get update 失败"
apt-get install -y -qq \
  autoconf automake libtool pkg-config patch wget git unzip \
  libeigen3-dev libyaml-cpp-dev \
  || warn "部分 apt 包安装失败，继续"

# -----------------------------------------------------------------------------
# 2. 建 conda 环境（RoboStack 的 ROS Noetic）
#    注意：这里**故意不装 conda 的 compilers**，用系统 g++ —— 目标代码是 2015 年
#    前后的老代码（glog 0.3.5 / nlopt 2.4.2），gcc 11 比 conda 的新版 gcc 好过。
# -----------------------------------------------------------------------------
# shellcheck disable=SC1091
source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda config --set channel_priority flexible

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  say "conda 环境 $ENV_NAME 已存在，跳过创建"
else
  say "创建 conda 环境 $ENV_NAME（RoboStack；下载约 1 GB，耐心）"
  conda create -n "$ENV_NAME" -c robostack-staging -c conda-forge \
    ros-noetic-ros-base \
    ros-noetic-tf ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs \
    ros-noetic-eigen-conversions ros-noetic-cmake-modules ros-noetic-roslib \
    ros-noetic-xacro ros-noetic-roslaunch ros-noetic-rosbag \
    catkin_tools \
    eigen numpy matplotlib scipy pip \
    || die "conda create 失败"
fi

conda activate "$ENV_NAME"
export ROS_SETUP="$CONDA_PREFIX/setup.bash"
# shellcheck disable=SC1091
source "$ROS_SETUP"

say "ROS_DISTRO=$ROS_DISTRO"
say "ROS_SETUP=$ROS_SETUP"

# catkin_tools 若 conda 里没有，退回 pip
if ! command -v catkin >/dev/null 2>&1; then
  warn "conda 里没有 catkin_tools，改用 pip 装"
  python -m pip install -q catkin_tools || die "pip install catkin_tools 失败"
fi

# -----------------------------------------------------------------------------
# 3. 建工作区、铺包（与 setup_autodl.sh 同构）
# -----------------------------------------------------------------------------
say "建工作区 $WS"
rm -rf "$WS"; mkdir -p "$WS/src"
cd "$LAB4_DIR" || die "cd $LAB4_DIR 失败"

for p in planner_pkg trajectory_generation_pkg controller_pkg; do
  cp -r "$p" "$WS/src/"
done
for p in catkin_simple eigen_catkin eigen_checks glog_catkin nlopt yaml_cpp_catkin; do
  cp -r "dependencies/$p" "$WS/src/"
done
# mav_comm / mav_trajectory_generation 是**容器目录**：只把里面的 catkin 包摊平出来
# （顶层那个 CMakeLists.txt 是 ethz-asl WARP 工作区专用的，引用了 ~/warp_ws，不能带进来）
for sub in dependencies/mav_comm/*/; do
  [ -f "$sub/package.xml" ] && cp -r "$sub" "$WS/src/"
done
for sub in dependencies/mav_trajectory_generation/*/; do
  [ -f "$sub/package.xml" ] || continue
  case "$(basename "$sub")" in mav_trajectory_generation_example) continue;; esac
  cp -r "$sub" "$WS/src/"
done

# 保险：yaml_cpp_catkin 的 CMakeLists 用的是 ${CMAKE_SOURCE_DIR}/extra_version.patch，
# 在 catkin_tools 下等于包目录（正确）；但如果谁改用 catkin_make，它会被解析成
# ws/src 于是找不到 patch。这里多放一份，两种构建器都能过。
cp -n "$WS/src/yaml_cpp_catkin/extra_version.patch" "$WS/src/" 2>/dev/null || true

say "src/ 下的包："
ls -1 "$WS/src"

# -----------------------------------------------------------------------------
# 4. 编译
# -----------------------------------------------------------------------------
cd "$WS" || die "cd $WS 失败"
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release || die "catkin config 失败"

say "catkin build -j$JOBS"
if ! catkin build -j"$JOBS" --no-status; then
  warn "catkin build 失败。常见原因与对策："
  warn "  1) glog 0.3.5 的 autoreconf 失败 → apt-get install -y autoconf automake libtool"
  warn "  2) nlopt 下载失败         → source /etc/network_turbo 后重试"
  warn "  3) 缺 yaml-cpp            → apt-get install -y libyaml-cpp-dev"
  warn "  4) 看单个包：catkin build <pkg> -j1 --verbose"
  exit 1
fi

# -----------------------------------------------------------------------------
# 5. 自检
# -----------------------------------------------------------------------------
say "自检："
bash -lc "source '$ROS_SETUP' && source '$WS/devel/setup.bash' && \
  rospack find trajectory_generation && rospack find planner_pkg && \
  rospack find controller_pkg && rospack find mav_trajectory_generation"

cat <<EOF

$(printf '\033[1;32m')============================================================$(printf '\033[0m')
 安装完成（RoboStack 版）。跑竞速：

   export ROS_SETUP=$ROS_SETUP
   export PYTHON=$CONDA_PREFIX/bin/python3
   WS=$WS bash $HEADLESS_DIR/run_race_headless.sh

 产出在 ~/race_out/。跑完关机：/usr/bin/shutdown
$(printf '\033[1;32m')============================================================$(printf '\033[0m')
EOF
