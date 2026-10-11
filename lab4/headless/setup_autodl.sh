#!/usr/bin/env bash
# =============================================================================
# setup_autodl.sh — 在 AutoDL（Ubuntu 20.04）上装好 LAB4 无头复现所需的 ROS Noetic 工作区
#
# 用法：
#   bash setup_autodl.sh                    # 全自动，工作区默认 ~/vnav_ws
#   WS=~/my_ws bash setup_autodl.sh         # 自定义工作区
#   JOBS=2 bash setup_autodl.sh             # 限制并行编译数（内存小时用）
#
# 前置：把整个 `机器人实验/` 目录上传到 AutoDL（含 lab3/ 与 lab4/），
#       因为要复用 lab3 作为对照，且 lab4/headless 里的脚本靠相对路径找东西。
#
# ⚠️ 这台机器**不需要 GPU**。建议：
#    · 构建阶段：用带卡实例（内存/核数更足，编译快）
#    · 构建完成后：关机 → 无卡模式开机（¥0.1/小时）再跑 run_race_headless.sh
# =============================================================================
set -o pipefail

HEADLESS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB4_DIR="$(cd "$HEADLESS_DIR/.." && pwd)"
REPO_DIR="$(cd "$LAB4_DIR/.." && pwd)"          # 机器人实验/
WS="${WS:-$HOME/vnav_ws}"
JOBS="${JOBS:-2}"

say()  { printf '\033[1;36m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[setup]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[setup] %s\033[0m\n' "$*" >&2; exit 1; }

# -----------------------------------------------------------------------------
# 0. 环境自检
# -----------------------------------------------------------------------------
say "LAB4 目录: $LAB4_DIR"
[ -d "$LAB4_DIR/planner_pkg" ] || die "找不到 $LAB4_DIR/planner_pkg —— 目录不对？"
[ -d "$LAB4_DIR/dependencies" ] || die "找不到 $LAB4_DIR/dependencies"

OS_VER="$(lsb_release -rs 2>/dev/null || echo unknown)"
say "系统版本: $OS_VER"
if [ "$OS_VER" != "20.04" ]; then
  warn "ROS Noetic 官方只支持 Ubuntu 20.04，当前是 $OS_VER —— 后面可能失败。"
  warn "可以试试：在 AutoDL 自定义镜像里选 Ubuntu 20.04 基础镜像重建实例。"
fi

# -----------------------------------------------------------------------------
# 1. AutoDL 学术加速（让它能连上 github/gitlab —— 依赖包全靠下载源码构建）
# -----------------------------------------------------------------------------
if [ -f /etc/network_turbo ]; then
  say "启用 AutoDL 学术加速（/etc/network_turbo）"
  # shellcheck disable=SC1091
  source /etc/network_turbo || warn "学术加速加载失败，继续（可能下载很慢）"
else
  warn "未找到 /etc/network_turbo（非 AutoDL 环境？）。"
  warn "依赖包需要访问 github.com / gitlab.com，请自行确保网络可达。"
fi

# -----------------------------------------------------------------------------
# 2. 安装 ROS Noetic
# -----------------------------------------------------------------------------
if [ -d /opt/ros/noetic ]; then
  say "已存在 /opt/ros/noetic，跳过安装"
else
  say "安装 ROS Noetic（ros-base，不带 GUI，够用）"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list'
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | sudo apt-key add - || warn "ros.asc 下载失败（网络？）"
  sudo apt-get update -qq || die "apt-get update 失败"
  sudo apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    ros-noetic-catkin \
    python3-rosdep python3-rosinstall-generator python3-vcstool \
    python3-catkin-tools \
    || die "ROS Noetic 安装失败"
fi

# -----------------------------------------------------------------------------
# 3. 构建依赖（关键的坑都在这一节）
#
#    依赖包**不是**用系统库，而是自己下载源码构建：
#      · eigen_catkin     : 先 find_package(Eigen3)，找到了就用系统的 → 装 libeigen3-dev 可省一次下载
#      · glog_catkin      : 无条件下载 glog 0.3.5 并用 autotools 构建 → 需 autoconf/automake/libtool/patch
#      · nlopt            : 用 wget 下载 nlopt-2.4.2.tar.gz（ethz-asl/thirdparty_library_binaries）
#      · yaml_cpp_catkin  : git clone jbeder/yaml-cpp 指定 commit → 需 git + patch
#    因此 autotools / wget / git / patch / pkg-config 一个都不能少。
# -----------------------------------------------------------------------------
say "安装构建依赖"
sudo apt-get install -y --no-install-recommends \
  build-essential cmake pkg-config \
  git wget curl patch unzip \
  autoconf automake libtool \
  libeigen3-dev \
  libboost-all-dev \
  libyaml-cpp-dev \
  python3-numpy python3-pip python3-yaml \
  || warn "部分包安装失败，继续（见后续报错）"

# pip 用清华源（AutoDL 上更稳）
pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple >/dev/null 2>&1 || true

# -----------------------------------------------------------------------------
# 4. rosdep
# -----------------------------------------------------------------------------
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  say "初始化 rosdep"
  sudo rosdep init || warn "rosdep init 失败（可能已初始化）"
fi
rosdep update || warn "rosdep update 失败（网络问题可忽略，后面用 -r 跳过未知依赖）"

# -----------------------------------------------------------------------------
# 5. 建工作区、铺包
# -----------------------------------------------------------------------------
say "创建工作区 $WS/src"
mkdir -p "$WS/src"

# 5.1 lab4 的两个包 + 从 lab3 复用控制器
for p in planner_pkg trajectory_generation_pkg; do
  rm -rf "$WS/src/$p"
  cp -r "$LAB4_DIR/$p" "$WS/src/$p"
done
if [ -d "$LAB4_DIR/controller_pkg" ]; then
  rm -rf "$WS/src/controller_pkg"
  cp -r "$LAB4_DIR/controller_pkg" "$WS/src/controller_pkg"
else
  die "找不到 lab4/controller_pkg（应从 lab3 复制过来）"
fi

# 5.2 依赖包：
#     · 单包直接拷
#     · mav_comm / mav_trajectory_generation 是**容器目录**，要把里面的包摊平到 src/
#     · mav_trajectory_generation 的顶层 CMakeLists.txt 是 WARP 工作区专用的（引用了 ~/warp_ws），
#       必须删掉，否则 catkin 会把它误当包处理
for p in catkin_simple eigen_catkin eigen_checks glog_catkin nlopt yaml_cpp_catkin; do
  rm -rf "$WS/src/$p"
  cp -r "$LAB4_DIR/dependencies/$p" "$WS/src/$p"
done

for sub in "$LAB4_DIR"/dependencies/mav_comm/*/; do
  [ -f "$sub/package.xml" ] || continue
  n="$(basename "$sub")"
  rm -rf "$WS/src/$n"; cp -r "$sub" "$WS/src/$n"
done

for sub in "$LAB4_DIR"/dependencies/mav_trajectory_generation/*/; do
  [ -f "$sub/package.xml" ] || continue
  case "$(basename "$sub")" in
    mav_trajectory_generation_example) continue ;;   # 示例包，会拖入额外依赖，跳过
  esac
  n="$(basename "$sub")"
  rm -rf "$WS/src/$n"; cp -r "$sub" "$WS/src/$n"
done

say "src/ 下的包："
ls -1 "$WS/src"

# -----------------------------------------------------------------------------
# 6. 编译
# -----------------------------------------------------------------------------
say "source ROS 环境"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

cd "$WS" || die "cd $WS 失败"
say "rosdep install"
rosdep install --from-paths src --ignore-src -y -r || warn "rosdep install 有非致命失败"

say "catkin config"
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release || \
  die "catkin config 失败（catkin_tools 没装好？）"

say "catkin build -j$JOBS（这一步最慢，重点看 glog/nlopt 的下载是否成功）"
if ! catkin build -j"$JOBS" --no-status; then
  warn "catkin build 失败。按以下顺序排查："
  warn "  1) 2GB 内存不够 → JOBS=1 bash setup_autodl.sh  或换带卡实例"
  warn "  2) glog 下载失败 → 先 source /etc/network_turbo 再来一次"
  warn "  3) nlopt 下载失败 → 手动下载 nlopt-2.4.2.tar.gz 放进 \$WS/build/nlopt_src/"
  warn "  4) 看具体报错：catkin build -j1 --verbose"
  exit 1
fi

# -----------------------------------------------------------------------------
# 7. 收尾自检
# -----------------------------------------------------------------------------
say "编译完成。自检："
bash -lc "source $WS/devel/setup.bash && rospack find trajectory_generation && rospack find planner_pkg && rospack find controller_pkg && rospack find mav_trajectory_generation"

cat <<EOF

$(printf '\033[1;32m')============================================================$(printf '\033[0m')
 安装完成。

 下一步（每次开新终端都要 source）：

   source /opt/ros/noetic/setup.bash
   source $WS/devel/setup.bash

 然后跑竞速并录 rosbag：

   WS=$WS bash $HEADLESS_DIR/run_race_headless.sh

 产出（默认在 ~/race_out/）：
   race.bag            官方要求的 rosbag（/current_state + /desired_state）
   metrics.json        自动分析的指标
   race_tracking.png   实际 vs 期望轨迹
   *.log               各节点日志（出问题先看它）

 跑完记得关机（无卡模式 ¥0.1/h，也别白花）。
$(printf '\033[1;32m')============================================================$(printf '\033[0m')
EOF
