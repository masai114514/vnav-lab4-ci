#!/usr/bin/env bash
# =============================================================================
# sync.sh —— 把 lab4 里 CI 需要的那部分拷进本仓的 lab4/ 目录
#
# 为什么要有这一步
# ----------------
# 本仓是**独立仓**（跑在 GitHub Actions 上），而源码的正本在 `../`（也就是 lab4/）。
# 每次改完控制器/launch/headless 脚本，先 sync 再 push，两边才一致。
#
# 只同步构建与运行真正要用的 5 个目录，不整仓拷：
#   planner_pkg / trajectory_generation_pkg / controller_pkg  —— 我们的三个包
#   dependencies                                              —— catkin_simple、nlopt、
#                                                                mav_comm、mav_trajectory_generation 等
#   headless                                                  —— analyze / wait / lag_probe
#                                                                + sim_data 里的门位姿 CSV
# 刻意**不**同步的：sim/（22 MB，含渲染好的 mp4）、race_out/、report/、tools/、individual/
#   —— 这些是交付物与文档，CI 用不到，放进来只会让 API 推送变慢。
#
# 用法：
#   bash sync.sh            # 默认从 ../ 同步
#   SRC=<lab4 路径> bash sync.sh
#   bash sync.sh --dry-run  # 只看会动哪些文件
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SRC:-$(cd "$HERE/.." && pwd)}"
DEST="$HERE/lab4"
PKGS=(planner_pkg trajectory_generation_pkg controller_pkg dependencies headless)

DRY=""
[ "${1:-}" = "--dry-run" ] && DRY="--dry-run"

say() { printf '\033[1;36m[sync]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[sync] %s\033[0m\n' "$*" >&2; exit 1; }

[ -d "$SRC/dependencies" ] || die "SRC=$SRC 下没有 dependencies/ —— 指向 lab4/ 了吗？"
[ -d "$SRC/controller_pkg" ] || die "SRC=$SRC 下没有 controller_pkg/"

command -v rsync >/dev/null 2>&1 || die "没有 rsync"

mkdir -p "$DEST"
for p in "${PKGS[@]}"; do
  [ -d "$SRC/$p" ] || { printf '\033[1;33m[sync]\033[0m 跳过（不存在）：%s\n' "$p"; continue; }
  say "同步 $p/"
  # --delete 保证删掉的源文件在镜像里也消失（
  #   --exclude __pycache__ 否则 .pyc 会被一起推上去，还带本机绝对路径）
  rsync -a --delete $DRY \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude '.DS_Store' \
    "$SRC/$p/" "$DEST/$p/"
done

say "镜像现状："
for p in "${PKGS[@]}"; do
  [ -d "$DEST/$p" ] || continue
  n=$(find "$DEST/$p" -type f | wc -l | tr -d ' ')
  s=$(du -sh "$DEST/$p" | cut -f1)
  printf '    %-28s %4s 文件  %6s\n' "$p" "$n" "$s"
done
printf '    %-28s %4s 文件  %6s\n' "合计" \
  "$(find "$DEST" -type f | wc -l | tr -d ' ')" "$(du -sh "$DEST" | cut -f1)"
say "完成。下一步：/Users/114514ma/.workbuddy/binaries/python/envs/default/bin/python push_via_api.py"
