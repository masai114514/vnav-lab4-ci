#!/usr/bin/env bash
# =============================================================================
# fetch_artifacts.sh —— 把某次 official-race 运行的产物拉回本地
#
# 用法：
#   bash fetch_artifacts.sh                 # 取最近一次成功运行
#   bash fetch_artifacts.sh <run_id>        # 取指定运行
#   bash fetch_artifacts.sh <run_id> <dest> # 指定落地目录
#   DEST=lab4/official_race_r5 bash fetch_artifacts.sh
#
# 落地内容（默认 DEST=<lab4>/official_race_rN）：
#   race.bag / race_video.mp4 / race_video_full.mp4 / metrics.json / race_tracking.png
#   lag_probe.txt / lag_probe.csv / console.log / 各类日志 / fps_*.png
#
# 顺带记一笔 run 元信息（run_meta.txt），便于事后对账「哪一趟是什么参数」。
# =============================================================================
set -euo pipefail

REPO="${REPO:-masai114514/vnav-lab4-ci}"
WORKFLOW="${WORKFLOW:-official-race.yml}"
ARTIFACT="${ARTIFACT:-official-race}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB4="$(cd "$HERE/.." && pwd)"

RUN_ID="${1:-}"
if [ -n "${2:-}" ]; then
  DEST="$2"
elif [ -n "${DEST:-}" ]; then
  : # 用外部传入的
else
  DEST="$LAB4/official_race_r$(date +%m%d)"
fi

say()  { printf '\033[1;36m[fetch]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[fetch]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fetch] %s\033[0m\n' "$*" >&2; exit 1; }

say "仓        = $REPO"
say "工作流    = $WORKFLOW"
say "落地目录  = $DEST"

# ---- 定 run_id -------------------------------------------------------------
if [ -z "$RUN_ID" ]; then
  say "找最近一次运行 ..."
  RUN_ID="$(gh run list --repo "$REPO" --workflow "$WORKFLOW" --limit 30 \
              --json databaseId,conclusion,status,createdAt \
              --jq '[.[] | select(.status == "completed")][0].databaseId' 2>/dev/null || true)"
  [ -n "$RUN_ID" ] && [ "$RUN_ID" != "null" ] || \
    die "找不到已完成的运行。用 gh run list --repo $REPO --workflow $WORKFLOW 看看"
fi

say "run_id    = $RUN_ID"
mkdir -p "$DEST"

# ---- run 元信息 ------------------------------------------------------------
{
  echo "repo        : $REPO"
  echo "workflow    : $WORKFLOW"
  echo "run_id      : $RUN_ID"
  echo "下载时间    : $(date '+%Y-%m-%d %H:%M:%S %z')"
  echo
  echo "--- gh run view ---"
  gh run view "$RUN_ID" --repo "$REPO" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' || true
} > "$DEST/run_meta.txt"
say "元信息 → $DEST/run_meta.txt"
grep -E "^(✓|X|✗|Status|Conclusion|run_id)" "$DEST/run_meta.txt" | head -5 || true

# ---- 拉 artifact -----------------------------------------------------------
# 先问一下大小。100 MB+ 的 artifact 走 `gh run download` 容易超时/被杀，
# 所以下面备了一条 REST zip 端点的回退路径。
AID="$(gh api "repos/$REPO/actions/runs/$RUN_ID/artifacts" \
        --jq ".artifacts[] | select(.name==\"$ARTIFACT\") | .id" 2>/dev/null | head -1 || true)"
if [ -n "$AID" ]; then
  SZB="$(gh api "repos/$REPO/actions/artifacts/$AID" --jq '.size_in_bytes' 2>/dev/null || echo '?')"
  say "artifact id=$AID  size=${SZB} bytes ($(( ${SZB:-0} / 1048576 )) MB)"
else
  warn "没查到名为 '$ARTIFACT' 的 artifact（可能没传成功）"
fi

say "下载 artifact '$ARTIFACT' ..."
ok=0
if gh run download "$RUN_ID" --repo "$REPO" --name "$ARTIFACT" --dir "$DEST"; then
  ok=1
fi

if [ "$ok" != 1 ] && [ -n "$AID" ]; then
  warn "gh run download 失败 —— 退回 REST zip 端点（大 artifact 更稳）"
  # zip 落在系统临时目录，别往 Desktop 下写中间文件
  TMPZ="$(mktemp -d)/artifact.zip"
  if curl -fL --retry 3 --retry-delay 5 --max-time 1800 \
        -H "Authorization: Bearer $(gh auth token)" \
        -H "Accept: application/vnd.github+json" \
        -o "$TMPZ" \
        "https://api.github.com/repos/$REPO/actions/artifacts/$AID/zip"; then
    unzip -q -o "$TMPZ" -d "$DEST" && ok=1
  fi
fi
[ "$ok" = 1 ] || die "artifact 下载失败"

# ---- 整理：artifact 可能带一层 out/ ----------------------------------------
if [ -d "$DEST/out" ]; then
  say "artifact 里带了一层 out/，摊平"
  shopt -s dotglob nullglob
  mv "$DEST/out/"* "$DEST/" 2>/dev/null || true
  rmdir "$DEST/out" 2>/dev/null || true
  shopt -u dotglob nullglob
fi

say "落地内容："
ls -la "$DEST" | sed 's/^/    /'

echo
say "快速判读："
if [ -f "$DEST/lag_probe.txt" ]; then
  echo "  ★ /current_state 滞后（关键四行）"
  grep -E "^\s*\[C\]|p95 lag|n=[0-9]+" "$DEST/lag_probe.txt" | head -4 | sed 's/^/      /' || true
else
  warn "  没有 lag_probe.txt"
fi
if [ -f "$DEST/metrics.json" ]; then
  echo "  metrics.json:"
  cat "$DEST/metrics.json" | sed 's/^/      /' | head -20
fi
if [ -f "$DEST/race.bag" ] || [ -f "$DEST/race.bag.active" ]; then
  echo "  bag: 有"
else
  warn "  bag: 无"
fi
if [ -f "$DEST/race_video.mp4" ]; then
  echo "  video: 有 ($(du -h "$DEST/race_video.mp4" | cut -f1))"
else
  warn "  video: 无"
fi

echo
say "下一步："
echo "    # 位置/姿态振荡 + 包络（脚本已随仓保存，直接跑）"
echo "    /Users/114514ma/.workbuddy/binaries/python/envs/default/bin/python $LAB4/headless/osc_report.py --bag $DEST/race.bag"
echo "    # 看视频"
echo "    open $DEST/race_video_full.mp4 2>/dev/null || open $DEST/race_video.mp4"
