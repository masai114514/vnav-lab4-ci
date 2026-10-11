# vnav-lab4-ci —— 官方 Unity(TESSE) 竞速链路的 CI 载体

MIT 16.485 VNAV **Lab 4** 的交付物里，有两件必须在**官方 Unity 仿真**里跑出来：

1. 竞速视频（Unity 窗口录屏）
2. 最快 rosbag（`rosbag record` 那一趟）

本机（Apple Silicon Mac）做不了 —— 不是没有桌面、也不是架构不匹配（Xvfb 与 `x86_64`
容器都已实测 OK），而是 **Mono 的 JIT trampoline 在 Rosetta 转译下越出 ±2 GB 约束而断言自杀**：
`* Assertion: should not be reached at tramp-amd64.c:596`。这条在本机结构性无解。

这个仓就是把它搬到**原生 x86_64 Linux** 上跑。用 GitHub Actions 的免费 runner：
runner 本身是原生 amd64，再用 `ros:noetic-ros-base` 容器拿到原版 focal + ROS Noetic，
两个问题（无 Rosetta / 无 Noetic deb）一起解决，**不需要租任何带 GPU 的云机**。

## 怎么用

```bash
# 1) 改完 lab4 里的控制器 / launch / headless 脚本后，先同步进本仓
bash sync.sh

# 2) 推上去（本机 git push 走代理必挂，所以走 gh 的 Git Data API）
/Users/114514ma/.workbuddy/binaries/python/envs/default/bin/python push_via_api.py --message "ci: xxx"

# 3) 触发运行（网页 Actions 里点 Run workflow，或命令行）
gh workflow run official-race.yml --repo masai114514/vnav-lab4-ci

# 4) 等跑完取产物
bash fetch_artifacts.sh              # 取最近一次完成的运行
bash fetch_artifacts.sh <run_id>     # 取指定运行
```

## 目录

| 路径 | 是什么 |
|---|---|
| `.github/workflows/official-race.yml` | 工作流。容器 `ros:noetic-ros-base`，`workflow_dispatch` 带参数（分辨率 / 控制频率 / 录屏开关 …） |
| `ci/official_race.sh` | ★ 一条龙主脚本：装依赖 → 建 catkin 工作区 → 拉桥接 → 下 Unity → Xvfb+llvmpipe → **端口判定就绪** → 按 W 上电 → 起控制器 → 录 bag + 录屏 → 并行 `lag_probe` → 分析 |
| `ci/probe_unity.sh` | 快探针：只验「Unity 起得来 + 桥接连得上」，迭代时省一整趟 |
| `sync.sh` | 把 `../`（lab4）里 CI 要用的 5 个目录 rsync 进 `lab4/` |
| `push_via_api.py` | 绕过挂掉的 `git push`，走 GitHub Git Data API 推仓（二进制安全、只传改动、顺手删远端多余文件） |
| `fetch_artifacts.sh` | 拉回 artifact 并打印关键判读 |
| `lab4/` | `sync.sh` 生成的源码镜像（**不要手改**，改了下次 sync 就没了） |

## 三个容易踩回去的坑（都已在脚本里注释）

1. **`use_broadcast` 必须保持官方默认 `true`。**
   `tesse_ros_node.py` 里是 `if self.use_broadcast: udp_host = '<broadcast>'`，
   改成 `false` 会把 UDP 监听绑到 `self_ip:9004` 的**单播**地址，而 Unity 往广播地址
   发 metadata → 直接丢 `/current_state`，整条链路废掉。

2. **等 Unity 就绪要用「端口能连上」，不能用「睡 N 秒」。**
   开发构建首次加载场景很慢，写死等待会在 `loading new scene...` 就判负 →
   桥接空等 → 整趟没有 bag。

3. **`bash x.sh | tee log` 的退出码来自 `tee`。**
   不在 workflow 里加 `set -o pipefail`，脚本失败也会显示 success、留下空产物。

## 产物

artifact 名 `official-race`，含：

```
race.bag             竞速 rosbag                         ← 交付物 [2]
race_video.mp4       Unity 窗口录屏（CAPFPS 帧率）
race_video_full.mp4  上采样版（UPSCALE 非空时）
metrics.json         位置 RMSE / 过门数 / 最大速度
race_tracking.png    实际 vs 期望轨迹
lag_probe.txt/.csv   ★ 控制器看到的 /current_state 到底有多旧
osc_report.txt/.csv  ★ xyz 逐轴误差 + 滑动 RMS 包络 + roll/pitch
console.log          完整控制台日志
unity.log bridge.log traj_following.log traj_gen.log
fps_*.png            左上角帧时浮层定时截图（核对真实帧率用）
```

## 权限说明

`push_via_api.py --create` 需要 token 有 `repo`；删仓需要 `delete_repo`。
本机 `gh` 的 token 目前是 `delete_repo, gist, read:org, repo, workflow`。
