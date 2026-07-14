# Codex 任务：实现 GEM 实时数据 → GMR-CPP 实时重定向 → Unitree E1，并同时可视化

## 工作目录

本机有两个同级项目，已经放入过一版可能有错误的桥接文件：

- GENMO：`/home/weili/GENMO`
- GMR-CPP：`/home/weili/GMR-CPP_e1jump_lowdpi`

请直接检查和修改这两个本地目录。不要重新 clone，不要执行或依赖之前损坏的 `.patch` 文件，不要 reset、checkout 或覆盖用户无关改动。开始前分别运行 `git status --short` 并保存现状；若存在未提交修改，保留它们，只修改本任务相关文件。

不要只给方案：完成代码修改、静态检查和可执行的本机测试。无法进行摄像头/GPU/GUI人工验证的部分，要明确列出，但仍要完成能自动完成的测试。

---

## 最终目标

实现以下实时链路：

```text
摄像头或视频文件
  → GENMO/GEM 实时推理
  → GENMO 主窗口显示实时视频、bbox、warmup/ready 和 UDP 状态
  → 复用 EnDecoder.fk_v2 得到 22 个 SMPL-X 关节的世界位置和世界旋转
  → Y-Up 转 Z-Up、初始 yaw 归零、初始水平位置与脚底高度归零
  → UDP GEM1 固定长度骨骼包
  → GMR-CPP GemReader
  → FrameQueue / MotionBuffer / GMR IK
  → 原仓库 E1 24DOF 模型 + 原仓库 xsens_to_e1.json
  → MuJoCo E1 实时窗口
  → Redis gmt_online_frame_e1，38×float32 = 152 字节
```

目标机器人只能是 **E1**。GEM 路径不得隐式使用 G1 的 XML、IK 配置、29 关节 preset、Redis key 或 172 字节格式。

---

# 一、必须先检查的本地文件

## GENMO

- `gem/network/endecoder.py`
- `gem/utils/motion_utils.py`
- `gem/gmr_udp_bridge.py`
- `scripts/demo/demo_webcam.py`

## GMR-CPP

- `apps/xsens_server.cpp`
- `apps/gem_server.cpp`
- `readers/base_reader.hpp`
- `readers/xsens_reader.hpp`
- `readers/xsens_reader.cpp`
- `readers/gem_reader.hpp`
- `readers/gem_reader.cpp`
- `include/gmr/frame_queue.hpp`
- `include/gmr/motion_buffer.hpp`
- `include/gmr/redis_publisher.hpp`
- `config/ik_configs/xsens_to_e1.json`
- `assets/e1/mjcf/e1_24dof.xml`
- `run_xsens_e1.sh`
- `run_gem.sh`
- `scripts/send_gem_test_packet.py`
- `CMakeLists.txt`
- `build.sh`

确认 `assets/e1/mjcf/e1_24dof.xml` 和 `config/ik_configs/xsens_to_e1.json` 确实存在。若路径在本地分支略有不同，使用本地真实 E1 24DOF 路径，但不要改成 G1。

---

# 二、修复 GENMO 端

## 1. 重构 `gem/gmr_udp_bridge.py`

当前文件可能自行调用 `make_smplx("supermotion")` 并重复做一套 FK。删除这个重复模型依赖。

桥接类只负责：

1. 接收已经算好的 22 个关节：
   - `joints_ay`: shape `(22, 3)`，GEM AY/Y-Up 世界位置
   - `rotations_ay`: shape `(22, 3, 3)`，对应世界旋转
2. 坐标转换、归一化、数据校验和 UDP 打包。
3. 不再加载第二个 SMPL-X body model。

建议公开接口：

```python
class GMRUDPBridge:
    def send_fk(
        self,
        joints_ay: torch.Tensor,
        rotations_ay: torch.Tensor,
        source_stamp_ns: int | None = None,
    ) -> None: ...

    def reset_origin(self) -> None: ...
    def close(self) -> None: ...
```

### 固定骨骼顺序

```python
BONE_NAMES = (
    "Pelvis",
    "Chest",
    "Left_UpperLeg",
    "Right_UpperLeg",
    "Left_LowerLeg",
    "Right_LowerLeg",
    "Left_Foot",
    "Right_Foot",
    "Left_UpperArm",
    "Right_UpperArm",
    "Left_Forearm",
    "Right_Forearm",
    "Left_Hand",
    "Right_Hand",
)

JOINT_INDICES = (0, 9, 1, 2, 4, 5, 7, 8, 16, 17, 18, 19, 20, 21)
```

这些名称必须与原仓库 `xsens_to_e1.json` 的 human body 名称匹配。不要改成 FBX 的 `Hips/LeftUpLeg`，也不要添加 G1 专属名称。

### 坐标转换

GEM rollout 输出 AY，即 Y-Up。GMR/MuJoCo 使用 Z-Up：

```text
[x, y, z]_GEM → [x, -z, y]_GMR
```

矩阵：

```python
C = [[1, 0, 0],
     [0, 0,-1],
     [0, 1, 0]]
```

世界旋转变换：

```python
R_zup = C @ R_ay
```

### 初始 yaw 归零

必须镜像原 `XsensReader::applyYawNorm()` 的行为：

1. 第一个有效帧，从转换到 Z-Up 后的 pelvis 旋转提取 Z 轴 yaw：
   `yaw = atan2(R[1,0], R[0,0])`
2. 保存 `R_yaw_inv = RotZ(-yaw)`
3. 每帧左乘所有世界位置和世界旋转。
4. 可选用户修正 `--gmr_yaw_deg` 必须在初始 yaw 归零之后应用，不能被初始归一化抵消：

```text
R_total = R_user_yaw @ R_yaw_inv @ C
p_out   = R_total @ p_ay
R_out   = R_total @ R_ay
```

### 初始原点

第一个有效帧在完成 yaw 处理后：

- 保存 pelvis 的 XY；
- 保存 `min(Left_Foot.z, Right_Foot.z)`；
- 后续位置减去 `[pelvis_x0, pelvis_y0, ground_z0]`；
- 只初始化一次，不能每帧贴地，否则会抹掉跳跃高度；
- `reset_origin()` 清除初始 yaw、XY 和 ground。

### UDP 协议

固定 little-endian：

```text
Header:
  magic             4 bytes = b"GEM1"
  version           uint16 = 1
  bone_count        uint16 = 14
  sequence          uint32
  source_stamp_ns   uint64

Payload:
  14 × (px, py, pz, qw, qx, qy, qz) float32
```

总长度必须严格为：

```text
20 + 14 × 7 × 4 = 412 bytes
```

要求：

- 四元数顺序 `wxyz`；
- 四元数归一化；
- 检查 NaN/Inf；
- 位置绝对值超过合理上限，例如 20 m，拒绝该帧；
- `source_stamp_ns` 默认 `time.monotonic_ns()`；
- 每 5 秒打印一次发送频率、sequence 和 packet bytes；
- 错误日志限频，不能每帧刷屏；
- `send_fk()` 只发送一包，不重发旧推理结果。

## 2. 修改 `scripts/demo/demo_webcam.py`

### 复用 `EnDecoder.fk_v2`

不要再通过桥接器加载第二套 SMPL-X。

在 `_run_backend()` 已经得到：

```python
pred_body_params_incam["body_pose"]
pred_body_params_incam["betas"]
body_params_global["global_orient"]
body_params_global["transl"]
```

后，整理成 `fk_v2` 需要的 `(B=1, L=1, ...)`：

```python
body_pose    -> (1, 1, 63)
betas        -> (1, 1, 10)
global_orient-> (1, 1, 3)
transl       -> (1, 1, 3)
```

调用：

```python
joints, _, fk_mat = self.endecoder.fk_v2(
    body_pose=...,
    betas=...,
    global_orient=...,
    transl=...,
    get_intermediate=True,
)
```

发送：

```python
joints[0, 0, :22]                 # (22,3)
fk_mat[0, 0, :22, :3, :3]        # (22,3,3)
```

发送调用必须位于 `_run_backend()` 内部，不能放在主显示循环，因为异步 pipeline 会多次返回同一个 `_pending_result`。

### CLI

保留或修正：

```text
--gmr_host        默认 None；不传则关闭 UDP
--gmr_port        默认 7001
--gmr_yaw_deg     默认 0
--gmr_scale       默认 1
```

增加：

```text
--display         在主线程打开 “GEM Live” OpenCV 视频窗口
```

### GEM 视频窗口

`--display` 必须满足：

- 在主线程使用 `cv2.imshow` 和 `cv2.waitKey`，避免 worker GUI 不稳定；
- 从第一帧开始显示，不能等 120 帧 warmup 完成后才出现；
- 无人、warmup、ready 三种状态都显示；
- 画出当前 bbox、track id、warmup/ready、FPS、GMR UDP on/off 和 sequence；
- `q` 退出；
- 退出时 `cv2.destroyAllWindows()`；
- 保留仓库已有 `--render --render_mode viser/opencv` 功能，不破坏它；
- 本任务默认运行指南使用 `--display`，因为它不额外启动第二个 SMPL-X 渲染进程。需要 GEM mesh overlay 时仍可使用原 `--render --render_mode opencv`。

### 线程和清理

- bridge 只在指定 `--gmr_host` 时创建；
- finally 中关闭 socket；
- 不影响未启用 GMR 时的原 GEM 行为；
- Python 异常必须打印，但不能让一次 UDP 发送失败杀死视频推理。

---

# 三、修复 GMR-CPP `GemReader`

## 1. `readers/gem_reader.hpp/.cpp`

实现 `BaseReader`，使用固定 UDP GEM1 协议。

### Config

至少包含：

```cpp
struct Config {
    std::string bind_ip = "0.0.0.0";
    int port = 7001;
    bool verbose = true;
};
```

### 接口

除 `BaseReader` 必需接口外，增加线程安全只读诊断：

```cpp
bool hasReceivedFrame() const;
double lastReceiveAgeMs() const;
uint64_t packetsReceived() const;
uint64_t packetsDropped() const;
uint64_t packetsInvalid() const;
```

### Socket

- UDP；
- `SO_REUSEADDR`；
- `SO_RCVBUF` 至少 1–4 MB；
- `SO_RCVTIMEO` 100 ms，保证 disconnect 能退出；
- 支持 `--bind` 指定地址；
- disconnect 必须可靠 join；
- 显式包含所需标准头，不能依赖传递 include，例如 `<cstring>`, `<limits>`, `<chrono>`, `<cmath>`。

### 解析

- 不要直接依赖 C++ struct 对齐；
- 按 little-endian 显式读取 header 和 float；
- 严格要求 412 字节；
- magic/version/bone_count 不匹配则 invalid；
- 固定映射到上面的 14 个 canonical names；
- 位置和四元数检查 finite；
- 四元数 norm 太小则拒绝，其他情况归一化；
- sequence：
  - 重复帧拒绝；
  - 旧帧/乱序拒绝；
  - 正向跳号累计 drop；
  - 正确处理 uint32 wrap-around；
- `RawFrame.stamp_ns` 必须使用接收机本地：
  `duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count()`；
  不要把 Python epoch/monotonic 时间直接作为 MotionBuffer 时间；
- `RawFrame.frame_number` 使用 sequence；
- 每 5 秒打印 recv Hz、drop、invalid、queue drop 和 last age。

不要对骨骼做第二次 Y-Up/Z-Up 转换；转换已经在 GENMO 端完成。

---

# 四、重写 `apps/gem_server.cpp` 为 E1 正确版本

以本地当前 `apps/xsens_server.cpp` 为结构模板，保留它成熟的：

- FrameQueue；
- MotionBuffer；
- seed；
- IK re-warm；
- stale frame flush；
- MuJoCo Viewer；
- Redis publisher；
- 发布诊断；
- joystick gate；
- signal handling。

只把 XsensReader 替换成 GemReader，并做下列 E1 修正。

## 1. 默认必须是 E1

```text
preset        = e1
XML           = assets/e1/mjcf/e1_24dof.xml
IK config     = config/ik_configs/xsens_to_e1.json
Redis key     = gmt_online_frame_e1
num joints    = 24
payload       = 38 float32 = 152 bytes
```

必须调用：

```cpp
cfg.redis.applyPreset(gmr::presetE1());
```

不要默认调用 `presetG1()`。如果保留 `--preset g1` 兼容，只有用户明确传 `g1` 才能启用；默认与 `run_gem.sh` 必须始终是 e1。

不要使用、引用或默认到：

```text
config/ik_configs/gem_to_g1_position.json
assets/unitree_g1/...
mmocap_motion_frame_g1
29 joints
43 floats / 172 bytes
```

已存在的 `gem_to_g1_position.json` 可以留在磁盘，但 GEM→E1 路径不得引用它。

## 2. 正确的仓库根路径

可执行文件在 `<repo>/build/gem_mocap_server`。不要使用会跑到仓库上一级的错误 `../../assets`。

使用：

```cpp
fs::path exe_dir = fs::canonical(argv[0]).parent_path();
fs::path repo_root = exe_dir.parent_path();
```

默认路径从 `repo_root` 拼接。

即使 run 脚本会显式传路径，server 自身默认路径也要正确。

## 3. CLI

至少支持：

```text
--bind                  默认 0.0.0.0
--port                  默认 7001
--preset                默认 e1
--xml
--ik-config
--redis-host            默认 127.0.0.1
--redis-port            默认 6379
--redis-db              默认 0
--redis-key             E1 默认 gmt_online_frame_e1
--hz                    默认 50
--ttl-ms                默认 200
--stale-ms              默认 250
--lin-vel-alpha
--ang-vel-alpha
--lin-vel-max
--ang-vel-max
--always
--vis
--viewer-width
--viewer-height
--offset-to-ground
--no-offset-to-ground
--help
```

GEM 端已经只在初始化时对齐脚底。为了保留跳跃高度，GMR 端 `offset_to_ground` 默认必须为 `false`；仅用户明确传 `--offset-to-ground` 才开启。

E1 默认不做 G1 的 Spine1 6° offset。可以完全不提供 spine offset，或明确保持关闭。

## 4. stale 安全逻辑

原发布循环会重复发布 `latestProcessedFrame()`。如果 GEM 停止，它可能持续刷新 Redis TTL。

发布前增加：

```text
如果尚未收到 GEM 帧，跳过发布；
如果 reader.lastReceiveAgeMs() > stale_ms，跳过发布；
```

这样在 `stale_ms=250` 后不再刷新 Redis，随后 `ttl_ms=200` 到期，最迟约 450 ms 后 E1 key 消失。

stale 时每秒最多打印一次警告，不能刷屏。

MuJoCo viewer 可以保留最后姿态，但 Redis 必须停止刷新。

## 5. Viewer

`--vis` 创建 E1 XML 的 `MujocoViewer`，实时渲染 `latestProcessedFrame().qpos`。

启动日志必须明确打印：

```text
preset=e1
xml=...
ik=...xsens_to_e1.json
redis key=gmt_online_frame_e1
joints=24
frame=38 floats / 152 bytes
bind/port
stale_ms
offset_to_ground=off
```

---

# 五、启动脚本

## `run_gem.sh`

将其改为 E1 专用且健壮：

- `#!/usr/bin/env bash`
- `set -euo pipefail`
- 使用 Bash 数组解析和转发参数，不要用会破坏空格的单字符串 `EXTRA_ARGS`；
- 默认：

```bash
EXECUTABLE="$ROOT/build/gem_mocap_server"
BIND="0.0.0.0"
PORT="7001"
PRESET="e1"
XML="$ROOT/assets/e1/mjcf/e1_24dof.xml"
IK_CONFIG="$ROOT/config/ik_configs/xsens_to_e1.json"
REDIS_HOST="127.0.0.1"
REDIS_PORT="6379"
REDIS_KEY="gmt_online_frame_e1"
HZ="50"
TTL_MS="200"
STALE_MS="250"
```

- 显式向可执行文件传：
  `--preset e1 --xml ... --ik-config ... --redis-key gmt_online_frame_e1`
- 支持 `--always`、`--vis`、`--viewer-width`、`--viewer-height`、速度滤波和 Redis 参数；
- 默认不传 `--offset-to-ground`；
- 启动前验证 executable/XML/IK 文件；
- `chmod +x run_gem.sh`。

如果认为名称更清楚，可以新增 `run_gem_e1.sh`，但必须让现有 `run_gem.sh` 成为调用它的兼容入口，避免用户现有命令失效。

---

# 六、CMake

确保 `CMakeLists.txt` 中 `gem_mocap_server` 只定义一次：

```cmake
add_executable(gem_mocap_server
    apps/gem_server.cpp
    readers/gem_reader.cpp
)
```

include 和 link 参照 `xsens_mocap_server`：

- repo root
- `include`
- `readers`
- MuJoCo include
- MuJoCo
- GLFW
- hiredis
- Eigen
- nlohmann_json
- pthread
- GL
- DAQP

不能依赖 NatNet。

如果之前安装脚本已经追加过目标，修改现有块，不要再追加第二块。

---

# 七、测试发送器

检查并修正 `scripts/send_gem_test_packet.py`：

- 发送同一 GEM1 412-byte 协议；
- 14 个 canonical names 对应的固定姿态；
- 默认 127.0.0.1:7001、30 Hz；
- sequence 递增；
- `source_stamp_ns=time.monotonic_ns()`；
- 支持 `--seconds`，便于自动测试；
- Ctrl+C 正常退出；
- 启动时打印 packet bytes=412。

该脚本只测试网络和 GMR，不代表真实人体重定向质量。

---

# 八、自动检查和测试

完成修改后必须执行并修复发现的问题。

## GENMO

```bash
cd /home/weili/GENMO

python3 -m py_compile \
  gem/gmr_udp_bridge.py \
  scripts/demo/demo_webcam.py

python scripts/demo/demo_webcam.py --help | \
  grep -E 'display|gmr_host|gmr_port|gmr_yaw_deg|gmr_scale'
```

添加一个无需模型权重的轻量测试，例如 `tests/test_gmr_udp_bridge.py` 或独立脚本，至少验证：

- packet size 412；
- magic/version/bone_count；
- 14×7 float；
- identity quaternion 输出有效；
- AY→Z-Up 的一个已知向量；
- 初始 yaw 归零；
- NaN 被拒绝。

测试不能初始化完整 SMPL-X，因为桥接器已经不应拥有 body model。

## GMR-CPP

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

bash build.sh --clean

test -x build/gem_mocap_server
build/gem_mocap_server --help
./run_gem.sh --help
```

检查：

```bash
grep -RIn \
  -e 'gem_to_g1_position' \
  -e 'unitree_g1' \
  -e 'mmocap_motion_frame_g1' \
  apps/gem_server.cpp run_gem.sh
```

该 grep 对 GEM/E1 文件应无匹配；若为了显式拒绝 G1 而出现在错误信息中，需要解释。

运行不带 GUI的 UDP 冒烟测试时，不要长期遗留后台进程。若 Redis 可用：

```bash
redis-cli ping
```

则可以启动 `gem_mocap_server` 无 `--vis`、运行测试发送器 2–3 秒，并验证：

```bash
redis-cli STRLEN gmt_online_frame_e1
```

必须返回：

```text
152
```

停止发送器后等待 1 秒，再验证：

```bash
redis-cli EXISTS gmt_online_frame_e1
```

必须返回：

```text
0
```

若本地自动环境没有 Redis、DISPLAY、摄像头或模型权重，不要伪造通过；记录哪些测试无法自动执行。

---

# 九、文档

在 GMR-CPP 中新增或更新：

```text
docs/GEM_TO_E1.md
```

内容包括：

- 数据流；
- 两个项目的修改文件；
- UDP 协议；
- E1 默认 XML、IK、preset、Redis key；
- 编译；
- synthetic test；
- 真实 webcam/video 运行；
- GUI/DISPLAY 要求；
- 120 帧 GEM warmup；
- Redis 152 字节检查；
- stale/TTL 检查；
- yaw 参数调试；
- 常见故障。

---

# 十、最终验收标准

必须全部满足代码层面的验收：

1. `gem/gmr_udp_bridge.py` 不再 import/call `make_smplx`。
2. GEM 使用现有 `self.endecoder.fk_v2(..., get_intermediate=True)` 生成关节位置和世界旋转。
3. GEM `--display` 从第一帧打开实时视频窗口，`q` 可退出。
4. UDP 包严格 412 字节。
5. GemReader 输出名称与 `xsens_to_e1.json` 匹配。
6. GemReader 使用本机 `steady_clock` 纳秒时间。
7. `gem_server` 默认和 `run_gem.sh` 显式使用：
   - `preset=e1`
   - `assets/e1/mjcf/e1_24dof.xml`
   - `config/ik_configs/xsens_to_e1.json`
   - `gmt_online_frame_e1`
8. RedisPublisher 应显示 `joints=24`、`frame=38 floats`；Redis value 长度 152 字节。
9. `--vis` 打开 E1 MuJoCo viewer。
10. GEM 停止后不会一直刷新旧帧；约 1 秒后 E1 Redis key 不存在。
11. GEM 路径不引用 `gem_to_g1_position.json`。
12. `bash build.sh --clean` 编译成功。
13. 不破坏原 Xsens/OptiTrack/FZMotion 的现有 target 和脚本。
14. 最后输出：
    - 修改文件清单；
    - 关键设计说明；
    - 自动测试结果；
    - 未能自动执行的 GUI/GPU 测试；
    - 完整运行命令；
    - `git diff --stat` 和关键 diff 摘要。

请现在开始检查并完成修改，不要停留在计划阶段。
