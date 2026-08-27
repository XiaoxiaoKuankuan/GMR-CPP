# SMPL-X → BUMI3 实时链路改动日志

## 2026-08-24：速度上限同步 Noetix `bumi.py`

### 参考来源

`/home/weili/legged_lab/source/NoetixRobot/NoetixRobot/assets/robots/bumi3/bumi.py`
中的 `Bumi_CFG.actuators[*].velocity_limit_sim`：

- `waist_yaw_joint`：`9 rad/s`；
- 双腿 hip/knee、双脚踝和双臂：`12 rad/s`。

### 改动内容与理由

- `realtime_motion_guard.hpp` 增加逐关节速度上限表，不能再只用“全身/手臂”两个
  统一数字，否则腰关节会被错误放宽到 `12 rad/s`；
- `smplx_to_bumi3_auto.json` 明确记录全部 21 个关节：腰为 `9 rad/s`，其余为
  `12 rad/s`，同时保存参考文件路径便于追溯；
- 脚接触 QP 的通用速度上限同步为 `12 rad/s`，最终输出仍由逐关节保护器把腰限制
  在 `9 rad/s`；
- `bumi.py` 未定义关节加速度限制，所以没有根据速度值猜测加速度，继续保留全身
  `80 rad/s²`、手臂 `60 rad/s²`；
- C++ 单元测试和 Python/C++ 配置校验新增 `9/12 rad/s` 精确断言，防止以后配置漂移。

### 本次验证

- 重新编译 `smplx_bumi3_server` 及相关测试目标，编译通过；
- 相关 CTest 共 5 项全部通过；
- 独立配置校验脚本通过，并确认速度表包含 21 个关节、腰为 `9 rad/s`、其余为
  `12 rad/s`。

## 2026-08-24：速度/加速度、手臂跳变与接触感知保护

### 改动目的

在不增加未来帧等待、不重复运行 IK 的前提下，限制实时 SMPL-X 输出的关节动态，
阻止单帧手臂换解直接发送给机器人，并把仓库中已有的脚接触、支撑脚和腾空能力接入
`smplx_bumi3_server`。同时把 BUMI3 离地参考统一为 `0.04 m`。

### 代码和配置改动

- `include/gmr/realtime_motion_guard.hpp`
  - 新增固定规模、因果式关节保护器；
  - 全身关节执行速度和加速度硬限制；
  - 左右手臂分别检测大幅换解，首帧保持、第二帧确认；
  - 实时 `apply()` 只扫描模型中的单自由度关节，不分配容器、不重跑 IK。
- `include/gmr/gmr_mink.hpp`
  - 在 IK 后、脚底最终投影前调用实时保护器；
  - 新增首帧固定 ground offset 标定接口；
  - 支撑脚选择不再依赖 G1 专用 motion-preserving 开关，使 SMPL-X 可以使用；
  - 增加保护器启用、时间戳输入和诊断接口。
- `include/gmr/foot_contact.hpp`
  - 增加从 SMP1 `BodyMap` 直接生成脚参考点的重载，复用已有接触检测器。
- `include/gmr/motion_buffer.hpp`
  - 增加求解前回调，把当前 SMP1 时间戳和脚接触状态在同一处理线程中送入 GMR；
  - 未设置回调的 G1/E1/Xsens 等原有路径保持原行为。
- `apps/smplx_server.cpp`
  - BUMI3 默认读取并启用实时安全与脚接触配置；
  - 首帧锁定源地面，之后保留真实竖直运动和腾空；
  - 接入支撑脚切换、真实机器人脚底约束、接触与安全累计诊断；
  - 输入断流超过 `0.2 s` 时重置接触检测状态；
  - 增加显式开启/关闭和权重命令行参数。
- `config/ik_configs/smplx_to_bumi3_auto.json`
  - 初版增加全身 `6 rad/s`、`80 rad/s²` 动态限制；速度值随后已按上节的
    `bumi.py` 参考改为逐关节 `9/12 rad/s`，加速度仍为 `80 rad/s²`；
  - 初版增加手臂 `4 rad/s`、`60 rad/s²` 和 `0.60 rad` 跳变确认配置；手臂速度
    随后已按上节改为 `12 rad/s`，加速度和跳变阈值保持不变；
  - 增加允许腾空的 SMPL-X 脚检测与 BUMI3 真实脚底约束配置。
- `run_smplx_bumi3.sh`、`run_smplx_bumi3_jump.sh`
  - 离地参考统一为 `0.04 m`；
  - 普通脚本改为固定首帧标定，并显式打开接触和实时安全保护。
- `scripts/validate_smplx_to_bumi3.py`
  - 校验安全阈值、手臂关节名称、源/目标脚名称和 `allow_flight`。
- `tests/realtime_motion_guard_test.cpp`
  - 使用真实 BUMI3 模型验证速度、加速度和两帧手臂跳变确认。
- `tests/smplx_bumi3_config_test.cpp`、`tests/ground_clearance_test.cpp`
  - 增加接触/安全配置检查，并把 BUMI3 clearance 期望更新为 `0.04 m`。
- `CMakeLists.txt`
  - 注册新的 `realtime_motion_guard_test`；保留工作区原有 batch server/viewer 改动。

### 实时性设计理由

整段轨迹 QP 需要等待未来帧，不适用于在线控制。本次保护只使用当前输出、上一帧输出
和上一帧速度；每帧最多处理固定的 21 个关节。手臂异常时也不重试 IK，而是用下一帧
做一次确认，所以正常路径无额外帧延迟，异常路径最多增加一帧确认。

### 验证记录

- `smplx_bumi3_server` 在原 `build/` 和独立 Release 测试目录均编译通过；
- CTest 相关 6 项全部通过：协议解析、BUMI3 配置、`0.04 m` clearance、脚接触
  检测、实时关节保护、G1 脚约束回归；
- 使用 650 个模拟 SMP1 包进行完整 UDP → IK → 保护链路测试，输入和重定向均保持
  `49.3 Hz` 左右，队列丢帧为 `0`，`slow > 50 ms` 为 `0`；
- 完整链路实际观察到双脚支撑、右脚主支撑、双脚腾空和重新落地状态；
- 运行中速度限制和加速度限制累计计数均发生变化，证明保护器已进入真实 SMPL-X
  输出路径，而不是只存在于未调用代码中；
- 短时无 Redis 启动检查确认默认打印 `ground_clearance=0.040`、脚接触开启和实时
  安全开启。
