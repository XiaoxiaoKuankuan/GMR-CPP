# SMPL-X → BUMI3 实时链路改动日志

## 2026-08-27：修复 batch 落脚卡顿与 5 cm 悬浮

### 问题与根因

首次接入 batch 接触约束后，FineDance 完整动作出现可见卡顿和整机悬浮。逐帧检查确认
是两个独立问题：每次 stance 进入都会对上一帧锚点执行多轮 `settleFootContacts()`，把
同一个输入帧推进多次并产生根节点跳变；GENMO 验证脚本又把历史
`--ground-clearance 0.05` 同时用于人体源标定和机器人输出，直接引入约 5 cm 悬浮。
此外，长动作中的人体源地面会缓慢漂移，单纯永久锁住首帧地面会把后半段误判成长腾空。

### 修复实现

- batch 只在 `RESET` 做一次 contact-only 初始化收敛；真实落脚先释放新脚、跟随当前
  人体帧一次，再从当前机器人姿态捕获锚点，约束从下一输入帧渐进启用，不再在接触进入
  时额外 settle，也不再让一个 SMP1 帧重复推进 IK；
- 人体源标定新增独立 `--source-ground-clearance`，默认 `0 m`；机器人 stance 仍由
  JSON 中的鞋底中心 `1--3 mm` 与不穿地约束决定，二者不再复用一个 clearance；
- 源地面跟踪器仅在上一帧有可靠支撑时以不超过 `0.04 m/s` 慢跟踪；普通 flight 冻结，
  连续 45 帧后且脚运动稳定时才以不超过 `0.12 m/s` 渐进重捕获，避免固定地面漂移和
  单帧重新贴地；
- 接触进入/退出改为 4/2 帧滞回；移动双支撑中的弱支撑侧保持一致，不再逐帧左右切换；
  强制移动支撑只负责鞋底平面和高度，不建立世界 XY 防滑锚点；
- batch 输出新增 `0.75 m/s` 的因果根节点水平限速。没有对根节点 Z 或整套 qpos 做
  输出后插值：验证表明那会破坏已满足的鞋底平面约束，因此竖直方向继续由同一帧 IK
  和接触 QP 联合求解。

### 验证结果

- GMR 完整 CTest `9/9` 通过；FineDance `finedance_01` 共 1851 帧数值回放有限；
- 修复前 5 cm clearance 输出中，双脚最低点同时高于地面 20 mm 的帧数为
  `1350/1851`；修复后为 `0/1851`，逐帧两脚最低点最小值的 P50/P95 为
  `0.707/1.511 mm`，无可测穿地；
- 修复前根节点单帧最大位移 `8.89 cm`，接触进入处 P95 `6.20 cm`；修复后全段
  P50/P95/P99/最大为 `0.663/1.578/2.140/3.841 cm`，水平位移最大 `1.500 cm`；
- 只在鞋底中心低于 4 mm 的近地帧检查平面：左右脚前后高度差 P95 分别
  `3.001/2.985 mm`，左右高度差 P95 为 `4.540/4.225 mm`。该统计包含状态切换帧；
  stance 内的硬约束仍是前后/左右各 `3 mm`。

### 兼容边界

旧 qpos 的 binary/IK/clearance 身份均不再匹配，必须重定向后再渲染；
`--no-foot-contact-constraints` 仍保留旧无接触路径。本修复是运动学接触和离线输出平滑，
不代表 GMT 动力学稳定性已通过仿真或实机验证。

## 2026-08-27：batch 接触约束启用与支撑脚硬平面约束

### 修改目标

修复同步 `smplx_bumi3_batch_server` 虽然读取了 `foot_contact.enabled=true`，但没有
初始化接触状态，导致 GENMO 完整音乐离线重定向实际仍走无脚底约束 IK 的问题。同时
把“鞋底法向软权重”补成支撑期可验证的脚尖/脚跟、内外侧高度差硬约束，避免人体脚
已经平放而 BUMI3 仍以前掌或后跟单点着地。

### 主要实现

- `apps/smplx_bumi3_batch_server.cpp`
  - 复用实时服务的 SMP1 源脚接触检测、首帧固定地面标定和真实 BUMI3 脚 mesh 对地；
  - `RESET` 先收敛人体 IK，再初始化左右支撑锚点；后续每帧调用
    `setFootContactState()`，stance/swing/flight 随输入自动切换；
  - 接触模式忽略旧的逐帧 `offset-to-ground`，因此腾空高度不会再被每帧最低脚归零；
  - 新增显式开关和权重参数，`--no-foot-contact-constraints` 可回退兼容路径；
  - `RESET` 做一次 contact-only 可行投影；落脚阶段的首次实现曾在每次进入接触时
    重复 settle，已由同日上节改成当帧跟随、当帧捕获锚点、下一帧启用约束。
- `readers/smplx_reader.cpp`
  - `decodePacket()` 保留 SMP1 生产端时间戳，batch 接触速度按真实 30/50 Hz 计算；
    UDP 实时入口仍在 `parsePacket()` 中覆盖为本机接收时间，不改变原实时语义。
- `include/gmr/gmr_mink.hpp`、`include/gmr/foot_contact*.hpp`
  - 支撑期约束 `|toe_z-heel_z|<=3 mm`、
    `|lateral_positive_z-lateral_negative_z|<=3 mm`；两项只含世界 Z 点雅可比，
    因此约束 roll/pitch 而不锁定 yaw；
  - 鞋底中心限制在 `1--3 mm` 高度带，真实 mesh 顶点继续执行不可穿透约束；
  - 平面约束不可在 DAQP fallback 中被删除；XY 防滑仍只施加到可靠支撑脚，检测器
    选出的移动双支撑弱支撑脚只平整/对地，不锁死舞蹈转向和滑步；
  - 新增初始化/落脚 contact-only 收敛和鞋底中心、前后、左右诊断字段。
- `config/ik_configs/smplx_to_bumi3_auto.json`
  - 启用 `hard_support_constraints`，显式记录前后/左右 `3 mm` 容差；
  - 保留原接触检测阈值、允许腾空、脚底 XY 锚点和关节速度边界。

### 兼容性与功能边界

- 默认配置下 batch 输出语义发生预期改变，旧 `bumi_qpos30.pt` 必须依据新 batch
  binary/IK SHA 重新生成，不能只替换视频；
- 配置未启用脚接触，或命令行指定 `--no-foot-contact-constraints` 时，仍使用旧的
  逐帧 grounding/IK 路径；
- 硬平面约束只在 stance 生效，swing/flight 立即释放；它改善运动学接触，但不替代
  GMT 的 COM、支撑多边形和动力学可行性验证。

### 实际验证结果

- `build/` 完整 CTest 9/9 通过；`build-genmo-stream/` 接触相关 CTest 5/5 通过；
- 人工冲突测试持续给人体双脚 `0.45 rad` 前倾目标，BUMI3 支撑脚最终倾角为
  `0.51°/0.67°`，前后高度差 `1.18/0.44 mm`，中心高度 `2.18/2.04 mm`，
  DAQP 成功且未降级；
- 对现有完整 `aioz_gdance_20`（1161 帧）使用同一 SMPL 参数重新 batch 重定向，
  接触状态机识别出 692 个左右支撑脚帧：倾角 P50/P95 从
  `10.55°/23.93°` 降至 `1.33°/2.11°`；脚尖/脚跟差 P50/P95 从
  `16.40/37.76 mm` 降至 `0.48/1.47 mm`，最大 `2.995 mm`；内外侧 P95
  `2.23 mm`，中心高度 P50/P95 `2.28/2.91 mm`，最低点未穿地；
- 该完整音乐逐帧求解耗时均值约 `2.84 ms`、P95 `6.11 ms`；一次性 RESET/初始
  1000 次 warmup 的最大响应约 `344 ms`，不计入稳定逐帧控制预算。

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
