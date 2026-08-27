# SMPL-X → BUMI3 固定实时重定向链路

## 数据链路

```text
GENMO SMPL-X FK
  → SMP1 UDP（固定14个人体 target）
  → SmplxReader（不变）
  → smplx_to_bumi3_auto.json（默认参数）
  → GMR C++ IK
  → 支撑脚/腾空约束 + 实时速度/加速度/手臂跳变保护
  → BUMI3 MuJoCo Viewer
  → Redis（默认关闭，显式启用时按已验证 GMT 顺序发布）
```

## 默认配置来源

普通模式默认使用
`config/ik_configs/smplx_to_bumi3_auto.json`。该文件复制自：

```text
/home/weili/下载/general_motion_retargeting/ik_configs/smplx_to_bumi3_auto.json
```

`config/ik_configs/smplx_to_bumi3.json` 继续保留，供已有 Jump 配置和 G1 → BUMI3
生成流程引用；`smplx_to_bumi3_jump.json` 未随普通模式默认配置切换而改变。
普通模式不做耗时的在线搜索或未来帧优化；它只在启动首帧标定一次固定地面，运行时
使用当前帧和历史帧执行常数规模的接触判断与关节保护。

## BUMI3 模型

默认 XML：

```text
assets/bumi3/mjcf/bumi3.xml
```

该模型及 `assets/bumi3/meshes/` 来自仓库 BUMI3 资产提交。MuJoCo 实测可加载：

```text
nq=28  nv=27  nu=21  nbody=23  njnt=22
```

`base_link` 拥有 qpos 地址 0 的 `root` freejoint。配置引用的 12 个 robot body
全部真实存在且没有重名。

只读检查：

```bash
/home/weili/GENMO/.venv/bin/python scripts/inspect_bumi3_model.py

/home/weili/GENMO/.venv/bin/python scripts/validate_smplx_to_bumi3.py \
  --xml assets/bumi3/mjcf/bumi3.xml \
  --config config/ik_configs/smplx_to_bumi3_auto.json
```

## 12 项 IK target mapping

| SMP1 human target | BUMI3 robot body |
|---|---|
| `pelvis` | `base_link` |
| `spine3` | `waist_yaw_link` |
| `left_hip` | `l_leg_roll_link` |
| `left_knee` | `l_knee_pitch_link` |
| `left_foot` | `l_ankle_roll_link` |
| `right_hip` | `r_leg_roll_link` |
| `right_knee` | `r_knee_pitch_link` |
| `right_foot` | `r_ankle_roll_link` |
| `left_shoulder` | `l_arm_yaw_link` |
| `left_elbow` | `l_elbow_pitch_link` |
| `right_shoulder` | `r_arm_yaw_link` |
| `right_elbow` | `r_elbow_pitch_link` |

SMP1 Reader 仍接收固定 14 项人体 target；BUMI3 IK JSON 只消费 12 项。
`left_wrist` 和 `right_wrist` 有意不使用，不会为它们创建错误的 IK task。

## 编译和运行

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
./build.sh

./run_smplx_bumi3.sh \
  --always \
  --vis \
  --vis-smplx-targets \
  --vis-smplx-frames
```

默认监听 `0.0.0.0:7006`。启动脚本支持：

```text
BUMI3_XML
BUMI3_IK_CONFIG
BUMI3_UDP_PORT
BUMI3_REDIS_KEY
```

普通模式默认启用接触感知地面处理，并统一使用 `0.04 m` 初始 target clearance：

```text
mode=contact-aware
config=config/ik_configs/smplx_to_bumi3_auto.json
port=7006
offset_to_ground=off
ground_clearance=0.04
foot_contact=on（允许腾空）
realtime_safety=on
```

GENMO 发送端不增加新坐标转换，继续使用 SMP1：

```bash
cd /home/weili/GENMO
source .venv/bin/activate

python scripts/demo/demo_webcam.py \
  --camera_id 0 \
  --no_imgfeat \
  --display \
  --gmr_protocol smplx1 \
  --gmr_host 127.0.0.1 \
  --gmr_port 7006
```

## Viewer 颜色与诊断

- 蓝色：原始 SMP1 SMPL-X target。
- 黄色：GMR scale 和固定 offset 后的 IK target。
- 白色：BUMI3 映射 robot body。
- RGB 轴：`--vis-smplx-frames` 显示的 target frame 轴。

Viewer 的白色映射由 `smplx_bumi3_server` 显式传入，不再通过“存在
`base_link` 就当作 E1”的判断。只读诊断只查询 BUMI3 XML 中实际存在的 joint/body。

## Redis 关节顺序

XML 中的 21 个 MuJoCo qpos 已按 BUMI3 GMT 策略文件
`policy_gmt_mha_him_27000_0717a.onnx` 的 `joint_names` 元数据重排。重排只影响
Redis 帧中的 `joint_pos`，不会改变 MuJoCo qpos、Viewer 或 IK 配置。

- `config/robot_presets/bumi3.json` 记录 qpos、GMT publish order 和完整重排表；
- Redis key 与部署侧统一为 `gmt_online_frame_bumi`；
- `run_smplx_bumi3.sh` 仍默认添加 `--no-redis`，只打开 Viewer 不会发布；
- Gazebo/GMT 联调时显式传入 `--redis`。

显式启用 Redis：

```bash
./run_smplx_bumi3.sh --redis --always --vis
```

当前默认 key 为 `gmt_online_frame_bumi`。

## 接触感知与 Jump 模式

普通模式：

```bash
./run_smplx_bumi3.sh \
  --always \
  --vis \
  --vis-smplx-targets \
  --vis-smplx-frames
```

它使用 `--no-offset-to-ground --ground-clearance 0.04`。服务在启动后的第一帧把
最低 SMPL-X 脚 target 标定到 `z=0.04 m`，此后锁定同一个平移量，不再逐帧重新
移动整个人。因此后续双脚同时升高仍然会被识别为腾空，不会被最低脚对地逻辑消除。

接触检测同时观察左右脚相对首帧地面的高度、竖直速度、水平速度，并带进入/退出
滞回。稳定低脚成为支撑脚；两脚都离地时允许 `flight`；从一只支撑脚换到另一只时，
机器人脚底约束渐进切换。脚约束使用 BUMI3 XML 中真实脚底几何，`0.04 m` 只表示
人体 target 的初始离地参考，不表示机器人 mesh 悬空 4 cm。

如果显式关闭 `--no-foot-contact-constraints`，可以回到旧的无接触约束路径；旧的
`--offset-to-ground` 仅用于兼容。接触约束打开时程序会忽略逐帧最低脚对地，以免
破坏腾空判断。

同步 `smplx_bumi3_batch_server` 使用同一规则：默认从 IK JSON 启用接触检测，
`RESET` 时只标定一次源地面并初始化机器人脚底锚点，之后逐帧更新 stance/swing/flight。
即使调用端仍传入历史参数 `--offset-to-ground`，接触模式也会忽略逐帧对地；需要精确
复现旧 batch 输出时必须显式传入 `--no-foot-contact-constraints`。支撑期额外限制
脚尖/脚跟和内外侧高度差不超过 `3 mm`，这些约束不包含 yaw 行，因此不会锁住原动作
的水平转向。

batch 的人体源地面与机器人鞋底高度是两个独立参数：

- `--source-ground-clearance 0` 只用于首次人体源地面标定；
- `--ground-clearance 0` 不再人为抬高机器人，stance 的真实输出高度由 IK JSON 中
  `support_height_m=0.001`、`support_height_upper_m=0.003` 决定；
- 有可靠支撑时源地面仅以 `0.04 m/s` 慢跟踪，普通腾空冻结；长腾空持续 45 帧且脚
  运动重新稳定后，才以 `0.12 m/s` 渐进重捕获；
- 落脚当帧只运行一次人体 IK，然后捕获当前帧锚点，下一帧才进入渐进硬约束。禁止在
  每次 stance 进入时运行多轮 settle，否则会表现为根节点跳动和机器人卡顿；
- batch 热身结束后启用帧共享时序 QP：两张 IK 表及其所有内部迭代共同使用一份
  30 Hz 位移额度，腿、腰、free-root 和脚底接触在同一 QP 内求解；人形跟踪与时序/
  接触冲突时优先少跟动作，不能靠多次 IK 迭代累计跳到另一套解；
- batch 接触 QP 的关节上限为 `6 rad/s`（30 Hz 理论单帧 `11.46°`），这是低于
  `9/12 rad/s` 执行器极限的离线平滑上限；根节点水平速度 `0.75 m/s`、Z 速度
  `0.45 m/s`、角速度 `6 rad/s` 及线/角加速度 `3/20` 都在同一帧 QP 内生效；
- 接触切换若暂时不可行，允许关节立即刹停到零并保持当前过渡带，不强迫沿上一帧速度
  继续运动，也不做输出后整套 qpos 插值或根节点 Z 直接投影；
- batch 同时启用两帧手臂换解确认。QP 后保护只处理手臂，腿、腰和根节点不会在脚底
  可行解之后被再次改写。

旧 batch 生成的 qpos 不会自动改变，需要按新的 binary、IK SHA、source/target
clearance 身份重新导出。

## 实时关节保护

`realtime_safety` 配置默认打开。实时 UDP 服务在每次 IK 结束后执行完整 21 关节保护；
batch 则把腿、腰和根节点限制前移到接触 QP，只在 QP 后保留手臂分支确认：

- 速度上限按 Noetix `bumi.py` 的 `Bumi_CFG` 执行器配置逐关节设置：腰 yaw
  为 `9 rad/s`，其余双腿、膝、脚踝和双臂 20 个关节均为 `12 rad/s`；
- `bumi.py` 没有给出加速度上限，因此仍使用全身 `80 rad/s²`、手臂
  `60 rad/s²` 的实时保护值；
- 左右手臂任一关节单帧候选变化超过 `0.60 rad` 时，整条手臂先保持一帧；
- 下一帧仍然是相近的大跳目标才确认，并在速度/加速度限制下逐渐追赶；
- 只有一帧的 IK 换解或识别毛刺不会发送到 Redis。

保护器只扫描固定的 21 个关节，不重复求解 IK、不等待计时器，也不使用未来帧。
普通动作没有额外帧延迟；只有疑似手臂换解时增加一帧确认。

Jump 模式：

```bash
./run_smplx_bumi3_jump.sh \
  --always \
  --vis \
  --vis-smplx-targets \
  --vis-smplx-frames
```

```text
mode=jump
config=config/ik_configs/smplx_to_bumi3_jump.json
port=7007
offset_to_ground=off（脚本强制）
fixed_ground_offset=0.65（每帧减去同一个固定值）
ground_clearance=0.04（统一 preset 信息，不参与逐帧修正）
```

`0.65 m` 是根据当前实际摄像头画面人工调整后的固定值，相比上一版 `0.55 m`
将机器人整体再向下移动 `0.10 m`。
它不是首帧自动标定，也不会在运行中更新。不同摄像头位置或 GENMO 世界原点需要人工
覆盖环境变量：

```bash
BUMI3_JUMP_GROUND_OFFSET=0.67 ./run_smplx_bumi3_jump.sh \
  --always --vis --vis-smplx-targets --vis-smplx-frames
```

参数为正时会把全部人体 target 向下移动：机器人仍悬空就增大该值；脚进入地下就减小。
所有帧使用同一个值，因此不会消除后续跳跃产生的相对 Z 变化。

GENMO Jump 发送端：

```bash
cd /home/weili/GENMO
source .venv/bin/activate

CUDA_VISIBLE_DEVICES=0 python scripts/demo/demo_webcam.py \
  --camera_id 2 \
  --no_imgfeat \
  --display \
  --gmr_host 127.0.0.1 \
  --gmr_port 7007 \
  --gmr_protocol smplx1
```

普通模式现在也保留双脚真实腾空高度，同时用接触状态约束站立、行走和下蹲时的
支撑脚。Jump 脚本仍保留独立配置和人工固定 offset，便于兼容原有相机标定流程。

## Xsens Jump 规律与 BUMI3 权重来源

完整逐字段报告见 `docs/XSENS_JUMP_CONFIG_DIFF.md`，机器可读报告由
`scripts/compare_xsens_jump_configs.py` 生成到 `build/xsens_jump_diff.json`。

两套 Xsens 配置的共同规律是 table2 的 pelvis、左右 hip、knee、foot position
权重都提高；变化方向相同而比例不同时，按任务规则使用 E1 比例。当前 BUMI3
base → jump 的唯一变化为：

| BUMI3 target | table2 position base → jump | 依据 |
|---|---:|---|
| `base_link <- pelvis` | `100 → 1000` | E1 `10 → 100`，比例 `10.0` |
| `l/r_leg_roll_link <- hip` | `10 → 20` | E1 `10 → 20`，比例 `2.0` |
| `l/r_knee_pitch_link <- knee` | `30 → 120` | E1 `10 → 40`，比例 `4.0` |
| `l/r_ankle_roll_link <- foot` | `100 → 120` | E1 `100 → 120`，比例 `1.2` |

G1/E1 都修改了部分 Xsens scale，但本任务明确要求保留当前 SMPL-X/BUMI3 scale，
因此没有把 Xsens scale 数值或比例复制过来。仅 E1 修改的手臂/膝盖 rotation 权重
以及仅 G1 修改的 foot rotation 权重被标记为 robot-specific，没有应用到 BUMI3。
两套 Xsens 的 position/rotation offset 都没有变化；Jump 配置继续使用 BUMI3 自身
的 SMPL-X offset/quaternion，未复制任何 Xsens quaternion。

目前已完成 XML、配置、协议、固定姿态 IK、接触检测和实时关节保护的软件测试；仍需
用真实摄像头动作检查阈值，并在上实机前确认仿真配置中的 `9/12 rad/s` 是否也是
实物控制器允许的命令速度；`80/60 rad/s²` 仍需硬件侧单独确认。
