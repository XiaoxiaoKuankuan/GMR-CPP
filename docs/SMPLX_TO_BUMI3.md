# SMPL-X → BUMI3 固定实时重定向链路

## 数据链路

```text
GENMO SMPL-X FK
  → SMP1 UDP（固定14个人体 target）
  → SmplxReader（不变）
  → smplx_to_bumi3.json（固定参数）
  → 原 GMR C++ IK（不变）
  → BUMI3 MuJoCo Viewer
  → Redis（默认关闭，顺序尚未完成下游验证）
```

## 固定配置来源

运行配置 `config/ik_configs/smplx_to_bumi3.json` 是当前普通模式的受保护基线。
本次 ground/jump 任务开始时记录的 SHA-256 为：

```text
e2b313018bc133599e058673cfae363ef1f08e61fb375d544c4948145b4553e8
```

该文件包含用户已经确认的膝盖/脚权重，本任务没有修改它。Jump 配置完整继承其
scale、position offset、rotation quaternion 和 body mapping。
程序不执行动态调参、在线标定、自动搜索或运行时参数修正。

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
  --config config/ik_configs/smplx_to_bumi3.json
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

普通 grounded 模式固定启用逐帧最低脚对地，并使用 `0.02 m` clearance：

```text
mode=grounded
config=config/ik_configs/smplx_to_bumi3.json
port=7006
offset_to_ground=on
ground_clearance=0.02
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

## Redis 安全状态

XML 已确认 MuJoCo qpos 顺序和 actuator 顺序均为 21 个关节，但尚未获得并验证
下游 BUMI3 控制器期望的 publish order。因此：

- `config/robot_presets/bumi3.json` 记录真实 MuJoCo qpos/actuator 顺序；
- `joint_names_publish_order` 和 `joint_ids_map` 保持为空；
- identity qpos order 只允许 viewer/test 使用，不能声称可直接上实机；
- `run_smplx_bumi3.sh` 默认添加 `--no-redis`，不会连接 Redis。

只有在下游顺序完成验证后，才可显式启用测试 Redis：

```bash
./run_smplx_bumi3.sh --redis --always --vis
```

当前默认 key 为 `smplx_online_frame_bumi3`。显式启用不等于已完成实机协议验证。

## Grounded 与 Jump 模式

普通模式：

```bash
./run_smplx_bumi3.sh \
  --always \
  --vis \
  --vis-smplx-targets \
  --vis-smplx-frames
```

它使用 `--offset-to-ground --ground-clearance 0.02`。所有经过 scale/offset 的人体
target 统一平移，使最低的 `left_foot/right_foot` target 位于 `z=0.02 m`。这里的
`0.02 m` 是当前 BUMI3 的固定经验 clearance，不是摄像头动态地面识别，也不代表
机器人 mesh 脚底与物理地面的精确距离。用户仍可显式传入
`--no-offset-to-ground` 覆盖普通脚本默认值。

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
ground_clearance=0.02（仅保留统一 preset 信息，不参与逐帧修正）
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

普通模式会消除最低脚的真实腾空高度，适用于站立、行走和下蹲；Jump 模式保留
pelvis/双脚整体竖直平移，但也会重新暴露 GENMO 单目竖直估计或积分漂移。

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

目前已完成 XML、配置、协议、固定姿态 IK 和 viewer-only 软件验证；尚未完成真实摄像头
动作质量确认、BUMI3 下游关节发布顺序确认和实机安全验证。
