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

运行配置 `config/ik_configs/smplx_to_bumi3.json` 与仓库根目录的
`smplx_to_bumi3.json` 字节完全相同。其固定 SHA-256 为：

```text
a5b810e0e12519f4c94fc595622bb2ff5a9ed668a419236aa9ac50eacb678f88
```

配置中的 scale、position offset、rotation quaternion 和两阶段权重均原样保留。
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

## 地面处理与限制

启动脚本固定使用 `--no-offset-to-ground`，保留发送端 pelvis/foot 高度以及蹲下、
抬腿或跳跃的垂直运动。`--offset-to-ground` 会逐帧把最低脚贴地，可能抹除真实高度，
仅应作为人工诊断选项。

目前已完成 XML、配置、协议、固定姿态 IK 和 viewer-only 软件验证；尚未完成真实摄像头
动作质量确认、BUMI3 下游关节发布顺序确认和实机安全验证。
