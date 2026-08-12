# OMG Unitree G1 → BUMI3

## 数据流

```text
OMG qpos36
  root xyz + root quaternion wxyz + G1 29 joints
        ↓ Redis JSON: omg_online_frame_g1
G1RedisReceiver → G1MotionReader
        ↓
G1MotionAdapter + G1 MuJoCo FK
        ↓ BodyMap：G1 body 世界 position/rotation
现有 GMR IK + 可选 gait-aware 重定向 + g1_to_bumi3.json
        ↓
BUMI3 qpos28
        ↓ Redis binary: gmt_online_frame_bumi
GMT / Gazebo / 实物 BUMI3
```

这是一条独立 source adapter 链路。新算法是显式 opt-in：只有运行
`g1_bumi3_server` 时传入 `--foot-contact-constraints`，才启用 gait-aware
重定向和轻量足底目标。不传该参数时，仍执行修改前的 full-pose IK、逐帧最低足点
ground-align 和 sample-and-hold 发布路径。原 SMPL-X/GEM → BUMI3 server、Reader、
配置、damping 和关节限位没有修改。

## 编译与配置生成

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
./build.sh
```

`g1_to_bumi3.json` 由模型和当前 BUMI3 权重参考配置确定性生成，不手写
scale/offset/rotation offset：

```bash
./build/generate_g1_to_bumi3_config \
  --source-xml assets/unitree_g1/g1_mocap_29dof.xml \
  --target-xml assets/bumi3/mjcf/bumi3.xml \
  --reference config/ik_configs/smplx_to_bumi3.json \
  --output config/ik_configs/g1_to_bumi3.json
```

仅在 G1/BUMI3 模型或参考权重发生变化时重新生成。

## 肘部语义坐标修正

最初版本只在两个机器人 `q=0` 时对齐 elbow body frame：

```cpp
rotation_offset = source_rest_rotation.conjugate() * target_rest_rotation;
```

这对膝、踝等 link frame 足够，但对前臂不成立：

- G1 的物理前臂纵轴是 elbow-local `+X`；
- BUMI3 的物理前臂纵轴是 elbow-local `-Z`；
- 两个 body frame 看似接近，实际前臂零位相差 `90°`；
- BUMI3 肘限位为 `[-2.26, 0] rad`，旧 target 会把常用的 G1 正肘角推向
  BUMI3 正方向，最后裁剪到 0。

生成器现在只对左右肘的 rotation offset 追加 target-local `Ry(-pi/2)`：

```cpp
rotation_offset *= Quaternion(AngleAxis(-pi / 2, UnitY));
```

对应的物理关节关系是：

```text
q_bumi_elbow = 1.0 * q_g1_elbow - pi/2
```

这不是简单取负号。配置中的 `semantic_joint_mapping` 显式记录 scale、offset 和
两边的前臂纵轴；table1/table2 使用同一个修正，其他 body 的 scale、position
offset、rotation offset 和权重保持不变。原 GEM/SMPL-X→BUMI3 配置
`smplx_to_bumi3.json` 没有修改。

默认 seed 单帧验证结果：

| 关节 | G1 | 旧 BUMI3 | 修正后 BUMI3 | 期望 `G1-90°` |
|---|---:|---:|---:|---:|
| 左肘 | +31.17° | 0.00°（限位） | -59.67° | -58.83° |
| 右肘 | +25.61° | 0.00°（限位） | -65.27° | -64.39° |

对 G1 肘角 `-22.9°、0°、28.6°、57.3°、85.9°、90°` 的连续扫描中，旧配置
的物理前臂方向误差最高约 `90°`；修正后最高约 `2.81°`。C++ 回归测试使用
五个独立姿态重新求解，最坏误差为 `2.68°`。

重新生成配置后应当与仓库文件完全一致：

```bash
./build/generate_g1_to_bumi3_config \
  --source-xml assets/unitree_g1/g1_mocap_29dof.xml \
  --target-xml assets/bumi3/mjcf/bumi3.xml \
  --reference config/ik_configs/smplx_to_bumi3.json \
  --output /tmp/g1_to_bumi3.json

diff -u config/ik_configs/g1_to_bumi3.json /tmp/g1_to_bumi3.json
./build/g1_to_bumi3_config_test
```

### “同样姿态”的边界

在两边都有对应自由度时，GMR 会匹配腿、肩和修正后的物理前臂方向。但两个模型
不是同构机器人：G1 有 waist roll/pitch 和双腕 3 DoF，BUMI3 没有这些关节。
因此不能数学上逐关节完全相同；BUMI3 只能在自身 21 DoF 和限位内得到最接近的
身体姿态。站立时应优先使用独立 neutral/实机验证 idle，而不是要求 BUMI3 用
浮动根去补偿 G1 的 waist pitch。gait-aware 模式因此把目标拆开处理：

- BUMI3 的实体 waist 关节只接收世界 yaw；G1 torso roll 丢弃；
- G1 torso pitch 作为单独的“前向躯干倾角”目标施加到浮动躯干，足部和腿部位置
  任务会让髋、膝、踝配合实现前屈，而不是要求不存在的 waist pitch 关节运动；
- BUMI3 `base_link` 只跟踪 G1 根的世界 XY 和 yaw；
- 另用 roll-upright 目标抑制 BUMI3 浮动根左右倾斜，但不锁住前后 pitch；
- 根 Z 不跟踪 G1 的绝对高度，由当前主支撑脚和 BUMI3 自身腿部 FK 决定。

这样既避免了 G1 torso roll 被错误变成 BUMI 浮动根左右晃，也保留了弯腰、鞠躬
等 torso pitch；根高度仍由支撑腿几何决定，不跟着源根高度逐帧颠动。

配置保留现有 GMR 所需的 `human_scale_table` 和两轮 `ik_match_table`，并增加：

```json
{
  "source": {"robot": "unitree_g1"},
  "target": {"robot": "bumi3"},
  "root": {
    "source": "pelvis",
    "target": "base_link",
    "ground_alignment": true,
    "height_offset": 0.0
  }
}
```

配置生成器还会从 G1 接触球和 BUMI3 足部 mesh 自动提取每只脚的足跟内外侧、
脚尖内外侧四个局部接触点，不使用踝关节原点或手工猜测的固定机器人高度。

启用 gait-aware 模式时，第一帧使用 BUMI3 `l_ankle_roll_link`、
`r_ankle_roll_link` 下真实接触 geom 完成 root Z 初始化。此后检测并保持一个
`primary support`：可靠单支撑优先；双支撑时保持上一主支撑脚，避免左右脚之间
逐帧跳变；只有支撑状态实际切换时才换脚。IK 得到关节姿态后，用主支撑脚的
BUMI3 FK 计算根 Z，使该脚足底落地。摆动脚不会参与根 Z 的目标计算，只有在它已
穿地时才做单向安全抬升。因此不再每帧按“左右脚全局最低点”重算根高度，也不再
把 root XY 投影回足底锚点。支撑脚防滑仍是带死区的低权重软目标。实时 server
另外提供
`--viewer-ground-penetration`（默认 `0.005 m`），只对 MuJoCo viewer 的 qpos
副本做 root-Z 视觉下沉。IK 结果及发布给 GMT 的根位姿仍是严格几何接触；该参数
与足底约束互不混用。设为 `0` 可关闭视觉下沉。

## Gait-aware 重定向与轻量足底目标

每个新 G1 qpos 帧经 FK 后，按左右脚四个真实足底点的高度、中心垂直速度、
中心水平速度和连续帧滞回判断 `stance/swing`。低空但快速横移的脚会被视为摆动脚，
避免转身和舞蹈时把两只脚都误锁。默认不允许飞行；两脚都未检测到可靠接触时，
较低一脚进入 `forced` grounding，但它只接受较弱的平足/贴地任务，不锁世界 XY。

支撑脚落地时在当前 BUMI3 世界位置捕获 XY 锚点。默认采用动作保真优先的
`soft-contact` 配置：

- 动态双支撑只选水平速度较低的一只主支撑脚；两脚都近似静止才双锚定；
- 足底 XY 使用带 `8 mm` 死区的软锚点，不默认使用硬防滑速度带；
- 足底法向和中心高度使用较低权重，只压制明显 roll/pitch，不锁 yaw；
- 新支撑脚使用 8 帧渐入，摆动脚立即释放；
- DAQP 默认只保留所有真实足部 mesh 点的硬防穿透下界。

离线任务若确实要求近似钉死足底，可在 JSON 中设
`hard_support_constraints=true` 恢复硬防滑/高度上界；该模式不建议用于当前实时
舞蹈和走路。接触模式下 DAQP 失败时保持当前配置，不会绕过硬防穿透。

除几何 IK 外，仅对受限的 hinge/slide 关节增加轻量时序连续项，free root 不参加：

```text
||q - q_ik||^2
+ 0.05 * ||q - q_prev||^2
+ 0.20 * ||q - (q_prev + dt_ratio * (q_prev - q_prevprev))||^2
```

第一项权重仍占绝对主导，所以这不是低通滤波，也不会继续增强足底锁定；速度项只
抑制单帧突变，加速度项轻量偏好延续上一帧速度。`dt_ratio` 使用输入时间戳，适配
30 Hz OMG 输入与 50 Hz GMT 发布。

当前 60 帧 OMG 走路样本的 A/B 结果：

| 指标 | 旧 GMR | 当前 gait-aware |
|---|---:|---:|
| root Z 范围 | 44.61 mm | 43.97 mm |
| root roll 范围 | 4.40° | 2.11° |
| root pitch 范围 | 2.34° | 2.54° |
| root Z 加速度 P95 | 4.06 m/s² | 2.75 m/s² |
| 关节步长 P95 | 0.0301 rad | 0.0305 rad |
| 平均关节加速度 | 2.52 rad/s² | 2.46 rad/s² |

当前模式不追求毫米级钉死足底，保留贴地、防穿透和有限防滑，同时优先保持源动作。
走路样本可靠支撑脚允许约 `15 mm` 软锚点偏移；这是为避免脚粘地和腰部代偿而
保留的有意自由度。上述数字是该 60 帧样本的离线运动学指标，不代表动力学稳定性。

接触模式将单关节速度、浮动根线速度和角速度分别限制为配置中的 `8 rad/s`、
`3 m/s`、`6 rad/s`，只拦截明显异常的单次 IK 步长。这些限制
同样只在足底约束模式生效。

这里约束的是发送给 GMT 的运动学参考轨迹，不是动力学平衡控制器：它不计算
接触力、摩擦锥、质心/ZMP 或电机力矩。实机上的抗滑、抗倾倒和关节安全仍必须由
GMT/WBC、状态估计与保护逻辑负责。默认 `allow_flight=false` 会在两脚都离地时保留
较低一脚作为低权重、不锁 XY 的 grounding 参考，适合当前“不要漂浮”的舞蹈目标，但仍会
压制真实跳跃的腾空段；需要保留跳跃时可在配置中改为 `true`。新支撑脚进入后的
前 8 帧是低权重渐入，不会突然把脚和腰拉向接触锚点。

实时 server 在新约束模式下默认对 viewer 和 GMT qpos 做一个 source-frame 的因果
插值：root xyz 和关节线性插值，root quaternion 使用最短路径 slerp。它消除
30/50 Hz 之间重复上一姿态造成的 sample-and-hold 卡顿，代价是约一个输入帧延迟。
`--no-reference-interpolation` 可关闭；旧兼容模式默认不启用。

## 离线测试

OMG 的 `qpos_36.npy` 可直接作为输入：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

./build/test_g1_to_bumi3 \
  --input /path/to/walk.npy \
  --config config/ik_configs/g1_to_bumi3.json \
  --fps 30 \
  --foot-contact-constraints \
  --foot-contact-weight-scale 0.25 \
  --output /tmp/bumi3_walk.npy
```

生成：

- `bumi3_walk.npy`：`(T,28)`，BUMI3 free root 7 + 21 joints。
- `bumi3_walk_root.npy`：`(T,7)` root trajectory。
- `bumi3_walk_feet.npy`：`(T,6)` 左右脚 body position。
- `bumi3_walk_replay.json`：MuJoCo replay 清单。

回放：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
/home/weili/OMG/.venv/bin/python scripts/replay_bumi3_motion.py \
  --replay /tmp/bumi3_walk_replay.json \
  --loop
```

## 实时运行：已有 qpos 文件按 30 Hz 发布

终端一启动 GMR：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
./run_g1_bumi3.sh --vis --vis-targets
```

终端二使用 OMG 环境发布：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
/home/weili/OMG/.venv/bin/python scripts/omg_g1_redis_stream.py \
  --input /home/weili/OMG/outputs/qpos_36.npy \
  --fps 30 \
  --loop \
  --redis-host 127.0.0.1 \
  --redis-port 6379 \
  --redis-db 0 \
  --redis-key omg_online_frame_g1
```

## 实时运行：OMG planner 文本/音乐生成

终端一启动 OMG planner。当前本机没有固定写死 denoiser 路径，使用实际导出的
ONNX：

```bash
cd /home/weili/OMG
source .venv/bin/activate

PYTHONPATH=src CUDA_VISIBLE_DEVICES=0 python -m omg.cli.realtime.planner_server \
  --bind tcp://0.0.0.0:5571 \
  --diffusion-onnx /path/to/actual_denoiser.onnx \
  --providers TensorrtExecutionProvider,CUDAExecutionProvider,CPUExecutionProvider \
  --dit-cache
```

终端二启动 OMG → Redis bridge。文本示例：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

PYTHONPATH=/home/weili/OMG/src \
/home/weili/OMG/.venv/bin/python scripts/omg_g1_redis_stream.py \
  --planner-connect tcp://127.0.0.1:5571 \
  --seed-motion /home/weili/OMG/inputs/seed_motion.npz \
  --condition-sequence "text: walk forward | text: turn left" \
  --fps 30 \
  --redis-key omg_online_frame_g1
```

音乐示例只替换 condition：

```bash
--condition-sequence "audio: /absolute/path/to/music.wav"
```

终端三启动 GMR → BUMI3：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
./run_g1_bumi3.sh \
  --redis-host 127.0.0.1 \
  --redis-port 6379 \
  --redis-db 0 \
  --input-redis-key omg_online_frame_g1 \
  --output-redis-key gmt_online_frame_bumi \
  --hz 50 \
  --ttl-ms 200 \
  --stale-ms 250 \
  --foot-contact-constraints \
  --foot-contact-weight-scale 0.25 \
  --reference-interpolation \
  --viewer-ground-penetration 0.005 \
  --vis
```

同一个 Redis 的两个 key 含义不同：

| Key | 方向 | 格式 |
|---|---|---|
| `omg_online_frame_g1` | OMG → GMR | JSON，G1 qpos36 |
| `gmt_online_frame_bumi` | GMR → GMT | 35×float32，BUMI3 GMT 关节顺序 |

## Redis 输入 JSON

```json
{
  "timestamp": 123.456,
  "root_pos": [0.0, 0.0, 0.793],
  "root_quat": [1.0, 0.0, 0.0, 0.0],
  "joints": ["29 finite numeric values"]
}
```

`root_quat` 是 `wxyz`。Receiver 会拒绝长度错误、NaN/Inf 和零范数四元数。

## 参数

`run_g1_bumi3.sh`：

- `--config`：G1 → BUMI3 IK 配置。
- `--redis-key` / `--input-redis-key`：OMG 输入 key。
- `--output-redis-key`：GMT BUMI3 输出 key。
- `--poll-hz`：输入 Redis GET 频率，默认 60 Hz，用于接收 30 Hz source。
- `--hz`：BUMI3 输出发布频率，默认 50 Hz。
- `--stale-ms`：超过该时间未收到新 OMG 帧就停止刷新输出。
- `--ttl-ms`：输出 key 自动过期时间。
- `--no-output-redis`：只运行 IK/viewer，不发送 GMT。
- `--vis`：BUMI3 MuJoCo viewer。
- `--vis-targets`：额外显示 G1 FK body targets。
- `--viewer-ground-penetration`：贴地后将 viewer qpos 副本额外下移的米数，
  默认 `0.005`，允许 `[0, 0.03]`；不改变 GMT 输出。
- `--ground-penetration`：以上参数的兼容别名。
- `--foot-contact-constraints`：显式启用 gait-aware 模式，包括 root XY/yaw 与 Z
  分离、waist 实体关节仅 yaw、torso pitch 经浮动躯干和髋腿实现、root roll
  upright、主支撑脚 FK 根 Z、轻量关节时序连续项，以及配置中的软防滑/平足/贴地
  和硬防穿透。默认不启用，以保持旧命令完全兼容。
- `--no-foot-contact-constraints`：关闭新约束，恢复此前的每帧最低几何点
  ground-align 和 full-pose IK；不传任何一个开关也走这条旧路径。
- `--foot-contact-weight-scale`：运行时缩放软防滑、平足和贴地权重，实时默认 `0.5`；
  若仍有腰部代偿可使用 `0.25`，设为 `0` 时只保留硬防穿地和最终 root-Z 贴地。
- `--reference-interpolation`：在相邻 GMR qpos 间插值后显示并发布给 GMT；
  gait-aware 模式默认启用，增加约一个 source frame 延迟。
- `--no-reference-interpolation`：关闭插值，恢复逐输入帧 sample-and-hold。

需要人工设置的只有运行环境参数：Redis 地址/key、OMG 实际 FPS、stale/TTL、
OMG planner ONNX 路径和提示词/音乐路径。配置中的 `height_offset` 保持模型生成值
`0.0`，实时视觉下沉由独立运行参数控制；
scale、position offset 和 rotation offset 不需要在线标定或动态搜索。

### 完整 A/B 命令

原始兼容模式（就是修改前的功能，不启用任何新算法）：

```bash
./run_g1_bumi3.sh \
  --redis-host 127.0.0.1 \
  --redis-port 6379 \
  --redis-db 0 \
  --input-redis-key omg_online_frame_g1 \
  --output-redis-key gmt_online_frame_bumi \
  --poll-hz 60 \
  --hz 50 \
  --ttl-ms 200 \
  --stale-ms 250 \
  --viewer-width 1280 \
  --viewer-height 720 \
  --vis \
  --vis-targets
```

推荐 gait-aware 模式：

```bash
./run_g1_bumi3.sh \
  --redis-host 127.0.0.1 \
  --redis-port 6379 \
  --redis-db 0 \
  --input-redis-key omg_online_frame_g1 \
  --output-redis-key gmt_online_frame_bumi \
  --poll-hz 60 \
  --hz 50 \
  --ttl-ms 200 \
  --stale-ms 250 \
  --foot-contact-constraints \
  --foot-contact-weight-scale 0.25 \
  --reference-interpolation \
  --viewer-ground-penetration 0.005 \
  --viewer-width 1280 \
  --viewer-height 720 \
  --vis \
  --vis-targets
```

## 实物 BUMI3

先完成离线和 MuJoCo/Gazebo 验证。实物阶段仍由现有 GMT 从
`gmt_online_frame_bumi` 读取，GMR 的 21 关节发布重排保持和 SMPL-X BUMI3
链路相同。切换实物不会改变 OMG input JSON、G1 FK adapter 或 IK 配置。
