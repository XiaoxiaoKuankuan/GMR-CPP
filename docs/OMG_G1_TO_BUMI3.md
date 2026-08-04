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
现有 GMR IK + g1_to_bumi3.json
        ↓
BUMI3 qpos28
        ↓ Redis binary: gmt_online_frame_bumi
GMT / Gazebo / 实物 BUMI3
```

这是一条独立 source adapter 链路。没有修改 `GMR::runIKStep`、`solveIK`、
DAQP、Jacobian、damping、关节限位，也没有修改原 SMPL-X → BUMI3 server、
Reader 或配置。

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
浮动根去补偿 G1 的 waist pitch。

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

贴地使用 BUMI3 `l_ankle_roll_link`、`r_ankle_roll_link` 下真实接触 geom 的
最低点计算 root Z 修正，不使用固定机器人高度。实时 G1→BUMI3 server 另外提供
`--viewer-ground-penetration`（默认 `0.005 m`），只对 MuJoCo viewer 的 qpos
副本做 root-Z 视觉下沉。IK 结果及发布给 GMT 的根位姿仍是严格几何接触；该参数
也不能替代足底平整/接触约束。设为 `0` 可关闭视觉下沉。

## 离线测试

OMG 的 `qpos_36.npy` 可直接作为输入：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

./build/test_g1_to_bumi3 \
  --input /path/to/walk.npy \
  --config config/ik_configs/g1_to_bumi3.json \
  --fps 30 \
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
  默认 `0.005`，允许 `[0, 0.03]`；不改变 GMT 输出，不能修复脚掌倾斜。
- `--ground-penetration`：以上参数的兼容别名。

需要人工设置的只有运行环境参数：Redis 地址/key、OMG 实际 FPS、stale/TTL、
OMG planner ONNX 路径和提示词/音乐路径。配置中的 `height_offset` 保持模型生成值
`0.0`，实时视觉下沉由独立运行参数控制；
scale、position offset 和 rotation offset 不需要在线标定或动态搜索。

## 实物 BUMI3

先完成离线和 MuJoCo/Gazebo 验证。实物阶段仍由现有 GMT 从
`gmt_online_frame_bumi` 读取，GMR 的 21 关节发布重排保持和 SMPL-X BUMI3
链路相同。切换实物不会改变 OMG input JSON、G1 FK adapter 或 IK 配置。
