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
最低点计算 root Z 修正，不使用固定机器人高度。

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

需要人工设置的只有运行环境参数：Redis 地址/key、OMG 实际 FPS、stale/TTL、
OMG planner ONNX 路径和提示词/音乐路径。`height_offset` 当前保持模型生成值 `0.0`；
scale、position offset 和 rotation offset 不需要在线标定或动态搜索。

## 实物 BUMI3

先完成离线和 MuJoCo/Gazebo 验证。实物阶段仍由现有 GMT 从
`gmt_online_frame_bumi` 读取，GMR 的 21 关节发布重排保持和 SMPL-X BUMI3
链路相同。切换实物不会改变 OMG input JSON、G1 FK adapter 或 IK 配置。
