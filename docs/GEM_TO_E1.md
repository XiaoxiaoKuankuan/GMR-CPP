# GEM 实时重定向到 Unitree E1

本文说明本机两个项目之间的实时链路：

```text
摄像头或视频
  -> GENMO/GEM-SMPL 推理
  -> EnDecoder.fk_v2 的 22 个 SMPL-X 关节世界位姿
  -> Y-Up 转 Z-Up、初始 yaw/水平原点/脚底高度归零
  -> UDP GEM1（14 个重定向骨骼，固定 412 字节）
  -> GMR-CPP GemReader
  -> MotionBuffer / GMR IK
  -> Unitree E1 24DOF MuJoCo 模型
  -> MuJoCo viewer（可选）和 Redis
```

这条路径只支持 E1。默认配置为：

- MuJoCo XML：`assets/e1/mjcf/e1_24dof.xml`
- IK：`config/ik_configs/xsens_to_e1.json`
- preset：`e1`，24 个关节
- Redis key：`gmt_online_frame_e1`
- Redis value：38 个 little-endian float32，共 152 字节
- GEM UDP：`0.0.0.0:7001`
- Redis 发布频率：50 Hz
- 输入 stale：250 ms
- Redis TTL：200 ms

## 修改文件

GENMO：

- `gem/gmr_udp_bridge.py`：GEM1 打包、坐标变换和初始原点归一化
- `scripts/demo/demo_webcam.py`：复用 `EnDecoder.fk_v2`、UDP 发送和主线程显示
- `tests/test_gmr_udp_bridge.py`：无模型权重的协议/坐标测试

GMR-CPP：

- `readers/gem_reader.hpp`、`readers/gem_reader.cpp`：GEM1 UDP Reader
- `apps/gem_server.cpp`：E1 IK、viewer、Redis 和 stale 安全逻辑
- `run_gem.sh`：E1 专用启动入口
- `scripts/send_gem_test_packet.py`：合成 T-pose UDP 发送器
- `CMakeLists.txt`、`build.sh`：构建 `gem_mocap_server`

## GEM1 UDP 协议

全部字段采用 little-endian。Header 为：

| 偏移 | 类型 | 内容 |
|---:|---|---|
| 0 | 4 bytes | magic `GEM1` |
| 4 | uint16 | version，固定 1 |
| 6 | uint16 | bone count，固定 14 |
| 8 | uint32 | sequence，允许 uint32 wrap-around |
| 12 | uint64 | 发送端 `time.monotonic_ns()` |

随后是 14 组 `(px, py, pz, qw, qx, qy, qz)` float32，共：

```text
20 + 14 * 7 * 4 = 412 bytes
```

固定骨骼顺序为：

```text
Pelvis, Chest,
Left_UpperLeg, Right_UpperLeg,
Left_LowerLeg, Right_LowerLeg,
Left_Foot, Right_Foot,
Left_UpperArm, Right_UpperArm,
Left_Forearm, Right_Forearm,
Left_Hand, Right_Hand
```

GemReader 不使用发送端时间驱动 MotionBuffer，而是在本机收到有效 UDP 包时记录
`steady_clock` 纳秒时间，避免两台机器的时钟基准不同。

## 编译

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
bash build.sh --clean
test -x build/gem_mocap_server
```

## 无摄像头 synthetic test

终端一启动 GMR-CPP（测试时必须使用 `--always` 绕过手柄门控）：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
./run_gem.sh --always
```

终端二发送 3 秒合成 T-pose：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
python3 scripts/send_gem_test_packet.py --seconds 3
```

发送过程中检查 E1 value：

```bash
redis-cli STRLEN gmt_online_frame_e1
# 期望：152
```

发送停止一秒后检查 stale/TTL：

```bash
sleep 1
redis-cli EXISTS gmt_online_frame_e1
# 期望：0
```

该发送器只验证 UDP、IK 和 Redis 链路，不代表真实动作重定向质量。

## 真实 webcam 运行

先启动 GMR-CPP 和 E1 MuJoCo 窗口：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
./run_gem.sh --always --vis
```

再启动 GENMO：

```bash
cd /home/weili/GENMO
source .venv/bin/activate
python scripts/demo/demo_webcam.py \
  --camera_id 0 \
  --display \
  --gmr_host 127.0.0.1 \
  --gmr_port 7001
```

如果 GENMO 与 GMR-CPP 在不同机器，将 `--gmr_host` 改成运行 GMR-CPP
机器可达的 IP，并确保 UDP 7001 未被防火墙拦截。GMR-CPP 可以用
`--bind <网卡IP>` 限制监听网卡。

`--display` 使用主线程打开 `GEM Live` OpenCV 窗口，从第一帧显示视频、bbox、
track id、warmup/ready、FPS 和 UDP sequence。按 `q` 退出。默认 GEM 上下文为
120 帧，因此出现人体后要先完成 `1/120 ... 120/120` warmup，之后才开始发送 UDP。

`--display` 需要有效 `DISPLAY`/桌面会话；GMR 的 `--vis` 同样需要 OpenGL 和桌面。
无桌面服务器上去掉这两个参数。原有 mesh renderer 仍可独立使用：

```bash
python scripts/demo/demo_webcam.py \
  --camera_id 0 --display --render --render_mode opencv \
  --gmr_host 127.0.0.1
```

## 视频文件运行

```bash
cd /home/weili/GENMO
source .venv/bin/activate
python scripts/demo/demo_webcam.py \
  --video /absolute/path/to/video.mp4 \
  --display \
  --gmr_host 127.0.0.1 \
  --gmr_port 7001
```

## yaw、比例和落地调试

发送端会在第一个有效 FK 帧完成三件事：

1. 抵消 pelvis 初始 Z 轴 yaw；
2. 把 pelvis 初始 XY 设为零；
3. 把双脚较低的初始 Z 设为地面零点。

它不会逐帧贴地，因此会保留跳跃高度。人物朝向仍需人工修正时使用：

```bash
python scripts/demo/demo_webcam.py \
  --camera_id 0 --display --gmr_host 127.0.0.1 \
  --gmr_yaw_deg 90
```

位置比例可通过 `--gmr_scale 1.0` 调整。GMR server 默认关闭二次
offset-to-ground；只有明确需要逐帧贴地时才使用：

```bash
./run_gem.sh --always --vis --offset-to-ground
```

## 常见故障

- `gem_mocap_server not found`：运行 `bash build.sh --clean`。
- `E1 XML/IK config not found`：不要从其他目录复制 G1 配置，检查本文顶部两个 E1 路径。
- 一直停在 `collecting 10 seed frames`：GENMO 尚未完成 120 帧 warmup、目标 IP/端口错误，或 UDP 7001 被防火墙拦截。
- `invalid` 增长：发送包不是 412 字节、magic/version/bone count 不匹配、包含 NaN/Inf、四元数退化或位置超过 20 m。
- Redis 没有 key：确认 `redis-cli ping` 返回 `PONG`，测试时使用 `--always`，并确认 GEM 输入没有超过 250 ms 未更新。
- Redis 长度不是 152：启动的不是 E1 GEM server，或使用了错误 key；检查启动日志中的 `joints=24 frame=38 floats / 152 bytes`。
- GUI 无法启动：检查 `echo "$DISPLAY"`、OpenCV GUI 和 OpenGL 驱动；无 GUI 环境不要传 `--display`/`--vis`。
- 朝向不正确：先保持首帧站直，再尝试 `--gmr_yaw_deg 90`、`-90` 或 `180`。
