# GMR-CPP

Python GMR（General Motion Retargeting）流水线的 C++ 移植版。将多种动捕设备的人体骨骼数据实时重定向到机器人关节，并以固定二进制格式写入 Redis，供下游运动控制模块消费。

```
动捕设备 → Reader → GMR IK（daqp + MuJoCo）→ Redis（43×float32）
```

**推荐用于生产部署**：单帧 IK 约 0.13 ms，Redis 发布 50 Hz，支持 warm start 与启动 re-warm。

---

## 支持的动捕后端

| 可执行文件 | 动捕系统 | 启动脚本 | 额外依赖 |
|---|---|---|---|
| `optitrack_mocap_server` | OptiTrack / Motive（NatNet） | `run.sh` / `run_e1.sh` | NatNet SDK（已内置） |
| `xsens_mocap_server` | Xsens MVN（UDP） | `run_xsens.sh` / `run_xsens_e1.sh` | 无 |
| `pico_mocap_server` | Pico / XRobot | `run_pico.sh` | `third_party/pico_sdk` |
| `fzmotion_mocap_server` | FZMotion | `run_fzmotion_g1.sh` / `run_fzmotion_e1.sh` | `third_party/LuMoSDK` |

支持的机器人模型：

| 模型 | MuJoCo XML | 典型 IK 配置 |
|---|---|---|
| Unitree G1（29 DOF） | `assets/unitree_g1/g1_mocap_29dof.xml` | `config/ik_configs/fbx_to_g1.json` |
| E1（23/24 DOF） | `assets/e1/mjcf/e1_23dof.xml` / `e1_24dof.xml` | `config/ik_configs/fbx_to_e1.json` |

---

## 依赖

### 系统包

```bash
sudo apt install \
    libeigen3-dev \
    libglfw3-dev \
    libhiredis-dev \
    nlohmann-json3-dev \
    build-essential \
    cmake
```

### 仓库内置第三方库

| 依赖 | 路径 | 说明 |
|---|---|---|
| MuJoCo 3.2.3 | `third_party/mujoco/` | 预编译，Linux x86_64 |
| NatNet SDK 4.4 | `third_party/NatNet_SDK_4.4_ubuntu/` | 预编译 |
| daqp | `third_party/daqp/` | QP 求解器，源码编译 |
| Pico SDK（可选） | `third_party/pico_sdk/` | XRoboToolkit-PC-Service-Pybind |
| LuMo SDK（可选） | `third_party/LuMoSDK/` | FZMotion 专用 |

---

## 编译

```bash
cd GMR-CPP
./build.sh
```

| 参数 | 说明 |
|---|---|
| `--clean` | 清理 build 目录后重建 |
| `--debug` | Debug 模式编译 |

编译步骤：

1. 检查系统依赖
2. 编译 `third_party/daqp` → `libdaqpstat.a`
3. CMake 构建各 mocap server 可执行文件

产物位于 `build/`。Pico / FZMotion server 仅在对应 SDK 存在时才会编译。

---

## 运行

### OptiTrack（最常用）

```bash
./run.sh \
    --server <motive_pc_ip> \
    --client <本机_ip> \
    --always \
    --vis
```

### Xsens

```bash
./run_xsens.sh --always --vis
# 默认 UDP 端口 9763，可用 --port 修改
```

### Pico

```bash
# 需先将 XRoboToolkit SDK 复制到 third_party/pico_sdk
./run_pico.sh --height=1.6 --no-vis
```

### FZMotion

```bash
./run_fzmotion_g1.sh --always
./run_fzmotion_e1.sh --always
```

### 直接调用可执行文件

```bash
cd build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../third_party/mujoco/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../third_party/NatNet_SDK_4.4_ubuntu/lib

./optitrack_mocap_server \
    --server <motive_ip> \
    --client <本机_ip> \
    --xml ../assets/unitree_g1/g1_mocap_29dof.xml \
    --ik-config ../config/ik_configs/fbx_to_g1.json \
    --always --vis
```

---

## CLI 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--server` | 必填（OptiTrack） | Motive 主机 IP |
| `--client` | 必填（OptiTrack） | 本机 IP（NatNet Client） |
| `--port` | 9763（Xsens） | Xsens MVN UDP 端口 |
| `--xml` | 见各 run 脚本 | MuJoCo 机器人模型路径 |
| `--ik-config` | 见各 run 脚本 | IK 配置 JSON |
| `--redis-host` | 127.0.0.1 | Redis 地址 |
| `--redis-port` | 6379 | Redis 端口 |
| `--redis-key` | mmocap_motion_frame_g1 | Redis 键名 |
| `--hz` | 50 | 发布频率（Hz） |
| `--ttl-ms` | 200 | Redis 键 TTL（ms） |
| `--lin-vel-alpha` | 1 | 线速度 EMA 系数（1=不过滤） |
| `--ang-vel-alpha` | 1 | 角速度 EMA 系数 |
| `--lin-vel-max` | 0 | 线速度尖峰拒绝阈值（0=关闭） |
| `--ang-vel-max` | 0 | 角速度尖峰拒绝阈值 |
| `--always` | 关闭 | 跳过手柄门控，持续发布 |
| `--vis` | 关闭 | 打开 MuJoCo 可视化 |
| `--no-offset-to-ground` | 关闭 | 禁用地面对齐偏移（Xsens） |

Pico 专用（`run_pico.sh`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--height` | 1.6 | 操作者实际身高（m） |
| `--gmr-hz` | 30 | GMR 重定向频率 |
| `--buffer-ms` | 200 | 缓冲延迟（ms，自适应） |

---

## Redis 输出格式

**键名**：`mmocap_motion_frame_g1`（可通过 `--redis-key` 修改）  
**格式**：43 × float32 = **172 字节**

| 索引 | 字段 | 数量 |
|---|---|---|
| 0 | 时间戳（相对，秒） | 1 |
| 1–3 | 骨盆位置 xyz（m，世界系） | 3 |
| 4–7 | 骨盆四元数 wxyz | 4 |
| 8–10 | 线速度 xyz（m/s，世界系） | 3 |
| 11–13 | 角速度 xyz（rad/s，世界系） | 3 |
| 14–42 | 关节角（rad），29 DOF | 29 |

**原始骨骼键**：`mmocap_motion_frame_g1_raw_bones`  
JSON 字符串，包含 OptiTrack 原始骨骼位置与朝向，用于人体标定。

---

## 人体标定

不同操作者体型差异较大，建议每人标定一次 IK 配置。

### 自动标定（推荐）

前提：mocap server 已以 `--always` 模式运行，操作者保持 **T-pose** 5 秒。

```bash
python3 scripts/auto_calibrate.py \
    --output-dir config/ik_configs/calibrated/ \
    --person-name alice \
    --duration 5
```

标定完成后，启动时指定生成的配置：

```bash
./run.sh --server <ip> --client <ip> \
    --ik-config config/ik_configs/calibrated/fbx_to_g1_alice_<timestamp>.json \
    --always
```

### 辅助脚本

```bash
# 录制 Redis 根节点位置，用于调试
python3 scripts/record_redis_root_pos.py
```

---

## 处理流水线

```
动捕回调（~120 Hz）
    └→ frame_queue
         └→ asyncLoop: GMR IK 求解（~120 Hz，~0.13 ms/帧）
              └→ MotionBuffer（最新帧）
                   └→ publishLoop（50 Hz）→ Redis
```

- **IK 求解器**：daqp QP，带 ConfigurationLimit（关节速度约束）
- **Warm start**：IK 状态跨帧保持，加速收敛
- **Re-warm**：启动时用 1000 帧真实数据初始化 IK 状态

---

## 目录结构

```
GMR-CPP/
├── apps/                       # 各动捕 server 入口
│   ├── mocap_server.cpp        # OptiTrack
│   ├── xsens_server.cpp
│   ├── pico_mocap_server.cpp
│   └── fzmotion_server.cpp
├── assets/
│   ├── unitree_g1/             # G1 MuJoCo 模型
│   └── e1/                     # E1 机器人模型
├── build/                      # 编译产物
├── build.sh                    # 一键编译脚本
├── config/ik_configs/          # IK 配置文件
│   ├── fbx_to_g1.json          # OptiTrack → G1（默认，1.8m 参考身高）
│   ├── xsens_to_g1.json        # Xsens → G1
│   ├── xrobot_to_g1.json       # Pico → G1
│   ├── fbx_to_e1.json          # OptiTrack → E1
│   └── calibrated/             # 按人标定后的配置
├── include/gmr/                # 核心库头文件
│   ├── gmr_mink.hpp            # IK 重定向主实现
│   ├── body_map.hpp            # 人体-机器人骨骼映射
│   ├── redis_publisher.hpp     # Redis 发布
│   ├── motion_buffer.hpp       # 帧缓冲
│   └── mujoco_viewer.hpp       # MuJoCo 可视化
├── readers/                    # 各动捕后端 Reader
├── run*.sh                     # 各场景启动脚本
├── scripts/
│   ├── auto_calibrate.py       # 自动人体标定
│   └── record_redis_root_pos.py
└── third_party/
    ├── daqp/
    ├── mujoco/
    ├── NatNet_SDK_4.4_ubuntu/
    ├── pico_sdk/               # 可选
    └── LuMoSDK/                # 可选
```

---

## Motive 配置（OptiTrack）

1. 骨骼命名规范设为 **FBX**
2. 开启 Skeleton Streaming
3. Motive 中 Server / Client IP 与 `--server` / `--client` 一致

---

## 跨机器 Redis 访问

```bash
# 在 mocap server 所在机器
sudo sed -i 's/^bind 127.0.0.1/bind 0.0.0.0/' /etc/redis/redis.conf
sudo sed -i 's/^protected-mode yes/protected-mode no/' /etc/redis/redis.conf
sudo systemctl restart redis

# 在客户端机器验证
redis-cli -h <mocap_server_ip> -p 6379 get mmocap_motion_frame_g1
```

---

## 与 Python 版对比

| 特性 | GMR-CPP | Online_Mocap_Retarget |
|---|---|---|
| 语言 | C++ | Python |
| 性能 | ~0.13 ms/帧 | 较慢 |
| 多后端 | OptiTrack / Xsens / Pico / FZMotion | 主要 OptiTrack |
| 推荐场景 | 生产部署 | 原型验证、算法调试 |

详见 [`../Online_Mocap_Retarget/README.md`](../Online_Mocap_Retarget/README.md)。
