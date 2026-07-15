# Codex 实现任务：新增独立 SMPL → E1 直连链路，并提供可选 SMPL → Xsens 标定；不修改 GMR IK 求解器

## 0. 本地仓库与禁止事项

本地目录：

```text
/home/weili/GENMO
/home/weili/GMR-CPP_e1jump_lowdpi
```

开始前执行并保存：

```bash
git -C /home/weili/GENMO status --short
git -C /home/weili/GENMO rev-parse HEAD

git -C /home/weili/GMR-CPP_e1jump_lowdpi status --short
git -C /home/weili/GMR-CPP_e1jump_lowdpi rev-parse HEAD
```

禁止：

```text
git reset --hard
git checkout .
git clean
```

保留所有现有无关改动和旧链路。

绝对禁止修改 GMR IK 求解核心：

```text
include/gmr/gmr_mink.hpp 中的
runIKStep
solveIK
DAQP
Jacobian
damping
joint limits
迭代次数与停止条件
```

不要覆盖：

```text
readers/xsens_reader.*
config/ik_configs/xsens_to_e1.json
原 Xsens / OptiTrack / Pico / FZMotion 启动路径
```

目标机器人始终为：

```text
Unitree E1 24DOF
assets/e1/mjcf/e1_24dof.xml
preset=e1
Redis key=gmt_online_frame_e1
```

---

# 1. 先在文档中准确整理 XsensReader 的真实语义

新增：

```text
docs/XSENS_AND_SMPL_FRAMES.md
```

必须明确写出：

## 1.1 Xsens Reader 已知信息

`readers/xsens_reader.cpp` 提供：

```text
MXTP02 Type-02 quaternion pose packet
24-byte header
每 segment 32 bytes：
    int32 segment_id
    float32 x y z
    float32 qw qx qy qz

网络字节序：big-endian
输出 quaternion：wxyz
输出世界坐标：Z-up、right-handed
```

segment ID：

```text
1  Pelvis
2  Spine       L5
3  Spine1      L3
4  Spine2      T12
5  Chest       T8
6  Neck
7  Head
8  Right_Shoulder
9  Right_UpperArm
10 Right_Forearm
11 Right_Hand
12 Left_Shoulder
13 Left_UpperArm
14 Left_Forearm
15 Left_Hand
16 Right_UpperLeg
17 Right_LowerLeg
18 Right_Foot
19 Right_Toe
20 Left_UpperLeg
21 Left_LowerLeg
22 Left_Foot
23 Left_Toe
```

首帧 yaw normalization：

```text
从 Pelvis global quaternion 得到 R
yaw = atan2(R[1,0], R[0,0])
Q_yaw_inv = RotZ(-yaw)

每个 segment：
p' = Q_yaw_inv * p
R' = Q_yaw_inv * R
```

这意味着代码把 pelvis local +X 在水平面的投影当作 heading。

## 1.2 Xsens Reader 没有提供的信息

必须明确：

```text
xsens_reader.cpp 没有定义每个 segment 的 local x/y/z anatomical axes
没有从 joint positions 构造 segment frame
没有 segment origin alpha 表
```

Reader 只是接收 MVN 已经计算好的 global segment position/quaternion。

因此不能仅凭 `xsens_reader.cpp` 推导出“精确的 proprietary Xsens local frame”。

`xsens_to_e1.json` 中的固定 rotation offsets 是：

```text
Xsens global segment frame → E1 robot body frame
```

不是 SMPL joint frame → Xsens frame 的定义。

## 1.3 单位注释不一致

代码注释写 cm→m，但 `CM_TO_M=1.0`。本任务不修改此行为，因为现有 Xsens 链路已验证可用；只在文档中标记需要结合实际 MVN 输出确认。

---

# 2. 架构决定：最终使用独立 SMPL → E1，不伪装成 Xsens

当前已有的 `GEM1` / `gem_segments_to_e1` 保留为 legacy/debug。

新增独立正式链路：

```text
GEM SMPL-X 22 joints
→ SMPLDirectAdapter
→ GEM2 UDP
→ GemReader GEM2 name table
→ smpl_direct_to_e1_calibrated.json
→ 原 GMR IK
→ E1
```

核心原则：

1. 不把 shoulder joint 假装命名成 Xsens `Left_UpperArm`。
2. 不复用 `xsens_to_e1.json` 的 offsets。
3. 使用明确的 SMPL landmark 名称。
4. position 是 SMPL joint center。
5. rotation 是由关节几何构造的 anatomical segment frame。
6. 用专用联合标定计算 SMPL landmark/frame → E1 body 的 position/rotation offset。
7. GMR IK solver 完全不改。

---

# 3. 新增 GEM2 协议，避免 GEM1 语义混淆

仍保持固定 412 bytes：

```text
Header:
magic             4 bytes = b"GEM2"
version           uint16 = 2
item_count        uint16 = 14
sequence          uint32
source_stamp_ns   uint64

Payload:
14 × (px, py, pz, qw, qx, qy, qz) float32 little-endian
```

新增固定顺序：

```text
0  SMPL_Pelvis
1  SMPL_Chest

2  SMPL_LeftHip
3  SMPL_RightHip

4  SMPL_LeftKnee
5  SMPL_RightKnee

6  SMPL_LeftAnkle
7  SMPL_RightAnkle

8  SMPL_LeftShoulder
9  SMPL_RightShoulder

10 SMPL_LeftElbow
11 SMPL_RightElbow

12 SMPL_LeftWrist
13 SMPL_RightWrist
```

## GENMO

修改或新增：

```text
gem/smpl_direct_adapter.py
gem/gmr_udp_bridge.py
```

增加：

```python
send_smpl_targets(targets)
```

发送 `GEM2`。

旧：

```python
send_segments()
send_fk()
```

保持兼容。

## GMR-CPP

修改：

```text
readers/gem_reader.cpp
readers/gem_reader.hpp
```

同时识别：

```text
GEM1 version 1 → 旧 canonical Xsens-style names
GEM2 version 2 → 新 SMPL_* names
```

包长都必须是 412。

启动日志打印当前收到的 protocol。

---

# 4. 新增 SMPLDirectAdapter

新增：

```text
/home/weili/GENMO/gem/smpl_direct_adapter.py
/home/weili/GENMO/config/gmr/smpl_direct_e1_adapter.json
```

输入：

```text
joints_ay:     (22, 3)
joint_rots_ay: (22, 3, 3)
timestamp
```

输出 14 个：

```python
TargetPose(
    position_zup: np.ndarray shape (3,),
    rotation_zup: np.ndarray shape (3,3),
)
```

---

# 5. SMPL 22-joint 索引与 14 target 映射

必须硬性测试：

```text
SMPL_Pelvis        = joint 0 pelvis
SMPL_Chest         = joint 9 spine3

SMPL_LeftHip       = joint 1 left_hip
SMPL_RightHip      = joint 2 right_hip

SMPL_LeftKnee      = joint 4 left_knee
SMPL_RightKnee     = joint 5 right_knee

SMPL_LeftAnkle     = joint 7 left_ankle
SMPL_RightAnkle    = joint 8 right_ankle

SMPL_LeftShoulder  = joint 16 left_shoulder
SMPL_RightShoulder = joint 17 right_shoulder

SMPL_LeftElbow     = joint 18 left_elbow
SMPL_RightElbow    = joint 19 right_elbow

SMPL_LeftWrist     = joint 20 left_wrist
SMPL_RightWrist    = joint 21 right_wrist
```

辅助 joints：

```text
joint 10 left_foot
joint 11 right_foot
joint 12 neck
```

所有 target position 必须严格等于对应 joint center，不能使用 midpoint。

---

# 6. 坐标系与 heading

AY/Y-up → GMR Z-up：

```python
C = np.array([
    [1, 0,  0],
    [0, 0, -1],
    [0, 1,  0],
])
```

位置：

```python
p_zup = C @ p_ay
```

joint rotation fallback：

```python
R_zup = C @ R_ay @ C.T
```

heading 默认从关节几何计算：

```python
left_axis_raw =
    (left_hip - right_hip)
    + (left_shoulder - right_shoulder)

up_axis = normalize(chest - pelvis)

left_axis =
    normalize(
        left_axis_raw
        - up_axis * dot(left_axis_raw, up_axis)
    )

forward_axis = normalize(left_axis × up_axis)
```

目标世界约定：

```text
+X = human forward / E1 forward
+Y = human left
+Z = up
```

首个有效帧计算 yaw，使 `forward_axis` 对齐 +X；之后固定该 world heading offset。

提供：

```text
--smpl_heading_source joints|pelvis
--smpl_forward_sign +1|-1
--smpl_yaw_deg
```

但默认必须由 synthetic tests 确定。

---

# 7. anatomical frame 的明确公式

所有 frame 最终列向量为：

```text
R = [x_axis, y_axis, z_axis]
```

满足：

```text
x × y = z
R.T R = I
det(R) = +1
```

## 7.1 Pelvis

位置：

```text
joint 0
```

轴：

```python
z = normalize(chest - pelvis)

y0 = normalize(left_hip - right_hip)
y = normalize(y0 - z * dot(y0, z))

x = normalize(y × z)
y = normalize(z × x)
R = [x, y, z]
```

## 7.2 Chest

位置：

```text
joint 9
```

轴：

```python
z = normalize(neck - pelvis)

y0 = normalize(left_shoulder - right_shoulder)
y = normalize(y0 - z * dot(y0, z))

x = normalize(y × z)
y = normalize(z × x)
R = [x, y, z]
```

## 7.3 Long-axis limb frame

用于：

```text
UpperLeg
LowerLeg
UpperArm
Forearm
```

定义：

```python
z = normalize(distal - proximal)

x0 = pelvis_or_chest_forward
x = normalize(x0 - z * dot(x0, z))

y = normalize(z × x)
x = normalize(y × z)

R = [x, y, z]
```

具体：

```text
SMPL_LeftHip:
    proximal left_hip
    distal left_knee
    reference pelvis forward

SMPL_LeftKnee:
    proximal left_knee
    distal left_ankle
    reference pelvis forward

SMPL_LeftShoulder:
    proximal left_shoulder
    distal left_elbow
    reference chest forward

SMPL_LeftElbow:
    proximal left_elbow
    distal left_wrist
    reference chest forward
```

右侧相同规则。

## 7.4 Foot/ankle target frame

位置：

```text
left ankle joint 7
right ankle joint 8
```

方向：

```python
x = normalize(foot_joint - ankle_joint)

z0 = world_up
z = normalize(z0 - x * dot(z0, x))

y = normalize(z × x)
z = normalize(x × y)

R = [x, y, z]
```

## 7.5 Wrist target frame

22-joint SMPL 没有可靠手掌方向。

位置：

```text
wrist joint
```

rotation：

```text
继承对应 Forearm frame
```

最终 E1 配置中 Wrist/Hand rotation weight 默认设 0 或最多 1；不得复制 Xsens Hand rotation weight=5/10。

## 7.6 退化与时序连续性

当 reference 与 primary 近似平行：

```text
优先使用上一帧 frame
首帧才 fallback 到 SMPL FK joint rotation
```

输出 quaternion 必须做 sign continuity：

```python
if dot(q_curr, q_prev) < 0:
    q_curr = -q_curr
```

禁止 180°随机跳变。

---

# 8. shape、层级缩放和 root translation

## 8.1 betas 稳定

复用或迁移当前 `BetaStabilizer`：

```text
mean 前 30 个有效结果后冻结
```

默认不能 per-frame 改变骨架长度。

## 8.2 joint-center hierarchy

层级：

```text
SMPL_Pelvis -> SMPL_Chest

SMPL_Pelvis -> SMPL_LeftHip
SMPL_LeftHip -> SMPL_LeftKnee
SMPL_LeftKnee -> SMPL_LeftAnkle

SMPL_Pelvis -> SMPL_RightHip
SMPL_RightHip -> SMPL_RightKnee
SMPL_RightKnee -> SMPL_RightAnkle

SMPL_Chest -> SMPL_LeftShoulder
SMPL_LeftShoulder -> SMPL_LeftElbow
SMPL_LeftElbow -> SMPL_LeftWrist

SMPL_Chest -> SMPL_RightShoulder
SMPL_RightShoulder -> SMPL_RightElbow
SMPL_RightElbow -> SMPL_RightWrist
```

缩放：

```python
scaled[root] = root_position * root_translation_scale

scaled[child] =
    scaled[parent]
    + edge_scale[parent->child] *
      (raw[child] - raw[parent])
```

不允许 midpoint hierarchy。

## 8.3 竖直模式

支持：

```text
--smpl_vertical_mode gem|foot_lock|contact
```

先实现并默认 `foot_lock`，保证站立、抬手、T-pose、蹲下正确。

### foot_lock

heading 对齐后：

```python
pelvis_raw = joints[0]
joints_rel = joints - pelvis_raw

candidate_left  = -joints_rel[10, 2]
candidate_right = -joints_rel[11, 2]

pelvis_z = median(candidate_left, candidate_right)

root_xy = pelvis_raw[:2] - initial_pelvis_xy
root_out = [root_xy.x, root_xy.y, pelvis_z]

joints_out = joints_rel + root_out
```

蹲下时必须：

```text
脚保持 z≈0
Pelvis z 降低
```

### contact

在 foot_lock 通过后实现：

```text
接触时用支撑脚 candidate 重建 pelvis z
腾空时从 takeoff 时刻保留 GEM vertical delta
落地后重新锁地
```

必须有 hysteresis 和 synthetic tests。

---

# 9. 新增 SMPL → E1 专用配置

新增：

```text
config/ik_configs/smpl_direct_to_e1.json
config/ik_configs/smpl_direct_to_e1_position_debug.json
config/ik_configs/smpl_direct_to_e1_calibrated.json
```

不得复用 `xsens_to_e1.json` 的 position/rotation offsets。

## 9.1 human_scale_table

全部：

```text
1.0
```

因为缩放已在 SMPL adapter 完成。

## 9.2 robot mapping

```text
base_link
    <- SMPL_Pelvis

waist_roll_link
    <- SMPL_Chest

l_leg_hip_pitch_link
    <- SMPL_LeftHip
l_leg_knee_link
    <- SMPL_LeftKnee
l_leg_ankle_roll_link
    <- SMPL_LeftAnkle

r_leg_hip_pitch_link
    <- SMPL_RightHip
r_leg_knee_link
    <- SMPL_RightKnee
r_leg_ankle_roll_link
    <- SMPL_RightAnkle

l_arm_shoulder_roll_link
    <- SMPL_LeftShoulder
l_arm_elbow_pitch_link
    <- SMPL_LeftElbow
l_arm_elbow_yaw_link
    <- SMPL_LeftWrist

r_arm_shoulder_roll_link
    <- SMPL_RightShoulder
r_arm_elbow_pitch_link
    <- SMPL_RightElbow
r_arm_elbow_yaw_link
    <- SMPL_RightWrist
```

## 9.3 完整配置必须使用 position + rotation

position-debug 文件允许 rotation=0。

正式 base/calibrated 文件：

```text
Pelvis/Chest/hips/knees/shoulders/elbows：
position 和 rotation 都非零

Ankle：
position 高权重，rotation 中高权重

Wrist：
position 非零
rotation 默认 0 或 1
```

不要把 position-debug 作为最终启动默认值。

建议初始权重保守：

```text
table1：
Pelvis/Chest/limb rotations 5
Ankle rotation 10
Wrist rotation 0

table2：
Pelvis/Chest/hips/knees/shoulders/elbows:
    pos 10, rot 2

Ankle:
    pos 100, rot 10

Wrist:
    pos 20, rot 0
```

后续可在校准验证后提高，但不要直接复制 Xsens foot 50 和 hand 5/10。

---

# 10. 新增 SMPL → E1 联合标定

新增：

```text
scripts/calibrate_smpl_direct_to_e1.py
```

同时输出：

```text
GENMO/config/gmr/smpl_direct_e1_adapter_calibrated.json
GMR-CPP/config/ik_configs/smpl_direct_to_e1_calibrated.json
```

参考姿态：

```text
直立
双脚平行
双臂自然下垂
肘部伸直
面向 +X
```

对每个 target：

```text
human:
p_h, R_h

robot body neutral:
p_r, R_r
```

GMR offset 公式：

```python
R_offset = R_h.T @ R_r

p_offset_local =
    R_r.T @ (p_r - p_h)
```

因为 GMR 使用：

```text
R_target = R_h @ R_offset
p_target = p_h + R_target @ p_offset_local
```

同时标定：

```text
root_translation_scale
13 条 hierarchy edge_scale
```

要求：

1. 标定输入必须是 joint-center v1/v2 direct adapter，不允许 midpoint adapter。
2. 两个输出写相同 calibration ID。
3. 启动前检查 calibration ID。
4. 输出每个 body 的 reference position/rotation residual。
5. 输出左右对称差。
6. 旧 Xsens 配置不动。

---

# 11. 可选：实现真正可标定的 SMPL → Xsens 模式

不是正式默认路径，但新增：

```text
scripts/calibrate_smpl_to_xsens.py
```

该工具不得硬编码“猜测的 Xsens offsets”。

输入同步记录：

```text
SMPL direct segment poses
XsensReader yaw-normalized segment poses
```

对每个 segment 拟合：

```python
R_smpl_to_xsens =
    mean_SO3(R_smpl.T @ R_xsens)

local_position_offset =
    mean(R_xsens.T @ (p_xsens - p_smpl_scaled))
```

运行时：

```python
R_xsens_like =
    R_smpl @ R_smpl_to_xsens

p_xsens_like =
    p_smpl_scaled
    + R_xsens_like @ local_position_offset
```

只有生成 paired calibration 后，才允许使用：

```text
xsens_to_e1.json
```

没有 paired Xsens 数据时，明确禁止把 direct SMPL frame 冒充 Xsens。

---

# 12. Render 与调试

## GENMO

新增开关：

```text
--smpl_gmr_debug
```

显示：

```text
灰色：22 SMPL joints
蓝色：14 direct SMPL target joint centers
RGB axes：direct anatomical frames
```

默认关闭，支持 `g` 键切换。

## MuJoCo

新增独立：

```text
--vis-smpl-targets
```

默认关闭。

显示：

```text
蓝色：GEM2 UDP SMPL targets
黄色：GMR offset 后的 IK targets
白色：E1 mapped bodies
```

普通 `--vis` 只显示 E1。

---

# 13. 新启动脚本

新增：

```text
run_smpl_direct_e1.sh
```

默认加载：

```text
smpl_direct_to_e1_calibrated.json
E1 XML
GEM2
base_link viewer follow
```

如果 calibrated adapter 或 calibrated IK 缺失/ID不匹配，直接报错退出。

不覆盖：

```text
run_gem.sh
run_gem_segments_e1.sh
run_xsens_e1.sh
```

---

# 14. 自动测试

## GENMO

新增：

```text
tests/test_smpl_direct_adapter.py
```

必须测试：

1. 14 target position 严格等于对应 joint center。
2. standing frame。
3. T-pose 左右对称。
4. 单臂侧抬。
5. 蹲下 foot_lock：
   - foot z≈0
   - pelvis z下降。
6. whole-body vertical drift 被消除。
7. jump/contact。
8. frame SO(3)。
9. quaternion sign continuity。
10. GEM2 packet 412 bytes。
11. GEM1 保持兼容。

运行：

```bash
cd /home/weili/GENMO

python3 -m py_compile \
  gem/smpl_direct_adapter.py \
  gem/gmr_udp_bridge.py \
  scripts/demo/demo_webcam.py

python3 -m unittest \
  tests.test_smpl_direct_adapter \
  tests.test_gmr_segment_adapter \
  tests.test_gmr_udp_bridge
```

## GMR-CPP

新增 GemReader GEM2 parser test，如项目没有 test harness，增加一个小型可执行 self-test。

运行：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

python3 -m py_compile \
  scripts/calibrate_smpl_direct_to_e1.py \
  scripts/calibrate_smpl_to_xsens.py

bash build.sh --clean

test -x build/gem_mocap_server
```

验证原 Xsens target 和脚本仍可构建。

---

# 15. 运行验收

## 15.1 Debug

GMR：

```bash
./run_gem.sh \
  --ik-config "$PWD/config/ik_configs/smpl_direct_to_e1_position_debug.json" \
  --always \
  --vis \
  --vis-smpl-targets
```

GENMO：

```bash
python scripts/demo/demo_webcam.py \
  --camera_id 0 \
  --no_imgfeat \
  --render \
  --render_mode opencv \
  --gmr_protocol gem2 \
  --gmr_host 127.0.0.1 \
  --gmr_port 7001 \
  --smpl_adapter_config \
    "$PWD/config/gmr/smpl_direct_e1_adapter.json" \
  --smpl_vertical_mode foot_lock \
  --smpl_gmr_debug
```

验证站立、T-pose、单臂侧抬、蹲下。

## 15.2 标定

运行新联合标定，生成配对的 calibrated 文件。

## 15.3 正式运行

GMR：

```bash
./run_smpl_direct_e1.sh --always --vis
```

GENMO：

```bash
python scripts/demo/demo_webcam.py \
  --camera_id 0 \
  --no_imgfeat \
  --render \
  --render_mode opencv \
  --gmr_protocol gem2 \
  --gmr_host 127.0.0.1 \
  --gmr_port 7001 \
  --smpl_adapter_config \
    "$PWD/config/gmr/smpl_direct_e1_adapter_calibrated.json" \
  --smpl_vertical_mode foot_lock
```

通过蹲下后再测试 contact/jump。

---

# 16. Codex 最终报告

必须输出：

1. XsensReader 已知/未知坐标语义整理。
2. 为什么不应仅凭 Reader 猜 proprietary Xsens local frames。
3. 新 GEM2/SMPL target name table。
4. 14 个 target position 和 frame 公式。
5. 直接 SMPL→E1 配置映射。
6. 联合标定公式。
7. 可选 SMPL→Xsens paired calibration。
8. 修改文件。
9. 自动测试结果。
10. C++ 编译结果。
11. 未能自动验证的 GUI 项目。
12. 完整重新标定和运行命令。
13. `git diff --stat`。
14. 明确声明未修改 GMR IK solver。
