# 下一阶段 Codex 任务：先复现原 GMR 的 SMPL-X→G1 权威基线，再建立 SMPL-X→E1

## 0. 目的

本任务不要替代当前正在修改的 E1 segment/direct 链路。等当前任务完成并保存后，新增一条完全独立的参考链路：

```text
GEM 实时 SMPL-X 参数
→ 精确复现原 Python GMR 的 SMPL-X frame dict
→ 原版 smplx_to_g1.json
→ G1 MuJoCo
```

先用原 GMR 已验证的 G1 配置验证：

```text
GEM → SMPL-X 数据构造
坐标系转换
全局 joint orientation
根高度
实时 UDP
C++ GMR
```

确认 G1 基线正确后，再新增：

```text
同一套 SMPL-X targets
→ smplx_to_e1.json
→ E1 MuJoCo
```

严禁修改原 Xsens/FBX/XRobot 配置，严禁修改 GMR IK 求解器。

---

# 1. 仓库

```text
原 Python GMR：
/home/weili/GMR
或从 https://github.com/XiaoxiaoKuankuan/GMR 获取的本地目录

GENMO：
/home/weili/GENMO

C++ GMR：
/home/weili/GMR-CPP_e1jump_lowdpi
```

开始前分别执行：

```bash
git status --short
git rev-parse HEAD
```

禁止：

```text
git reset --hard
git checkout .
git clean
```

---

# 2. 原 Python GMR 的 SMPL-X 输入是唯一权威定义

精确复现：

```text
general_motion_retargeting/utils/smpl.py
get_smplx_data()
get_smplx_data_offline_fast()
```

每帧 human data：

```python
dict[
    joint_name
] = (
    global_joint_position_xyz,
    global_joint_quaternion_wxyz,
)
```

位置：

```text
smplx_output.joints[i]
```

旋转：

```python
joint_global_R[0] = R.from_rotvec(global_orient)

joint_global_R[i] =
    joint_global_R[parent[i]]
    * R.from_rotvec(full_body_pose[i])
```

名称：

```text
smplx.joint_names.JOINT_NAMES
```

不要用：

```text
肢段中点
自己构造的 anatomical frame
Xsens-like frame
```

这条参考链路必须与原 Python GMR 完全一致。

---

# 3. 选择 14 个原版 SMPL-X target

固定名称和 SMPL-X 索引：

```text
pelvis          0
spine3          9

left_hip        1
right_hip       2

left_knee       4
right_knee      5

left_foot       10
right_foot      11

left_shoulder   16
right_shoulder  17

left_elbow      18
right_elbow     19

left_wrist      20
right_wrist     21
```

注意：

```text
原 smplx_to_g1.json 使用 left_foot/right_foot，
不是 left_ankle/right_ankle。
```

positions 必须是这些 joint centers。

rotations 必须是原 SMPL-X FK global joint rotations。

名称保持原版小写，不能改成 Xsens 名称。

---

# 4. 新增独立协议 SMPLX1

不要复用含糊的 GEM1 segment names。

协议：

```text
magic             b"SMP1"
version           uint16 = 1
item_count        uint16 = 14
sequence          uint32
source_stamp_ns   uint64

14 ×:
px py pz qw qx qy qz float32 little-endian
```

总长度仍为 412 bytes。

固定顺序就是上面的 14 个小写名称。

## GENMO

新增：

```text
gem/smplx_gmr_reference.py
```

## GMR-CPP

新增：

```text
readers/smplx_reader.hpp
readers/smplx_reader.cpp
```

不要让现有 GemReader 同时承担太多不同语义；独立 Reader 更容易核对。

---

# 5. 坐标系不能猜，必须做数值 parity test

原 Python GMR `smpl.py` 没有额外转换 SMPL-X joints/orientations。

GEM rollout 输出为 AY/Y-up。必须通过同一帧参数数值对比确定 GEM→原 GMR SMPL-X convention 的变换。

新增：

```text
/home/weili/GENMO/scripts/compare_genmo_fk_with_original_gmr_smplx.py
```

对同一组：

```text
body_pose
betas
global_orient
transl
```

分别计算：

A. 原 GMR `smplx.create(...)` + `get_smplx_data`
B. GENMO `EnDecoder.fk_v2`

测试候选：

```text
none:
p' = p
R' = R

world-only:
p' = C p
R' = C R

basis-conjugate:
p' = C p
R' = C R C^T
```

其中：

```python
C = [[1,0,0],
     [0,0,-1],
     [0,1,0]]
```

选择与原 GMR 输出误差最小且满足预期坐标系的公式。

不要凭“identity 应保持 identity”猜测，因为原 `smplx_to_g1.json` 的 rotation offsets绑定了原 SMPL-X local-frame convention。

验收：

```text
14 joint position max error < 1e-5 m
14 orientation angular max error < 1e-5 rad
```

若模型实现导致浮点差异，可放宽到 1e-4，但必须打印结果。

---

# 6. 复制原版 G1 资产与配置，不做重调

从原 GMR复制并保持数值不变：

```text
general_motion_retargeting/ik_configs/smplx_to_g1.json
assets/unitree_g1/g1_mocap_29dof.xml
```

写入 C++ 仓库的新文件：

```text
config/ik_configs/smplx_reference_to_g1.json
assets/unitree_g1/g1_mocap_29dof.xml
```

如果 C++ 仓库已经有同 SHA/同内容资产，不重复复制，但验证 body names。

不得修改原 config 中：

```text
human_scale_table
position weights
rotation weights
position offsets
rotation offsets
```

原 config 的关键人类名称为：

```text
pelvis
spine3
left/right_hip
left/right_knee
left/right_foot
left/right_shoulder
left/right_elbow
left/right_wrist
```

---

# 7. 新增 G1 server 和运行脚本

新增：

```text
apps/smplx_g1_server.cpp
run_smplx_g1.sh
```

结构复用现有 server：

```text
SmplxReader
FrameQueue
MotionBuffer
原 GMR IK
MuJoCo Viewer
```

G1 默认：

```text
XML=assets/unitree_g1/g1_mocap_29dof.xml
IK=config/ik_configs/smplx_reference_to_g1.json
preset=g1
Redis key=smplx_online_frame_g1
29 joints
43 floats / 172 bytes
```

不能影响现有 E1 `run_gem.sh`。

---

# 8. 实现原 GMR 风格 render

原 Python 脚本将：

```python
human_motion_data=retarget.scaled_human_data
```

传给 `RobotMotionViewer.step()`。

C++ Viewer 新增显式开关：

```text
--vis-smplx-targets
```

显示：

```text
蓝色：UDP raw SMPL-X joint targets
黄色：GMR scale+offset 后 targets
白色：G1 mapped robot bodies
```

连接拓扑：

```text
pelvis-spine3
pelvis-left_hip-left_knee-left_foot
pelvis-right_hip-right_knee-right_foot
spine3-left_shoulder-left_elbow-left_wrist
spine3-right_shoulder-right_elbow-right_wrist
```

默认关闭。

普通 `--vis` 只显示机器人。

---

# 9. 建立 Python GMR 与 C++ GMR 双路对照

新增：

```text
scripts/compare_python_gmr_and_cpp_gmr_g1.py
```

流程：

1. 记录一段 GEM 的 14 个原版 SMPL-X targets。
2. 同一序列同时输入：
   - 原 Python GMR + 原 `smplx_to_g1.json`
   - C++ GMR + 复制的相同 config/XML
3. 保存：
   ```text
   raw targets
   scaled targets
   qpos_python
   qpos_cpp
   ```
4. 比较：
   ```text
   target position/orientation 必须一致
   qpos 允许因 Mink/C++ solver实现不同有小差异
   ```
5. 输出逐关节 RMSE 和最大误差。

不要为了让 qpos 完全相同而修改 C++ IK；目标是输入与配置 parity。

---

# 10. G1 人工动作验收

实时 GEM + G1：

```text
自然站立
T-pose
单臂侧平举
双臂自然下垂
慢速蹲下
原地转身
抬腿
跳跃
```

检查：

```text
目标骨架与 G1 动作方向一致
左右不反
根朝向正确
脚不穿地
蹲下时 pelvis下降
静止高度不漂
```

如果原 Python GMR 正确、C++不正确：

```text
问题在 C++ port/server
```

如果二者都不正确：

```text
问题在 GEM→原版 SMPL-X target 数据
```

---

# 11. G1 基线通过后新增 SMPL-X→E1

新增：

```text
config/ik_configs/smplx_to_e1.json
config/ik_configs/smplx_to_e1_calibrated.json
run_smplx_e1.sh
```

保留同一套 14 个原版 SMPL-X human names和frames，不要再改 human-side convention。

## 11.1 Robot body mapping

```text
base_link
    <- pelvis

waist_roll_link
    <- spine3

l_leg_hip_pitch_link
    <- left_hip
l_leg_knee_link
    <- left_knee
l_leg_ankle_roll_link
    <- left_foot

r_leg_hip_pitch_link
    <- right_hip
r_leg_knee_link
    <- right_knee
r_leg_ankle_roll_link
    <- right_foot

l_arm_shoulder_roll_link
    <- left_shoulder
l_arm_elbow_pitch_link
    <- left_elbow
l_arm_elbow_yaw_link
    <- left_wrist

r_arm_shoulder_roll_link
    <- right_shoulder
r_arm_elbow_pitch_link
    <- right_elbow
r_arm_elbow_yaw_link
    <- right_wrist
```

## 11.2 初始 scale

原版 SMPL-X→G1 和现有 FBX/XRobot→E1 都体现：

```text
pelvis/chest/legs = 0.9
arms = 0.8
```

因此作为 E1 初始值：

```text
pelvis/spine3/hips/knees/feet = 0.9
shoulders/elbows/wrists = 0.8
```

但最终允许重新标定。

## 11.3 不要平均不同配置的 quaternion

以下配置仅用于 robot-side参考：

```text
xsens_to_e1.json
fbx_to_e1.json
xrobot_to_e1.json
```

不要对其中 quaternion 求平均。

正式 SMPL-X offset 通过 neutral reference解析：

```python
R_offset = R_smplx_reference.T @ R_e1_neutral

p_offset_local =
    R_e1_neutral.T
    @ (p_e1_neutral - p_smplx_scaled_reference)
```

因为 GMR 使用：

```text
R_target = R_human @ R_offset
p_target = p_human + R_target @ p_offset_local
```

## 11.4 配置模板选择

优先用：

```text
xrobot_to_e1.json
```

作为 E1 robot-body mapping与权重参考，因为其 human names已经是：

```text
Pelvis
Spine3
Left_Hip
Left_Knee
Left_Foot
Left_Shoulder
Left_Elbow
Left_Wrist
```

语义最接近 SMPL-X joint centers。

名称改为原 GMR 的小写 SMPL-X名称，offset重新计算。

`fbx_to_e1` 和 `xsens_to_e1` 只用来交叉检查：

```text
robot body选择
table1/table2结构
weights数量级
foot task强度
```

---

# 12. E1 联合标定与验证

新增：

```text
scripts/calibrate_smplx_to_e1.py
```

参考姿态：

```text
直立
双脚平行
双臂自然下垂
肘伸直
面向 +X
```

输出：

```text
smplx_to_e1_calibrated.json
```

打印：

```text
每个 target 的 neutral position residual
rotation residual
左右对称差
foot physical sole height
```

E1 render 使用同一个 SMPL-X target overlay，不重新发明 human render格式。

---

# 13. “完美配置”的验收标准

不能只在一个 neutral pose上正确。

建立动作测试集：

```text
stand
T-pose
arms down
left/right arm raise
squat
single-leg raise
turn
walk in place
jump/land
```

对 G1 和 E1 都记录：

```text
target body position error
target body rotation error
left/right symmetry
foot-ground error
pelvis height response
joint-limit saturation
```

SMPL-X→E1 的“完美”定义是：

```text
输入格式与原 GMR SMPL-X一致
neutral offsets解析正确
动作集误差低
无固定翻转
无明显穿地
蹲下/跳跃根高度正确
不破坏原 Xsens/FBX/XRobot链路
```

---

# 14. 自动测试

## GENMO

```bash
cd /home/weili/GENMO

python3 -m py_compile \
  gem/smplx_gmr_reference.py \
  scripts/compare_genmo_fk_with_original_gmr_smplx.py

python3 -m unittest \
  tests.test_smplx_gmr_reference
```

测试：

```text
14 names/order
position parity
orientation parity
SMP1 packet=412 bytes
coordinate transform candidate selection
```

## GMR-CPP

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

bash build.sh --clean

test -x build/smplx_g1_server
test -x build/smplx_e1_server
```

确保现有 server仍能编译。

---

# 15. Codex 最终报告

必须输出：

1. 原 GMR SMPL-X 输入定义。
2. GENMO 与原 GMR FK parity 数值。
3. 最终选用的坐标转换公式及证据。
4. G1复制的 XML/config SHA或diff。
5. Python/C++ G1 target parity。
6. G1人工验证命令。
7. SMPL-X→E1 mapping、scale、offset推导。
8. E1标定和运行命令。
9. render颜色/含义。
10. 自动测试与编译结果。
11. 未完成的GUI验证。
12. `git diff --stat`。
13. 明确声明未修改 GMR IK 求解器。
