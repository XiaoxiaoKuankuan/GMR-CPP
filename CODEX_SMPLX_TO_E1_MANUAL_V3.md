# Codex 修改任务：用固定经验参数重写 SMPL-X→E1 配置，不做动态标定或自动修正

## 0. 任务原则

当前事实：

```text
GEM → SMP1/原版 SMPL-X targets → 原版 smplx_to_g1.json → G1
```

已经验证正确。

因此本任务把以下部分视为正确，不允许再改：

```text
GENMO 的 SMP1 输出
14 个 SMPL-X 关节名称和顺序
AY→Z-up 的 world-only 旋转
SMPL-X global joint rotations
SmplxReader
GMR IK solver
```

当前问题只处理：

```text
SMPL-X targets → E1 robot bodies 的固定 config
```

用户明确不接受：

```text
在线动态校准
自动搜索 quaternion
根据画面自动调权重
运行时动态修改 offset
自动 foot/contact 修正
```

本任务采用：

```text
图像现象
+ 原版 smplx_to_g1
+ 已验证的 xrobot_to_e1
+ xsens_to_e1 / fbx_to_e1 的权重经验
```

写出一套固定、可解释、可手工复现的 E1 配置。

---

# 1. 工作目录

```text
/home/weili/GMR
/home/weili/GENMO
/home/weili/GMR-CPP_e1jump_lowdpi
```

开始前执行：

```bash
git -C /home/weili/GMR status --short
git -C /home/weili/GENMO status --short
git -C /home/weili/GMR-CPP_e1jump_lowdpi status --short
```

禁止：

```text
git reset --hard
git checkout .
git clean
```

禁止修改：

```text
GENMO/gem/smplx_gmr_reference.py 的 target语义
readers/smplx_reader.*
include/gmr/gmr_mink.hpp 的 IK求解
DAQP/Jacobian/damping/joint limit/迭代参数
原 smplx_to_g1.json
原 xrobot_to_e1.json
原 xsens_to_e1.json
原 fbx_to_e1.json
```

---

# 2. 先输出当前配置差异报告

新增：

```text
docs/SMPLX_E1_MANUAL_CONFIG_REASONING.md
scripts/compare_smplx_e1_configs.py
```

对比：

```text
GMR/general_motion_retargeting/ik_configs/smplx_to_g1.json
GMR-CPP/config/ik_configs/smplx_to_e1.json
GMR-CPP/config/ik_configs/xrobot_to_e1.json
GMR-CPP/config/ik_configs/xsens_to_e1.json
GMR-CPP/config/ik_configs/fbx_to_e1.json
```

必须打印：

```text
human name
robot body
table1/table2 pos weight
table1/table2 rot weight
rotation offset wxyz
position offset
```

重点指出当前 `smplx_to_e1.json` 与 `xrobot_to_e1.json` 的手臂 quaternion 差异。

必须在报告中明确写出：

## 当前左臂

当前 SMPL-X→E1 左肩约为：

```text
[+0.7071, 0, -0.7071, 0]
```

xrobot→E1 左肩/肘/腕为：

```text
[+0.7071, 0, +0.7071, 0]
```

这不是 quaternion 的整体正负等价，而是相反的 Y 轴 90°旋转。

当前左肘接近 identity，但 xrobot 左肘使用：

```text
[+0.7071, 0, +0.7071, 0]
```

这与截图中的：

```text
T-pose 左肘折回
手臂下垂时左肘向上翻
```

一致。

## 当前右臂

当前右肩约为：

```text
[0, +0.7071, 0, +0.7071]
```

xrobot 右肩/肘/腕为：

```text
[0, +0.7071, 0, -0.7071]
```

同样不是整体符号等价。

因此本次配置不再沿用当前 smplx_to_e1 的手臂 quaternion。

---

# 3. 固定配置来源规则

新配置必须遵循以下固定规则，不能运行时自动改变。

## 3.1 Human names 和 scale

来自已验证的原版：

```text
smplx_to_g1.json
```

保持：

```text
pelvis/spine3/hips/knees/feet = 0.9
shoulders/elbows/wrists = 0.8
```

## 3.2 E1 robot body mapping

来自已验证的：

```text
xrobot_to_e1.json
```

保持：

```text
base_link             <- pelvis
waist_roll_link       <- spine3

l/r_leg_hip_pitch_link  <- left/right_hip
l/r_leg_knee_link       <- left/right_knee
l/r_leg_ankle_roll_link <- left/right_foot

l/r_arm_shoulder_roll_link <- left/right_shoulder
l/r_arm_elbow_pitch_link   <- left/right_elbow
l/r_arm_elbow_yaw_link     <- left/right_wrist
```

不要在本任务中改成 shoulder_yaw virtual body，也不要新增虚拟 wrist/foot body。

原因：

```text
xrobot_to_e1 已经证明这组 E1 robot body mapping 可用；
当前先只修 human frame offset 与权重，控制变量。
```

## 3.3 Rotation offsets

固定使用 `xrobot_to_e1.json` 的 E1 rotation offsets。

仅把 human names 改成小写 SMPL-X名称。

固定 quaternion：

```text
pelvis:
[-0.5, +0.5, -0.5, -0.5]

spine3:
[-0.5, +0.5, -0.5, -0.5]

left_hip:
[-0.56379311, +0.42677550, -0.42677550, -0.56379311]

right_hip:
[-0.56379311, +0.42677550, -0.42677550, -0.56379311]

left_knee / right_knee:
[-0.5, +0.5, -0.5, -0.5]

left_foot / right_foot:
[-0.5, +0.5, -0.5, -0.5]

left_shoulder / left_elbow / left_wrist:
[+0.70710678, 0.0, +0.70710678, 0.0]

right_shoulder / right_elbow / right_wrist:
[0.0, +0.70710678, 0.0, -0.70710678]
```

不要使用当前 smplx_to_e1 中由 G1 offset小幅修改出来的 quaternion。

不要平均 Xsens、FBX、XRobot 的 quaternion。

## 3.4 Position offsets

第一版全部保持：

```text
[0,0,0]
```

和 xrobot_to_e1 一致。

本任务不做自动 position offset 标定。

---

# 4. 新建固定配置文件

不要覆盖当前文件。

新增：

```text
config/ik_configs/smplx_to_e1_manual_v3.json
run_smplx_e1_manual_v3.sh
```

`run_smplx_e1_manual_v3.sh` 必须显式加载新配置。

固定配置内容按下面写入。

```json
{
  "robot_root_name": "base_link",
  "human_root_name": "pelvis",
  "ground_height": 0.0,
  "human_height_assumption": 1.8,
  "use_ik_match_table1": true,
  "use_ik_match_table2": true,

  "human_scale_table": {
    "pelvis": 0.9,
    "spine3": 0.9,

    "left_hip": 0.9,
    "right_hip": 0.9,
    "left_knee": 0.9,
    "right_knee": 0.9,
    "left_foot": 0.9,
    "right_foot": 0.9,

    "left_shoulder": 0.8,
    "right_shoulder": 0.8,
    "left_elbow": 0.8,
    "right_elbow": 0.8,
    "left_wrist": 0.8,
    "right_wrist": 0.8
  },

  "ik_match_table1": {
    "base_link":
      ["pelvis", 0, 10, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "waist_roll_link":
      ["spine3", 0, 8, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "l_leg_hip_pitch_link":
      ["left_hip", 0, 6, [0,0,0],
       [-0.56379311,0.42677550,-0.42677550,-0.56379311]],

    "l_leg_knee_link":
      ["left_knee", 0, 6, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "l_leg_ankle_roll_link":
      ["left_foot", 100, 8, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "r_leg_hip_pitch_link":
      ["right_hip", 0, 6, [0,0,0],
       [-0.56379311,0.42677550,-0.42677550,-0.56379311]],

    "r_leg_knee_link":
      ["right_knee", 0, 6, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "r_leg_ankle_roll_link":
      ["right_foot", 100, 8, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "l_arm_shoulder_roll_link":
      ["left_shoulder", 0, 8, [0,0,0],
       [0.70710678,0.0,0.70710678,0.0]],

    "l_arm_elbow_pitch_link":
      ["left_elbow", 0, 5, [0,0,0],
       [0.70710678,0.0,0.70710678,0.0]],

    "l_arm_elbow_yaw_link":
      ["left_wrist", 0, 1, [0,0,0],
       [0.70710678,0.0,0.70710678,0.0]],

    "r_arm_shoulder_roll_link":
      ["right_shoulder", 0, 8, [0,0,0],
       [0.0,0.70710678,0.0,-0.70710678]],

    "r_arm_elbow_pitch_link":
      ["right_elbow", 0, 5, [0,0,0],
       [0.0,0.70710678,0.0,-0.70710678]],

    "r_arm_elbow_yaw_link":
      ["right_wrist", 0, 1, [0,0,0],
       [0.0,0.70710678,0.0,-0.70710678]]
  },

  "ik_match_table2": {
    "base_link":
      ["pelvis", 30, 5, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "waist_roll_link":
      ["spine3", 10, 4, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "l_leg_hip_pitch_link":
      ["left_hip", 15, 3, [0,0,0],
       [-0.56379311,0.42677550,-0.42677550,-0.56379311]],

    "l_leg_knee_link":
      ["left_knee", 20, 3, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "l_leg_ankle_roll_link":
      ["left_foot", 100, 8, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "r_leg_hip_pitch_link":
      ["right_hip", 15, 3, [0,0,0],
       [-0.56379311,0.42677550,-0.42677550,-0.56379311]],

    "r_leg_knee_link":
      ["right_knee", 20, 3, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "r_leg_ankle_roll_link":
      ["right_foot", 100, 8, [0,0,0], [-0.5,0.5,-0.5,-0.5]],

    "l_arm_shoulder_roll_link":
      ["left_shoulder", 15, 4, [0,0,0],
       [0.70710678,0.0,0.70710678,0.0]],

    "l_arm_elbow_pitch_link":
      ["left_elbow", 20, 2, [0,0,0],
       [0.70710678,0.0,0.70710678,0.0]],

    "l_arm_elbow_yaw_link":
      ["left_wrist", 30, 0, [0,0,0],
       [0.70710678,0.0,0.70710678,0.0]],

    "r_arm_shoulder_roll_link":
      ["right_shoulder", 15, 4, [0,0,0],
       [0.0,0.70710678,0.0,-0.70710678]],

    "r_arm_elbow_pitch_link":
      ["right_elbow", 20, 2, [0,0,0],
       [0.0,0.70710678,0.0,-0.70710678]],

    "r_arm_elbow_yaw_link":
      ["right_wrist", 30, 0, [0,0,0],
       [0.0,0.70710678,0.0,-0.70710678]]
  }
}
```

---

# 5. 这些权重为什么这样改

必须写入说明文档。

## 5.1 Pelvis position 10 → 30

截图中下蹲时 E1 更倾向于抬脚，而不是降低 base。

因此增强：

```text
base_link <- pelvis
```

的位置权重，让 E1 pelvis更明确跟随 SMPL pelvis下降。

## 5.2 Knee position 10 → 20

增强膝关节目标位置，帮助下蹲主要通过：

```text
pelvis下降 + knee flexion
```

完成。

## 5.3 Foot rotation 50 → 8

原 G1 的 50 是针对：

```text
SMPL foot → G1 toe_link
```

当前 E1 是：

```text
SMPL foot → E1 ankle_roll_link
```

不是同一 target frame。

继续用 50 会让脚 orientation压过 pelvis/knee位置，导致抬脚折中。

固定降到 8，不做动态改变。

## 5.4 Wrist rotation 10/5 → 1/0

E1没有 wrist roll/pitch/yaw。

仅能通过 elbow_yaw部分响应 wrist twist。

因此：

```text
table1 wrist rotation=1
table2 wrist rotation=0
```

保留 wrist position=30，避免手臂末端失去位置跟踪。

## 5.5 Shoulder/Elbow 使用 xrobot quaternion

因为当前 smplx E1 手臂 quaternion与 xrobot E1相差约180° frame方向，且截图正好表现为肘部反折。

这次固定替换，不做在线搜索。

---

# 6. 只增加诊断显示，不自动修改参数

扩展现有 Viewer 开关：

```text
--vis-smplx-targets
--vis-smplx-frames
```

显示：

```text
蓝色：raw SMPL-X targets
黄色：scale+offset 后的 targets
白色：E1 对应 body
```

frame axes：

```text
X 红
Y 绿
Z 蓝
```

重点显示：

```text
left/right shoulder
left/right elbow
left/right wrist
left/right foot
pelvis
```

增加控制台只读诊断，每0.5秒打印：

```text
l/r shoulder_pitch
l/r shoulder_roll
l/r shoulder_yaw
l/r elbow_pitch
l/r elbow_yaw
l/r knee
base_link z
left/right foot body z
```

禁止根据这些值自动修改 config。

---

# 7. 新增固定姿态测试，不做自动调参

新增：

```text
scripts/test_smplx_to_e1_manual_v3.py
```

输入合成的原版 SMPL-X targets：

```text
stand
T-pose
arms-down
left-arm-up
right-arm-up
left-elbow-flex 45°
right-elbow-flex 45°
squat
```

输出：

```text
E1 qpos
关键关节角
是否触及关节限位
target position error
target rotation error
```

判断规则：

## T-pose

```text
左右肩外展方向相反且对称
左右 elbow_pitch 接近0
不能有一侧肘大幅反折
```

## Arms-down

```text
左右 elbow_pitch 接近0
左右手臂向下
```

## Elbow flexion

E1 elbow_pitch范围为负方向弯曲：

```text
[-2.44, 0]
```

人体肘弯曲时，E1 elbow_pitch应向负方向变化。

## Squat

```text
base_link z明显下降
左右 knee增加弯曲
左右 foot body不明显抬离地面
```

测试只报告，不自动改参数。

---

# 8. 运行脚本

新增：

```text
run_smplx_e1_manual_v3.sh
```

固定：

```text
XML=assets/e1/mjcf/e1_24dof.xml
IK=config/ik_configs/smplx_to_e1_manual_v3.json
preset=e1
port=7005
```

默认不启用：

```text
--offset-to-ground
任何动态vertical/contact修正
任何自动calibration
```

运行：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi

./run_smplx_e1_manual_v3.sh \
  --always \
  --vis \
  --vis-smplx-targets \
  --vis-smplx-frames
```

GENMO 继续使用当前已验证的 SMP1发送命令，不改人体侧。

---

# 9. 回归检查

必须验证：

```text
run_smplx_g1.sh
```

行为不变。

执行：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
bash build.sh --clean
```

确认：

```text
smplx_g1_server
smplx_e1_server
```

均编译。

检查新配置：

```bash
python3 -m json.tool \
  config/ik_configs/smplx_to_e1_manual_v3.json >/dev/null
```

---

# 10. Codex最终输出

必须给出：

1. 当前 smplx_to_e1 与 xrobot_to_e1 的差异表。
2. 新固定 config完整内容。
3. 每个权重调整的经验依据。
4. 截图现象与 quaternion差异的对应解释。
5. 固定姿态测试输出。
6. C++编译结果。
7. G1回归结果。
8. 完整运行命令。
9. `git diff --stat`。
10. 明确说明：
   - 未做动态自动修正；
   - 未做在线标定；
   - 未修改GMR IK solver；
   - 未修改GENMO SMP1输入定义。
