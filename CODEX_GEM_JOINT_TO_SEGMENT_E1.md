# Codex 任务：只修复 GEM → GMR 输入适配，生成 Xsens-like 肢段位姿；不要修改 GMR IK 求解器

## 仓库

- GENMO：`/home/weili/GENMO`
- GMR-CPP：`/home/weili/GMR-CPP_e1jump_lowdpi`

开始前记录：

```bash
git -C /home/weili/GENMO status --short
git -C /home/weili/GMR-CPP_e1jump_lowdpi status --short
```

禁止：

- 不要 reset/checkout/clean。
- 不要修改 DAQP、Jacobian、IK 迭代、damping、关节限位或 `runIKStep/solveIK`。
- 不要破坏原 Xsens/OptiTrack/FZMotion/Pico 文件。
- 不要继续把 position-only 当最终方案。
- 不要引用任何 G1 模型或配置。

目标是新增一条独立链路：

```text
GEM SMPL-X joints
→ GEM segment adapter
→ Xsens-like 14 segment poses
→ GemReader
→ 新增 GEM-specific E1 IK config
→ 原 GMR IK
→ E1
```

---

# 一、先明确现有语义错误

当前 GENMO 使用：

```python
JOINT_INDICES = (0,9,1,2,4,5,7,8,16,17,18,19,20,21)
```

然后命名为：

```text
Pelvis
Chest
Left_UpperLeg
Right_UpperLeg
Left_LowerLeg
Right_LowerLeg
Left_Foot
Right_Foot
Left_UpperArm
Right_UpperArm
Left_Forearm
Right_Forearm
Left_Hand
Right_Hand
```

但实际位置分别是：

```text
Pelvis        = pelvis joint center
Chest         = spine3 joint center
UpperLeg      = hip joint center
LowerLeg      = knee joint center
Foot          = ankle joint center
UpperArm      = shoulder joint center
Forearm       = elbow joint center
Hand          = wrist joint center
```

实际旋转是 SMPL joint frame 的 FK rotation。

Xsens 同名数据表示的是人体 rigid segment pose：

```text
segment origin position
+
segment anatomical frame orientation
```

因此同名不等义。必须在 GEM 端构造 virtual segment pose，而不是把 joint center 直接改名为 segment。

---

# 二、新增独立适配器，不覆盖现有桥接逻辑

新增：

```text
/home/weili/GENMO/gem/gmr_segment_adapter.py
/home/weili/GENMO/config/gmr/e1_segment_adapter.json
```

保留原 `gmr_udp_bridge.py` 的 UDP 打包功能，但让它支持：

```python
send_segments(segment_poses)
```

其中：

```python
segment_poses: dict[str, SegmentPose]

SegmentPose:
    position_zup: Tensor/ndarray (3,)
    rotation_zup: Tensor/ndarray (3,3)
```

不得再让 UDP bridge 自己猜 segment frame。

---

# 三、构造 14 个 virtual anatomical segments

输入：

```text
joints_ay:     (22,3)
joint_rots_ay: (22,3,3) 仅用于 pelvis fallback
```

先将全部 joints 从 GEM AY/Y-up 转成 Z-up。

使用 joints 位置构造人体公共参考轴：

```text
left/right hip:       1 / 2
left/right shoulder: 16 / 17
pelvis:               0
chest:                9
neck:                12
left/right knee:      4 / 5
left/right ankle:     7 / 8
left/right foot:     10 / 11
left/right elbow:    18 / 19
left/right wrist:    20 / 21
```

人体 lateral：

```python
lateral = normalize(
    (right_hip - left_hip)
    + (right_shoulder - left_shoulder)
)
```

Z-up：

```python
up_world = [0,0,1]
```

人体 forward：

```python
forward = normalize(cross(lateral, up_world))
```

或根据实际左右定义选择相反号；必须添加合成测试确定人在正对摄像头、SMPL canonical forward 时输出为配置的 E1 forward。提供：

```text
--gmr_forward_sign +1|-1
```

作为显式调试参数，但正确默认值必须由测试确定。

---

## Segment frame 通用规则

每个 segment 构造：

```text
origin
primary axis
reference axis
third axis
```

用 Gram-Schmidt 建立右手 SO(3)，确保：

```text
R.T @ R = I
det(R) = +1
```

必须处理退化：

- 向量长度过小则沿用上一帧 frame；
- 对每根轴与上一帧做符号连续性选择；
- 禁止 180°随机翻转；
- 首帧退化才 fallback 到 SMPL joint rotation。

---

## 14 个 segment 定义

配置文件中每个 segment 都要包含：

```json
{
  "origin": {
    "proximal_joint": 16,
    "distal_joint": 18,
    "alpha": 0.5
  },
  "primary_axis": "proximal_to_distal",
  "reference": "chest_forward",
  "axis_layout": ["x","y","z"]
}
```

`alpha` 的含义：

```text
origin = proximal + alpha * (distal - proximal)
```

不要硬编码 Xsens segment origin 一定在 proximal joint。初始默认使用：

```text
Pelvis: pelvis joint
Chest: spine3 joint
UpperLeg/LowerLeg/UpperArm/Forearm: segment midpoint alpha=0.5
Foot: ankle 与 foot joint 中点
Hand: wrist joint，orientation 继承 forearm
```

这些 alpha 必须可配置并可由标定脚本修正。

建议定义：

```text
Pelvis:
  origin pelvis
  up pelvis→chest
  lateral hips
  forward derived

Chest:
  origin spine3
  up pelvis→neck
  lateral shoulders

Left_UpperLeg:
  origin midpoint(left_hip,left_knee)
  primary left_hip→left_knee
  reference pelvis_forward

Left_LowerLeg:
  origin midpoint(left_knee,left_ankle)
  primary left_knee→left_ankle
  reference pelvis_forward

Left_Foot:
  origin midpoint(left_ankle,left_foot)
  primary left_ankle→left_foot
  reference world_up / pelvis_lateral

Right side symmetric.

Left_UpperArm:
  origin midpoint(left_shoulder,left_elbow)
  primary left_shoulder→left_elbow
  reference chest_forward

Left_Forearm:
  origin midpoint(left_elbow,left_wrist)
  primary left_elbow→left_wrist
  reference chest_forward

Left_Hand:
  origin left_wrist
  frame inherit Left_Forearm

Right side symmetric.
```

注意左右 segment frame 必须使用统一 anatomical convention，不能因为 cross product 顺序使左右四元数镜像成不同坐标系。

---

# 四、在 GEM 端完成层级长度缩放，不再依赖 GMR 的 Xsens scale

新增 adapter calibration 配置：

```json
{
  "root_translation_scale": 1.0,
  "edge_scales": {
    "Pelvis->Chest": 1.0,
    "Pelvis->Left_UpperLeg": 1.0,
    "Left_UpperLeg->Left_LowerLeg": 1.0,
    "Left_LowerLeg->Left_Foot": 1.0,
    "Chest->Left_UpperArm": 1.0,
    "Left_UpperArm->Left_Forearm": 1.0,
    "Left_Forearm->Left_Hand": 1.0
  }
}
```

右侧同理。

按骨架层级缩放 segment origins：

```text
scaled[root] = normalized_root_translation * root_translation_scale

scaled[child] =
    scaled[parent]
    + edge_scale[parent->child] *
      (raw[child] - raw[parent])
```

禁止继续使用：

```text
(child - pelvis) * 每个名字不同scale
```

完成层级缩放后，新 GMR 配置中的：

```json
"human_scale_table"
```

对所有 14 个 segment 全部设为 `1.0`，避免 GMR 再次改变几何方向。

不修改 GMR `scaleHumanData()`，原动捕链路完全保持不变。

---

# 五、稳定 GEM 的 shape、heading 和 ground

## 1. 冻结 betas

当前每帧预测 betas。新增：

```text
--gmr_shape_mode first|mean|ema|per_frame
--gmr_shape_warmup 30
```

默认：

```text
mean
```

收集前 30 个有效推理结果，平均 betas 后固定；不能每帧改变骨架长度。

## 2. heading

新增：

```text
--gmr_heading_source joints|pelvis
```

默认 `joints`，使用 hips+shoulders 几何求 forward。

首帧对齐后固定 heading offset；不要每帧重新归零。

## 3. ground/contact lock

新增：

```text
--gmr_ground_mode initial|per_frame|contact
```

默认 `contact`。

使用 joint 10/11 脚部点：

- 接触时更新 ground；
- 腾空时冻结 ground；
- 静止 60 秒不允许根高度持续上升；
- jump 时不能把 jump 消掉。

---

# 六、新增 GEM 专用完整 E1 配置，不再使用 position-only 作为最终配置

新增：

```text
/home/weili/GMR-CPP_e1jump_lowdpi/config/ik_configs/gem_segments_to_e1.json
```

从 `xsens_to_e1.json` 复制：

- robot body mapping 保持；
- position/rotation weights 先保持原 Xsens 值；
- human names保持 14 个 canonical names；
- `human_scale_table` 全部设 1.0；
- rotation offsets 和 position offsets由新标定脚本写入；
- 不覆盖 `xsens_to_e1.json`。

另外保留：

```text
gem_segments_to_e1_position_debug.json
```

仅用于排查，rotation weight=0。最终运行脚本默认必须使用完整的：

```text
gem_segments_to_e1_calibrated.json
```

若 calibrated 文件不存在，启动脚本应明确报错并提示先标定，而不是悄悄退回 position-only。

---

# 七、新增“位置+旋转”联合标定脚本

新增：

```text
/home/weili/GMR-CPP_e1jump_lowdpi/scripts/calibrate_gem_segments_to_e1.py
```

输入：

```text
--xml assets/e1/mjcf/e1_24dof.xml
--base-config config/ik_configs/gem_segments_to_e1.json
--output config/ik_configs/gem_segments_to_e1_calibrated.json
--adapter-output /home/weili/GENMO/config/gmr/e1_segment_adapter_calibrated.json
--redis-key gmt_online_frame_e1
--duration 5
```

采集：

```text
gmt_online_frame_e1_raw_bones
```

操作者保持 E1 neutral reference：

```text
直立
双脚平行
双臂自然下垂
肘部伸直
面向固定方向
```

对每个映射：

```text
human segment:
    p_h
    R_h

E1 neutral robot body:
    p_r
    R_r
```

计算旋转偏置：

```text
R_offset = R_h^T @ R_r
q_offset = quat(R_offset)
```

GMR 应满足：

```text
R_target = R_h @ R_offset = R_r
```

计算位置偏置。GMR 当前应用规则是：

```text
p_target = p_h + R_target @ pos_offset_local
```

因此：

```text
pos_offset_local = R_r^T @ (p_r - p_h)
```

把：

```text
entry[3] = pos_offset_local
entry[4] = q_offset_wxyz
```

写入两张 IK table。

注意：

1. 标定前 segment adapter 已完成层级 scale。
2. 标定脚本同时输出每个 body 的位置残差和旋转残差。
3. 标定后 reference pose 中误差应接近 0。
4. 左右残差需对称。
5. 不修改 GMR IK 算法。

---

# 八、增加端到端转换诊断，而不是只看绿色 mesh

新增 GENMO 调试记录：

```text
scripts/debug_gem_gmr_segments.py
```

支持记录 10 秒：

```text
raw SMPL joints 22×3
derived 14 segment origins
derived 14 segment rotations
scaled 14 segment origins
UDP payload
frame id/timestamp
```

输出 NPZ/JSON。

增加可视化窗口：

```text
--gmr_debug_skeleton
```

在 GEM OpenCV 画面或 Viser 中同时画：

```text
SMPL joint skeleton
derived segment origin
segment xyz axes
```

必须能够看到：

- UpperArm origin 是否在肩肘之间；
- Forearm origin 是否在肘腕之间；
- segment primary axis 是否沿骨段；
- 左右轴是否对称；
- 自然下垂和 T-pose 是否合理。

---

# 九、GMR Viewer 增加 human target overlay，但不改 IK

修改 Viewer/ProcessedFrame，只用于显示：

```text
raw GEM segment targets
GMR scaled+offset targets
E1 robot body positions
```

颜色区分：

```text
raw segment      蓝色
scaled/offset    黄色
robot body       白色
```

连线显示人体链。

目的是直接判断：

```text
转换目标错
还是机器人没跟上
```

不得改变 IK QP、权重、damping、迭代或关节限制。

---

# 十、运行脚本

新增：

```text
run_gem_segments_e1.sh
```

不要覆盖原 `run_gem.sh`。

默认：

```text
E1 XML
GemReader
gem_segments_to_e1_calibrated.json
gmt_online_frame_e1
--vis
--viewer-follow-body base_link
```

若 calibrated config 不存在，明确退出。

GENMO 新增参数：

```text
--gmr_adapter_config
--gmr_shape_mode
--gmr_shape_warmup
--gmr_ground_mode
--gmr_heading_source
--gmr_forward_sign
--gmr_debug_skeleton
```

---

# 十一、自动测试

## GENMO

新增：

```text
tests/test_gmr_segment_adapter.py
```

必须覆盖：

1. 自然站立：
   - shoulder→elbow、elbow→wrist向下；
   - segment frame proper SO(3)。
2. T-pose：
   - 左右 UpperArm primary axes水平向外；
   - 左右对称。
3. 单臂侧抬：
   - 抬起侧 UpperArm orientation变化；
   - elbow不被错误识别成 shoulder rotation。
4. midpoint origins正确。
5. hierarchical scale只改变长度，不改变方向。
6. betas固定后骨段长度稳定。
7. heading front/90/180测试。
8. contact ground漂移/jump/landing测试。
9. UDP仍严格412 bytes。

运行：

```bash
cd /home/weili/GENMO
python3 -m py_compile \
  gem/gmr_segment_adapter.py \
  gem/gmr_udp_bridge.py \
  scripts/demo/demo_webcam.py
python3 -m unittest tests.test_gmr_segment_adapter tests.test_gmr_udp_bridge
```

## GMR-CPP

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
bash build.sh --clean
```

不允许修改 IK solver核心。

---

# 十二、人工验收

1. 自然站立：
   - derived UpperArm/Forearm segments向下；
   - E1双手不交叉；
   - 两侧对称。
2. T-pose：
   - E1两侧都侧平举；
   - 不能一侧下垂、一侧肘屈上举。
3. 单臂侧抬：
   - 主要由shoulder完成；
   - elbow保持接近人体实际角度。
4. 静止60秒：
   - E1根高度漂移小于2cm。
5. Viewer初始正面。
6. Redis长度152 bytes。
7. 完整配置的position和rotation weights均为非零，不能长期运行position-only。
8. 原Xsens链路不受影响。

---

# 十三、Codex最终报告

必须给出：

- 新增/修改文件；
- joint pose与segment pose区别；
- segment frame公式；
- segment origin规则；
- hierarchical scale公式；
- position/rotation offset标定公式；
- 自动测试和编译结果；
- 未能自动执行的GUI验证；
- 完整运行命令；
- `git diff --stat`；
- 明确说明没有修改GMR IK solver。
