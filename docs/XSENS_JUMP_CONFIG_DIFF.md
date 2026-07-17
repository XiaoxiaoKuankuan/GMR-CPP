# Xsens base → jump 配置逐字段差异

四元数比较将 `q` 与 `-q` 视为同一旋转。

## G1

- base: `/home/weili/GMR-CPP_e1jump_lowdpi/config/ik_configs/xsens_to_g1.json`
- jump: `/home/weili/GMR-CPP_e1jump_lowdpi/config/ik_configs/xsens_to_g1_jump.json`

### 顶层字段

| 字段 | base → jump | 是否改变 |
|---|---:|---|
| `robot_root_name` | `pelvis` | 否 |
| `human_root_name` | `Pelvis` | 否 |
| `ground_height` | `0.0` | 否 |
| `human_height_assumption` | `1.8` | 否 |
| `use_ik_match_table1` | `True` | 否 |
| `use_ik_match_table2` | `True` | 否 |

### Human scale

| Human body | base → jump | Δ | 比例 |
|---|---:|---:|---:|
| `Pelvis` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Chest` | `0.8 → 0.85` | 0.05 | 1.0625 |
| `Left_UpperLeg` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Right_UpperLeg` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Left_LowerLeg` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Right_LowerLeg` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Left_Foot` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Right_Foot` | `0.8 → 0.95` | 0.15 | 1.1875 |
| `Left_UpperArm` | `0.75` | 0 | 1 |
| `Right_UpperArm` | `0.75` | 0 | 1 |
| `Left_Forearm` | `0.75` | 0 | 1 |
| `Right_Forearm` | `0.75` | 0 | 1 |
| `Left_Hand` | `0.75` | 0 | 1 |
| `Right_Hand` | `0.75` | 0 | 1 |

### `ik_match_table1`

| Robot body | Human body | pos base→jump | pos Δ/ratio | rot base→jump | rot Δ/ratio | position offset | rotation offset |
|---|---|---:|---:|---:|---:|---|---|
| `pelvis` | `Pelvis` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `torso_link` | `Chest` | `0` | 0 / — | `100` | 0 / 1 | equal | equal |
| `left_hip_yaw_link` | `Left_UpperLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `left_knee_link` | `Left_LowerLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `left_ankle_roll_link` | `Left_Foot` | `50` | 0 / 1 | `10` | 0 / 1 | equal | equal |
| `right_hip_yaw_link` | `Right_UpperLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `right_knee_link` | `Right_LowerLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `right_ankle_roll_link` | `Right_Foot` | `50` | 0 / 1 | `10` | 0 / 1 | equal | equal |
| `left_shoulder_yaw_link` | `Left_UpperArm` | `0` | 0 / — | `100` | 0 / 1 | equal | equal |
| `left_elbow_link` | `Left_Forearm` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `left_wrist_yaw_link` | `Left_Hand` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `right_shoulder_yaw_link` | `Right_UpperArm` | `0` | 0 / — | `100` | 0 / 1 | equal | equal |
| `right_elbow_link` | `Right_Forearm` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `right_wrist_yaw_link` | `Right_Hand` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |

### `ik_match_table2`

| Robot body | Human body | pos base→jump | pos Δ/ratio | rot base→jump | rot Δ/ratio | position offset | rotation offset |
|---|---|---:|---:|---:|---:|---|---|
| `pelvis` | `Pelvis` | `100 → 150` | 50 / 1.5 | `5` | 0 / 1 | equal | equal |
| `left_hip_yaw_link` | `Left_UpperLeg` | `10 → 15` | 5 / 1.5 | `5` | 0 / 1 | equal | equal |
| `left_knee_link` | `Left_LowerLeg` | `10 → 20` | 10 / 2 | `5` | 0 / 1 | equal | equal |
| `left_ankle_roll_link` | `Left_Foot` | `50 → 80` | 30 / 1.6 | `10 → 15` | 5 / 1.5 | equal | equal |
| `right_hip_yaw_link` | `Right_UpperLeg` | `10 → 15` | 5 / 1.5 | `5` | 0 / 1 | equal | equal |
| `right_knee_link` | `Right_LowerLeg` | `10 → 20` | 10 / 2 | `5` | 0 / 1 | equal | equal |
| `right_ankle_roll_link` | `Right_Foot` | `50 → 80` | 30 / 1.6 | `10 → 15` | 5 / 1.5 | equal | equal |
| `torso_link` | `Chest` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `left_shoulder_yaw_link` | `Left_UpperArm` | `5` | 0 / 1 | `15` | 0 / 1 | equal | equal |
| `left_elbow_link` | `Left_Forearm` | `20` | 0 / 1 | `5` | 0 / 1 | equal | equal |
| `left_wrist_yaw_link` | `Left_Hand` | `50` | 0 / 1 | `5` | 0 / 1 | equal | equal |
| `right_shoulder_yaw_link` | `Right_UpperArm` | `5` | 0 / 1 | `50` | 0 / 1 | equal | equal |
| `right_elbow_link` | `Right_Forearm` | `20` | 0 / 1 | `5` | 0 / 1 | equal | equal |
| `right_wrist_yaw_link` | `Right_Hand` | `50` | 0 / 1 | `5` | 0 / 1 | equal | equal |
## E1

- base: `/home/weili/GMR-CPP_e1jump_lowdpi/config/ik_configs/xsens_to_e1.json`
- jump: `/home/weili/GMR-CPP_e1jump_lowdpi/config/ik_configs/xsens_to_e1_jump.json`

### 顶层字段

| 字段 | base → jump | 是否改变 |
|---|---:|---|
| `robot_root_name` | `base_link` | 否 |
| `human_root_name` | `Pelvis` | 否 |
| `ground_height` | `0.0` | 否 |
| `human_height_assumption` | `1.8` | 否 |
| `use_ik_match_table1` | `True` | 否 |
| `use_ik_match_table2` | `True` | 否 |

### Human scale

| Human body | base → jump | Δ | 比例 |
|---|---:|---:|---:|
| `Pelvis` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Chest` | `0.8 → 0.85` | 0.05 | 1.0625 |
| `Left_UpperLeg` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Right_UpperLeg` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Left_LowerLeg` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Right_LowerLeg` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Left_Foot` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Right_Foot` | `0.8 → 0.9` | 0.1 | 1.125 |
| `Left_UpperArm` | `0.35` | 0 | 1 |
| `Right_UpperArm` | `0.35` | 0 | 1 |
| `Left_Forearm` | `0.45` | 0 | 1 |
| `Right_Forearm` | `0.45` | 0 | 1 |
| `Left_Hand` | `0.45` | 0 | 1 |
| `Right_Hand` | `0.45` | 0 | 1 |

### `ik_match_table1`

| Robot body | Human body | pos base→jump | pos Δ/ratio | rot base→jump | rot Δ/ratio | position offset | rotation offset |
|---|---|---:|---:|---:|---:|---|---|
| `base_link` | `Pelvis` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `waist_roll_link` | `Chest` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `l_leg_hip_pitch_link` | `Left_UpperLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `l_leg_knee_link` | `Left_LowerLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `l_leg_ankle_roll_link` | `Left_Foot` | `100` | 0 / 1 | `50` | 0 / 1 | equal | equal |
| `r_leg_hip_pitch_link` | `Right_UpperLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `r_leg_knee_link` | `Right_LowerLeg` | `0` | 0 / — | `10` | 0 / 1 | equal | equal |
| `r_leg_ankle_roll_link` | `Right_Foot` | `100` | 0 / 1 | `50` | 0 / 1 | equal | equal |
| `l_arm_shoulder_roll_link` | `Left_UpperArm` | `0` | 0 / — | `10 → 4` | -6 / 0.4 | equal | equal |
| `l_arm_elbow_pitch_link` | `Left_Forearm` | `0` | 0 / — | `10 → 3` | -7 / 0.3 | equal | equal |
| `l_arm_elbow_yaw_link` | `Left_Hand` | `0` | 0 / — | `10 → 3` | -7 / 0.3 | equal | equal |
| `r_arm_shoulder_roll_link` | `Right_UpperArm` | `0` | 0 / — | `10 → 4` | -6 / 0.4 | equal | equal |
| `r_arm_elbow_pitch_link` | `Right_Forearm` | `0` | 0 / — | `10 → 3` | -7 / 0.3 | equal | equal |
| `r_arm_elbow_yaw_link` | `Right_Hand` | `0` | 0 / — | `10 → 3` | -7 / 0.3 | equal | equal |

### `ik_match_table2`

| Robot body | Human body | pos base→jump | pos Δ/ratio | rot base→jump | rot Δ/ratio | position offset | rotation offset |
|---|---|---:|---:|---:|---:|---|---|
| `base_link` | `Pelvis` | `10 → 100` | 90 / 10 | `5` | 0 / 1 | equal | equal |
| `waist_roll_link` | `Chest` | `10` | 0 / 1 | `5` | 0 / 1 | equal | equal |
| `l_leg_hip_pitch_link` | `Left_UpperLeg` | `10 → 20` | 10 / 2 | `5` | 0 / 1 | equal | equal |
| `l_leg_knee_link` | `Left_LowerLeg` | `10 → 40` | 30 / 4 | `5 → 8` | 3 / 1.6 | equal | equal |
| `l_leg_ankle_roll_link` | `Left_Foot` | `100 → 120` | 20 / 1.2 | `50` | 0 / 1 | equal | equal |
| `r_leg_hip_pitch_link` | `Right_UpperLeg` | `10 → 20` | 10 / 2 | `5` | 0 / 1 | equal | equal |
| `r_leg_knee_link` | `Right_LowerLeg` | `10 → 40` | 30 / 4 | `5 → 8` | 3 / 1.6 | equal | equal |
| `r_leg_ankle_roll_link` | `Right_Foot` | `100 → 120` | 20 / 1.2 | `50` | 0 / 1 | equal | equal |
| `l_arm_shoulder_roll_link` | `Left_UpperArm` | `10 → 2` | -8 / 0.2 | `5 → 2` | -3 / 0.4 | equal | equal |
| `l_arm_elbow_pitch_link` | `Left_Forearm` | `10 → 2` | -8 / 0.2 | `5 → 2` | -3 / 0.4 | equal | equal |
| `l_arm_elbow_yaw_link` | `Left_Hand` | `10 → 2` | -8 / 0.2 | `5 → 2` | -3 / 0.4 | equal | equal |
| `r_arm_shoulder_roll_link` | `Right_UpperArm` | `10 → 2` | -8 / 0.2 | `5 → 2` | -3 / 0.4 | equal | equal |
| `r_arm_elbow_pitch_link` | `Right_Forearm` | `10 → 2` | -8 / 0.2 | `5 → 2` | -3 / 0.4 | equal | equal |
| `r_arm_elbow_yaw_link` | `Right_Hand` | `10 → 2` | -8 / 0.2 | `5 → 2` | -3 / 0.4 | equal | equal |

## G1/E1 交叉规律

| 范围 | Human body | 字段 | G1 Δ/ratio | E1 Δ/ratio | 同向 | 分类 |
|---|---|---|---:|---:|---|---|
| `human_scale` | `Chest` | `scale` | 0.05 / 1.0625 | 0.05 / 1.0625 | 是 | `common-exact` |
| `human_scale` | `Left_Foot` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `human_scale` | `Left_LowerLeg` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `human_scale` | `Left_UpperLeg` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `human_scale` | `Pelvis` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `human_scale` | `Right_Foot` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `human_scale` | `Right_LowerLeg` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `human_scale` | `Right_UpperLeg` | `scale` | 0.15 / 1.1875 | 0.1 / 1.125 | 是 | `common-direction-e1-ratio` |
| `ik_match_table1` | `Left_Forearm` | `rotation_weight` | 0 / 1 | -7 / 0.3 | 否 | `robot-specific-e1` |
| `ik_match_table1` | `Left_Hand` | `rotation_weight` | 0 / 1 | -7 / 0.3 | 否 | `robot-specific-e1` |
| `ik_match_table1` | `Left_UpperArm` | `rotation_weight` | 0 / 1 | -6 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table1` | `Right_Forearm` | `rotation_weight` | 0 / 1 | -7 / 0.3 | 否 | `robot-specific-e1` |
| `ik_match_table1` | `Right_Hand` | `rotation_weight` | 0 / 1 | -7 / 0.3 | 否 | `robot-specific-e1` |
| `ik_match_table1` | `Right_UpperArm` | `rotation_weight` | 0 / 1 | -6 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_Foot` | `position_weight` | 30 / 1.6 | 20 / 1.2 | 是 | `common-direction-e1-ratio` |
| `ik_match_table2` | `Left_Foot` | `rotation_weight` | 5 / 1.5 | 0 / 1 | 否 | `robot-specific-g1` |
| `ik_match_table2` | `Left_Forearm` | `position_weight` | 0 / 1 | -8 / 0.2 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_Forearm` | `rotation_weight` | 0 / 1 | -3 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_Hand` | `position_weight` | 0 / 1 | -8 / 0.2 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_Hand` | `rotation_weight` | 0 / 1 | -3 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_LowerLeg` | `position_weight` | 10 / 2 | 30 / 4 | 是 | `common-direction-e1-ratio` |
| `ik_match_table2` | `Left_LowerLeg` | `rotation_weight` | 0 / 1 | 3 / 1.6 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_UpperArm` | `position_weight` | 0 / 1 | -8 / 0.2 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_UpperArm` | `rotation_weight` | 0 / 1 | -3 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Left_UpperLeg` | `position_weight` | 5 / 1.5 | 10 / 2 | 是 | `common-direction-e1-ratio` |
| `ik_match_table2` | `Pelvis` | `position_weight` | 50 / 1.5 | 90 / 10 | 是 | `common-direction-e1-ratio` |
| `ik_match_table2` | `Right_Foot` | `position_weight` | 30 / 1.6 | 20 / 1.2 | 是 | `common-direction-e1-ratio` |
| `ik_match_table2` | `Right_Foot` | `rotation_weight` | 5 / 1.5 | 0 / 1 | 否 | `robot-specific-g1` |
| `ik_match_table2` | `Right_Forearm` | `position_weight` | 0 / 1 | -8 / 0.2 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_Forearm` | `rotation_weight` | 0 / 1 | -3 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_Hand` | `position_weight` | 0 / 1 | -8 / 0.2 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_Hand` | `rotation_weight` | 0 / 1 | -3 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_LowerLeg` | `position_weight` | 10 / 2 | 30 / 4 | 是 | `common-direction-e1-ratio` |
| `ik_match_table2` | `Right_LowerLeg` | `rotation_weight` | 0 / 1 | 3 / 1.6 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_UpperArm` | `position_weight` | 0 / 1 | -8 / 0.2 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_UpperArm` | `rotation_weight` | 0 / 1 | -3 / 0.4 | 否 | `robot-specific-e1` |
| `ik_match_table2` | `Right_UpperLeg` | `position_weight` | 5 / 1.5 | 10 / 2 | 是 | `common-direction-e1-ratio` |
