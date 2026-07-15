# Codex 修复任务：GEM → GMR-CPP → E1 姿态、地面与手臂不对应

## 仓库

- `/home/weili/GENMO`
- `/home/weili/GMR-CPP_e1jump_lowdpi`

不要 reset、checkout 或覆盖无关改动。先记录两个仓库的 `git status --short`。直接修改和测试，不要只给分析。

## 已确认的问题

### 1. SMPL joint frame 被当作 Xsens segment frame

当前 `GENMO/gem/gmr_udp_bridge.py` 使用：

```python
rotations_out = total_rotation @ rotations_ay
```

而 GMR 直接加载 `config/ik_configs/xsens_to_e1.json`。该 JSON 的旋转偏置针对 Xsens segment frame，不针对 SMPL joint frame。

当前 AY→Z-up 矩阵：

```python
C = [[1,0,0],[0,0,-1],[0,1,0]]
```

等于 Rx(+90°)。当 SMPL 中性旋转为 I 时，当前桥接输出 C。GMR 又执行：

```text
target_rotation = human_rotation * rotation_offset
```

所以 root/chest/legs（offset=I）被施加 Rx(+90°)，左臂（offset=Rx(+90°)）约变成 Rx(180°)，右臂（offset=Rx(-90°)）约变成 I。这与实际“身体入地、左臂上举、右臂平举”一致。

### 2. 把 SMPL ankle 当成地面

当前 packet 的 `Left_Foot/Right_Foot` 使用 joint 7/8，这是合理的 ankle target；但初始化 ground 也用了 7/8。GENMO `EnDecoder` 已明确 7/8 是 ankle、10/11 是 foot。应使用 10/11 定义初始脚底地面，同时仍发送 7/8 给 E1 ankle-roll target。

E1 ankle-roll 下方 foot box 中心 z=-0.048、半高 0.013；若 ankle target z=0，脚底约在 -0.061 m。当前 GEM server 又默认关闭 per-frame offset-to-ground，因此会入地。

## 必须修改

### A. `GENMO/gem/gmr_udp_bridge.py`

1. 保留 payload 映射：

```python
JOINT_INDICES = (0, 9, 1, 2, 4, 5, 7, 8, 16, 17, 18, 19, 20, 21)
GROUND_JOINT_INDICES = (10, 11)
```

2. 分离 world 轴转换和 local frame 转换：

```python
C = self._axis_convert
rotations_zup = np.einsum("ij,njk,kl->nil", C, rotations, C.T)
```

3. 初始 yaw 从 `rotations_zup[0]` 提取，不再从 `C @ rotations[0]` 提取。

4. 位置：

```python
world_yaw = self._user_yaw @ self._yaw_inv
positions_out = np.einsum("ij,nj->ni", world_yaw @ C, joints)
```

5. 旋转：

```python
rotations_out = np.einsum("ij,njk->nik", world_yaw, rotations_zup)
```

6. 初始 ground：

```python
ground_z = min(
    positions_out[GROUND_JOINT_INDICES[0], 2],
    positions_out[GROUND_JOINT_INDICES[1], 2],
)
```

初始 origin 仍是 pelvis XY + ground Z；只初始化一次，保留跳跃。

7. 对每个 rotation 检查：

```text
R.T @ R ≈ I
det(R) ≈ +1
```

异常帧拒绝。

### B. 桥接单元测试

修复 `tests/test_gmr_udp_bridge.py`，新增并确保通过：

1. `rotations_ay=I`、yaw=0 时，输出 pelvis quaternion 必须为 `[1,0,0,0]`，不能只检查 norm。
2. 给 ankle 7/8 高度 0.10、foot 10/11 高度 0，初始化后发送的 Left_Foot/Right_Foot z 应约为 0.10（再乘 scale），不能为 0。
3. 左右 identity frame 输出必须对称。
4. packet 仍严格 412 bytes。

### C. 新建 GEM 专用 E1 配置

不要把 `xsens_to_e1.json` 直接作为最终旋转配置。

新建：

```text
config/ik_configs/gem_to_e1_position.json
```

要求：

- 机器人 body、human body、位置权重和 scale 先复用 `xsens_to_e1.json`；
- `use_ik_match_table1=false`；
- 两个表所有 `rotation_weight`（entry[2]）先设 0；
- 保留 table2 position weights；
- E1 foot position weight 保留；
- 不引用 G1。

`run_gem.sh` 初始调试默认改为该 position-only 配置，或提供明确 `--ik-config` 使用方式。

### D. 旋转标定

`calibrate_gem_rot_offsets.py` 的公式正确：

```text
q_offset = inverse(q_human_reference) * q_robot_neutral
```

但修复默认 Redis key 为：

```text
gmt_online_frame_e1
```

使用 position-only server，操作者保持与 E1 零位相符的直立、双臂自然下垂参考姿态，生成：

```text
config/ik_configs/gem_to_e1_calibrated.json
```

标定后不要立即恢复原 Xsens 权重。先：

- table1 保持关闭；
- table2 root/chest/arms/legs rotation weight 从 1 开始；
- feet rotation weight 从 2–5 开始；
- 确认稳定后逐步增加；
- 位置权重必须继续主导。

### E. E1 scale 脚本

当前 `scripts/calibrate_gem_scales.py` 写的是 G1 连杆长度和 G1 默认 Redis key，不能用于 E1。将其改成 E1 版本，或重命名保留 G1 并新增：

```text
scripts/calibrate_gem_e1_scales.py
```

E1 长度必须从 `assets/e1/mjcf/e1_24dof.xml` 的 body positions/neutral FK 计算，不能继续使用 G1 常量。

### F. 调试日志

在 GEM bridge 每 5 秒可选打印一次：

- pelvis/left ankle/right ankle/left foot/right foot 的 z；
- pelvis quaternion；
- left/right upper-arm quaternion。

在 GMR server 启动日志明确打印所用 IK 配置。

## 自动测试

```bash
cd /home/weili/GENMO
python3 -m unittest tests.test_gmr_udp_bridge
python3 -m py_compile gem/gmr_udp_bridge.py scripts/demo/demo_webcam.py

cd /home/weili/GMR-CPP_e1jump_lowdpi
bash build.sh --clean
```

检查 GEM 路径没有 G1：

```bash
grep -RIn \
  -e 'gem_to_g1' \
  -e 'unitree_g1' \
  -e 'mmocap_motion_frame_g1' \
  apps/gem_server.cpp run_gem.sh config/ik_configs/gem_to_e1*.json
```

## 运行验收

### 位置链路

```bash
./run_gem.sh \
  --ik-config "$PWD/config/ik_configs/gem_to_e1_position.json" \
  --always --vis
```

不启用 `--offset-to-ground`。正常直立时：

- E1 base upright；
- 脚底在地面上方而非穿透；
- 双臂自然下垂时 E1 双臂也下垂；
- 左右动作对应。

### 旋转标定

```bash
python3 scripts/calibrate_gem_rot_offsets.py \
  --xml assets/e1/mjcf/e1_24dof.xml \
  --base-config config/ik_configs/gem_to_e1_position.json \
  --output config/ik_configs/gem_to_e1_calibrated.json \
  --redis-key gmt_online_frame_e1 \
  --duration 3
```

再把 table2 rotation weights 从小值逐步开启。

## 最终输出

Codex 最后必须给出：

- 修改文件列表；
- root/arms/ground 修复说明；
- 测试结果；
- 无法自动进行的摄像头/MuJoCo人工验证；
- 完整运行命令；
- `git diff --stat` 和关键 diff。
