# SMPL-X → E1 manual-v3 固定配置说明

本配置只包含人工指定的常量。运行时不会搜索 quaternion、修改权重、写回
JSON、执行在线标定或增加 foot/contact 修正。人体侧继续使用既有 SMP1：
14 个原版小写 SMPL-X joint center 和 global FK rotation。

本机没有 `/home/weili/GMR` 仓库，因此
`scripts/compare_smplx_e1_configs.py` 会明确提示并使用当前已经验证的本地
`config/ik_configs/smplx_to_g1.json`。这两个 JSON 的角色相同，但报告不会把
本地副本冒充成不存在的仓库文件。

## 当前配置与 XRobot E1 的差异

表中权重格式为 `table1 position/rotation ; table2 position/rotation`。
XRobot human name 仅大小写不同，robot body mapping 与 manual-v3 保持一致。

| human | robot body | 当前 smplx_to_e1 权重 | XRobot 权重 | 当前 rotation offset | XRobot rotation offset |
|---|---|---:|---:|---|---|
| pelvis | base_link | 0/10 ; 10/5 | 0/10 ; 10/5 | `[.5,-.5,-.5,-.5]` | `[-.5,.5,-.5,-.5]` |
| spine3 | waist_roll_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.50035,-.49965,-.50035,-.49965]` | `[-.5,.5,-.5,-.5]` |
| left_hip | l_leg_hip_pitch_link | 0/10 ; 10/5 | 0/10 ; 10/5 | `[.47439,-.52436,-.52436,-.47439]` | `[-.56379,.42678,-.42678,-.56379]` |
| right_hip | r_leg_hip_pitch_link | 0/10 ; 10/5 | 0/10 ; 10/5 | `[.47439,-.52436,-.52436,-.47439]` | `[-.56379,.42678,-.42678,-.56379]` |
| left_knee | l_leg_knee_link | 0/10 ; 10/5 | 0/10 ; 10/5 | `[.5,-.5,-.5,-.5]` | `[-.5,.5,-.5,-.5]` |
| right_knee | r_leg_knee_link | 0/10 ; 10/5 | 0/10 ; 10/5 | `[.5,-.5,-.5,-.5]` | `[-.5,.5,-.5,-.5]` |
| left_foot | l_leg_ankle_roll_link | 100/50 ; 100/50 | 100/10 ; 100/10 | `[.5,-.5,-.5,-.5]` | `[-.5,.5,-.5,-.5]` |
| right_foot | r_leg_ankle_roll_link | 100/50 ; 100/50 | 100/10 ; 100/10 | `[.5,-.5,-.5,-.5]` | `[-.5,.5,-.5,-.5]` |
| left_shoulder | l_arm_shoulder_roll_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.70712,.00090,-.70709,.00104]` | `[.70711,0,.70711,0]` |
| left_elbow | l_arm_elbow_pitch_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.99899,.04498,-.00002,.00010]` | `[.70711,0,.70711,0]` |
| left_wrist | l_arm_elbow_yaw_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.99899,.04498,-.00002,.00010]` | `[.70711,0,.70711,0]` |
| right_shoulder | r_arm_shoulder_roll_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.00005,.70709,.00009,.70712]` | `[0,.70711,0,-.70711]` |
| right_elbow | r_arm_elbow_pitch_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.00010,.00002,-.04358,.99905]` | `[0,.70711,0,-.70711]` |
| right_wrist | r_arm_elbow_yaw_link | 0/10 ; 10/5 | 0/10 ; 0/10 | `[.00010,.00002,-.04358,.99905]` | `[0,.70711,0,-.70711]` |

所有 position offset 在 manual-v3 中固定为 `[0,0,0]`。完整逐项报告可运行：

```bash
python3 scripts/compare_smplx_e1_configs.py
```

## 手臂 quaternion 与截图现象

当前左肩接近 `[+0.7071,0,-0.7071,0]`，XRobot 左肩、左肘、左腕使用
`[+0.7071,0,+0.7071,0]`。这不是 quaternion 整体乘 `-1` 的等价表示，
而是相反方向的 Y 轴 90° rotation。当前左肘又接近 identity，与截图中的
T-pose 左肘折回、手臂下垂时左肘向上翻相符。

当前右肩接近 `[0,+0.7071,0,+0.7071]`，XRobot 右肩、右肘、右腕使用
`[0,+0.7071,0,-0.7071]`；同样不是整体符号等价。manual-v3 因此固定使用
XRobot 的手臂 offsets，不对 Xsens、FBX、XRobot quaternion 求平均。

## Manual-v3 固定规则

- human names 和 scale 来自 `smplx_to_g1.json`：躯干/腿 `0.9`，手臂 `0.8`。
- E1 robot body mapping 与 rotation offsets 来自 `xrobot_to_e1.json`。
- position offsets 全部为零。
- 两张 IK table 均启用；程序只读取配置，不修改配置。

## 权重的经验依据

- Pelvis table2 position `10 → 30`：下蹲时加强 base 跟随 pelvis 下降，减少
  通过抬脚折中的倾向。
- Knee table2 position `10 → 20`：让下蹲更多由 pelvis 下降和 knee flexion
  完成。
- Foot rotation 固定为 `8`：SMPL foot 映射到 E1 ankle-roll，而不是 G1
  toe-link；降低 orientation 对 pelvis/knee position 的压制。
- Wrist rotation 固定为 table1 `1`、table2 `0`：E1 没有 wrist DOF，仅保留
  微弱的第一阶段朝向约束；table2 wrist position 保持 `30`。
- Shoulder/Elbow 使用 XRobot quaternion：针对当前手臂 frame 方向错误做
  固定替换，不进行在线搜索。

## 只读诊断

`--vis-smplx-frames` 只显示 manual-v3 target frame axes，并每 0.5 秒打印
E1 肩、肘、膝角度以及 base/foot body 高度。坐标轴颜色固定为 X 红、Y 绿、
Z 蓝。诊断代码不包含 JSON 写入、参数搜索或 offset/weight 更新。

## 固定姿态测试结果

运行：

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
/home/weili/GENMO/.venv/bin/python \
  scripts/test_smplx_to_e1_manual_v3.py
```

测试使用合成的 stand、T-pose、arms-down、左右抬臂、左右肘弯 45° 和
squat SMP1 包，实际经过 Reader、C++ IK 和 Redis。每个姿态均输出完整 E1
qpos、关键关节、触及的限位、position error 和 rotation error。

本次固定配置的只读规则报告为：

- PASS：T-pose 左右肩外展反向且对称，elbow pitch 接近零。
- PASS：arms-down 的 elbow pitch 接近零，左右前臂 body 均低于 shoulder。
- PASS：squat 左右 knee 增加弯曲，左右 foot body 未抬高。
- WARN：合成 45° 肘弯未使 E1 elbow pitch 明显向负方向变化。
- WARN：squat 的 base 实测只下降 `0.017 m`，没有达到测试的 `0.10 m` 阈值。

WARN 被保留用于后续人工判断；测试不会据此修改 JSON 或搜索参数。

## 编译与完整运行

```bash
cd /home/weili/GMR-CPP_e1jump_lowdpi
bash build.sh --clean

./run_smplx_e1_manual_v3.sh \
  --always \
  --vis \
  --vis-smplx-targets \
  --vis-smplx-frames
```

另开终端，从摄像头 2 发送原版 SMP1（参数名与 GENMO 当前 CLI 一致）：

```bash
cd /home/weili/GENMO
source .venv/bin/activate

CUDA_VISIBLE_DEVICES=0 python scripts/demo/demo_webcam.py \
  --camera_id 2 \
  --no_imgfeat \
  --gmr_host 127.0.0.1 \
  --gmr_port 7005 \
  --gmr_protocol smplx1 \
  --display
```

`--camera_id 2` 选择 `/dev/video2`，`--gmr_protocol smplx1` 选择 14-target
原版 SMP1，`--gmr_host/--gmr_port` 指向 E1 server，`--no_imgfeat` 使用无图像
特征推理路径，`--display` 打开 GENMO 主窗口。server 默认不启用
`--offset-to-ground`，也没有任何在线标定或动态 vertical/contact 修正。
