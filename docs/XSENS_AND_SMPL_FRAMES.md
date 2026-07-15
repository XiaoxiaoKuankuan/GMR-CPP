# Xsens 与 SMPL-direct 坐标语义

## XsensReader 能确定的信息

`readers/xsens_reader.cpp` 接收 Xsens MVN Network Streamer 的 `MXTP02`
Type-02 quaternion pose datagram：

- header 为 24 bytes；
- 每个 segment 为 32 bytes：`int32 segment_id`、`float32 x y z`、
  `float32 qw qx qy qz`；
- 多字节字段和 float 位模式均按网络字节序（big-endian）读取；
- quaternion 输出顺序为 `wxyz`；
- Reader 按 Xsens 协议注释将输入解释为 Z-up、right-handed 世界坐标。

Reader 中的 segment ID/name table 是：

| ID | 名称 | MVN 部位注释 |
|---:|---|---|
| 1 | Pelvis | |
| 2 | Spine | L5 |
| 3 | Spine1 | L3 |
| 4 | Spine2 | T12 |
| 5 | Chest | T8 |
| 6 | Neck | |
| 7 | Head | |
| 8 | Right_Shoulder | |
| 9 | Right_UpperArm | |
| 10 | Right_Forearm | |
| 11 | Right_Hand | |
| 12 | Left_Shoulder | |
| 13 | Left_UpperArm | |
| 14 | Left_Forearm | |
| 15 | Left_Hand | |
| 16 | Right_UpperLeg | |
| 17 | Right_LowerLeg | |
| 18 | Right_Foot | |
| 19 | Right_Toe | |
| 20 | Left_UpperLeg | |
| 21 | Left_LowerLeg | |
| 22 | Left_Foot | |
| 23 | Left_Toe | |

第一帧用 Pelvis global quaternion 做 yaw normalization。令其旋转矩阵为
`R`：

```text
yaw = atan2(R[1,0], R[0,0])
Q_yaw_inv = RotZ(-yaw)
p' = Q_yaw_inv * p
R' = Q_yaw_inv * R
```

因此当前实现把 Pelvis local `+X` 在水平面的投影作为初始 heading。这个
yaw offset 只在第一帧捕获，随后固定。

## XsensReader 不能确定的信息

`xsens_reader.cpp` 没有定义每个 MVN segment 的 proprietary local
anatomical `x/y/z` axes，也没有：

- 从 joint positions 构造 segment frame；
- segment origin alpha/midpoint 表；
- SMPL joint local frame 到 Xsens segment local frame 的转换。

Reader 只是接收 MVN 已经计算好的 global segment position/quaternion。
因此不能仅凭 Reader 或 segment 名称推导出精确的 proprietary Xsens
local frame。

`config/ik_configs/xsens_to_e1.json` 中的 rotation offset 表示现有、已验证
Xsens global segment frame 到 E1 robot body frame 的固定转换。它不是
SMPL joint frame 到 Xsens frame 的定义，不能直接复制给 SMPL 输入。

## 单位注释不一致

Reader 注释写的是 position `cm -> m`，但当前常量 `CM_TO_M=1.0`，实际没有
乘 `0.01`。现有 Xsens 链路保持该行为不变；是否需要换算必须结合实际 MVN
Network Streamer 输出设置验证，不能在本任务中凭注释修改。

## 独立 SMPL-direct / GEM2 语义

正式 SMPL 路径使用独立 GEM2 name table：

```text
SMPL_Pelvis, SMPL_Chest,
SMPL_LeftHip, SMPL_RightHip,
SMPL_LeftKnee, SMPL_RightKnee,
SMPL_LeftAnkle, SMPL_RightAnkle,
SMPL_LeftShoulder, SMPL_RightShoulder,
SMPL_LeftElbow, SMPL_RightElbow,
SMPL_LeftWrist, SMPL_RightWrist
```

每个 position 是对应 SMPL-X 22-joint landmark 的 joint center；rotation
是由相邻关节几何构造的 right-handed anatomical frame。它们不是 Xsens
segment，也不使用 `xsens_to_e1.json` 的 offsets。

GEM2 首个有效帧将人物几何 forward 对齐到世界 `+X`，世界约定为：

```text
+X human/E1 forward
+Y human left
+Z up
```

本仓库不提供 SMPL 到 Xsens 的标定或转换。不得将 SMPL direct frame 称为
Xsens-compatible，也不得把它直接送入 `xsens_to_e1.json`。
