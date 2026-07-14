#!/usr/bin/env python3
"""
自动人体尺寸标定程序
====================
从 OptiTrack 原始骨骼数据自动计算人体各段长度，生成标定后的 IK config。

使用方法:
  1. 先启动 mocap server（--always 模式）
  2. 运行此程序:
    python3 auto_calibrate.py \
    --output-dir /home/user/Workspace/GMR/general_motion_retargeting/ik_configs/calibrated/ \
    --person-name alice

标定流程:
  人站立 T-pose 保持5秒 → 自动采集 → 计算骨骼长度 → 生成 json
"""

import redis
import json
import time
import argparse
import os
import numpy as np

# ── Redis 配置 ────────────────────────────────────────────────────────────────
REDIS_RAW_KEY = "mmocap_motion_frame_g1_raw_bones"

# ── 机器人各段长度（从 MuJoCo XML 测量，单位 m）──────────────────────────────
ROBOT_SEGMENT_LENGTHS = {
    "thigh":     0.3061,   # left_hip_roll_link → left_knee_link
    "shank":     0.3521,   # left_knee_link → left_toe_link
    "trunk":     0.0442,   # pelvis → torso_link
    "upper_arm": 0.0821,   # left_shoulder_yaw_link → left_elbow_link
    "forearm":   0.1843,   # left_elbow_link → left_wrist_yaw_link
}

# ── 需要采集的骨骼对（用于计算各段长度）────────────────────────────────────
SEGMENT_PAIRS = {
    "thigh":     ("LeftUpLeg",  "LeftLeg"),
    "shank":     ("LeftLeg",    "LeftToeBase"),
    "trunk":     ("Hips",       "Spine1"),
    "upper_arm": ("LeftArm",    "LeftForeArm"),
    "forearm":   ("LeftForeArm","LeftHand"),
}

# ── IK config 模板 ────────────────────────────────────────────────────────────
IK_CONFIG_TEMPLATE = {
    "robot_root_name": "pelvis",
    "human_root_name": "Hips",
    "ground_height": 0.0,
    "human_height_assumption": 1.8,
    "use_ik_match_table1": True,
    "use_ik_match_table2": True,
    "human_scale_table": {
        "Hips": 0.9, "Spine1": 0.9,
        "LeftUpLeg": 0.9, "RightUpLeg": 0.9,
        "LeftLeg": 0.9, "RightLeg": 0.9,
        "LeftToeBase": 0.9, "RightToeBase": 0.9,
        "LeftArm": 0.8, "RightArm": 0.8,
        "LeftForeArm": 0.8, "RightForeArm": 0.8,
        "LeftHand": 0.8, "RightHand": 0.8
    },
    "ik_match_table1": {
        "pelvis":               ["Hips",        0,   10, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_hip_roll_link":   ["LeftUpLeg",   0,   10, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_knee_link":       ["LeftLeg",     0,   10, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_toe_link":        ["LeftToeBase", 100, 50, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "right_hip_roll_link":  ["RightUpLeg",  0,   10, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "right_knee_link":      ["RightLeg",    0,   10, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "right_toe_link":       ["RightToeBase",100, 50, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "torso_link":           ["Spine1",      0,   10, [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_shoulder_yaw_link":  ["LeftArm",     0, 10, [0.0,0.0,0.0], [0.5,  0.5, -0.5, -0.5]],
        "left_elbow_link":         ["LeftForeArm", 0, 10, [0.0,0.0,0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "left_wrist_yaw_link":     ["LeftHand",    0, 10, [0.0,0.0,0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "right_shoulder_yaw_link": ["RightArm",    0, 10, [0.0,0.0,0.0], [0.5, -0.5,  0.5, -0.5]],
        "right_elbow_link":        ["RightForeArm",0, 10, [0.0,0.0,0.0], [0.0,  0.0, -0.70710678, 0.70710678]],
        "right_wrist_yaw_link":    ["RightHand",   0, 10, [0.0,0.0,0.0], [0.0,  0.0, -0.70710678, 0.70710678]],
    },
    "ik_match_table2": {
        "pelvis":               ["Hips",        10,  5,  [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_hip_roll_link":   ["LeftUpLeg",   10,  5,  [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_knee_link":       ["LeftLeg",     10,  5,  [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_toe_link":        ["LeftToeBase", 100, 50, [0.0,0.0,0.0], [-0.70710678, 0.0, 0.0, 0.70710678]],
        "right_hip_roll_link":  ["RightUpLeg",  10,  5,  [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "right_knee_link":      ["RightLeg",    10,  5,  [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "right_toe_link":       ["RightToeBase",100, 50, [0.0,0.0,0.0], [-0.70710678, 0.0, 0.0, 0.70710678]],
        "torso_link":           ["Spine1",      10,  5,  [0.0,0.0,0.0], [0.70710678, 0.0, 0.0,-0.70710678]],
        "left_shoulder_yaw_link":  ["LeftArm",     10, 5, [0.0,0.0,0.0], [0.5,  0.5, -0.5, -0.5]],
        "left_elbow_link":         ["LeftForeArm", 10, 5, [0.0,0.0,0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "left_wrist_yaw_link":     ["LeftHand",    10, 5, [0.0,0.0,0.0], [0.70710678, 0.70710678, 0.0, 0.0]],
        "right_shoulder_yaw_link": ["RightArm",    10, 5, [0.0,0.0,0.0], [0.5, -0.5,  0.5, -0.5]],
        "right_elbow_link":        ["RightForeArm",10, 5, [0.0,0.0,0.0], [0.0,  0.0, -0.70710678, 0.70710678]],
        "right_wrist_yaw_link":    ["RightHand",   10, 5, [0.0,0.0,0.0], [0.0,  0.0, -0.70710678, 0.70710678]],
    }
}


def read_raw_bones(r):
    """从 Redis 读取原始骨骼数据"""
    raw = r.get(REDIS_RAW_KEY)
    if raw is None:
        return None
    try:
        return json.loads(raw)
    except:
        return None


def bone_pos(bones, name):
    """从骨骼数据中提取位置"""
    if name not in bones:
        return None
    return np.array(bones[name][:3])  # [x, y, z]


def compute_segment_length(bones, bone_a, bone_b):
    """计算两个骨骼之间的距离"""
    pa = bone_pos(bones, bone_a)
    pb = bone_pos(bones, bone_b)
    if pa is None or pb is None:
        return None
    return float(np.linalg.norm(pa - pb))


def collect_samples(r, duration=5.0, sample_hz=60):
    """采集一段时间的骨骼数据"""
    samples = []
    dt = 1.0 / sample_hz
    n = int(duration * sample_hz)
    for i in range(n):
        t0 = time.time()
        bones = read_raw_bones(r)
        if bones is not None:
            samples.append(bones)
        elapsed = time.time() - t0
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)
        if (i+1) % sample_hz == 0:
            print(f"    已采集 {len(samples)} 帧...")
    return samples


def compute_human_lengths(samples):
    """从多帧数据计算各段平均长度（取中位数，更稳健）"""
    lengths = {seg: [] for seg in SEGMENT_PAIRS}

    for bones in samples:
        for seg, (bone_a, bone_b) in SEGMENT_PAIRS.items():
            d = compute_segment_length(bones, bone_a, bone_b)
            if d is not None and d > 0.05:  # 过滤掉明显错误的值
                lengths[seg].append(d)

    result = {}
    for seg, vals in lengths.items():
        if len(vals) == 0:
            print(f"  [警告] {seg} 没有有效数据！")
            return None
        result[seg] = float(np.median(vals))

    return result


def compute_scale_table(human_lengths):
    """计算 scale_table = robot_segment_length / human_segment_length"""
    r = ROBOT_SEGMENT_LENGTHS
    h = human_lengths

    thigh_scale     = r["thigh"]     / h["thigh"]
    shank_scale     = r["shank"]     / h["shank"]
    trunk_scale     = r["trunk"]     / h["trunk"]
    upper_arm_scale = r["upper_arm"] / h["upper_arm"]
    forearm_scale   = r["forearm"]   / h["forearm"]
    hips_scale      = (thigh_scale + shank_scale) / 2.0

    return {
        "Hips":         round(hips_scale,     4),
        "Spine1":       round(trunk_scale,     4),
        "LeftUpLeg":    round(thigh_scale,     4),
        "RightUpLeg":   round(thigh_scale,     4),
        "LeftLeg":      round(shank_scale,     4),
        "RightLeg":     round(shank_scale,     4),
        "LeftToeBase":  round(shank_scale,     4),
        "RightToeBase": round(shank_scale,     4),
        "LeftArm":      round(upper_arm_scale, 4),
        "RightArm":     round(upper_arm_scale, 4),
        "LeftForeArm":  round(forearm_scale,   4),
        "RightForeArm": round(forearm_scale,   4),
        "LeftHand":     round(forearm_scale,   4),
        "RightHand":    round(forearm_scale,   4),
    }


def main():
    parser = argparse.ArgumentParser(description='GMR 自动人体尺寸标定程序')
    parser.add_argument('--redis-host',  default='127.0.0.1')
    parser.add_argument('--redis-port',  type=int, default=6379)
    parser.add_argument('--output-dir',  default='/tmp/gmr_calibration/',
                        help='输出目录')
    parser.add_argument('--person-name', default='person',
                        help='标定人员名字')
    parser.add_argument('--duration',    type=float, default=5.0,
                        help='采集时长（秒，默认5秒）')
    args = parser.parse_args()

    print("=" * 60)
    print("  GMR 自动人体尺寸标定程序")
    print("=" * 60)
    print(f"  标定人员 : {args.person_name}")
    print(f"  输出目录 : {args.output_dir}")
    print(f"  采集时长 : {args.duration} 秒")
    print()

    # 连接 Redis
    r = redis.Redis(host=args.redis_host, port=args.redis_port)
    try:
        r.ping()
        print(f"  Redis 连接成功: {args.redis_host}:{args.redis_port}")
    except Exception as e:
        print(f"  [错误] Redis 连接失败: {e}")
        return

    # 检查 mocap server 是否在运行
    print(f"  检查原始骨骼数据 key: {REDIS_RAW_KEY}")
    bones = read_raw_bones(r)
    if bones is None:
        print("  [错误] 未检测到原始骨骼数据！")
        print("  请先启动 mocap server，确认 [Init] ready 出现后再运行此程序")
        return

    print(f"  ✓ 检测到 {len(bones)} 个骨骼")
    print()

    # 检查需要的骨骼是否都存在
    needed = set()
    for bone_a, bone_b in SEGMENT_PAIRS.values():
        needed.add(bone_a)
        needed.add(bone_b)
    missing = needed - set(bones.keys())
    if missing:
        print(f"  [错误] 缺少骨骼: {missing}")
        return

    # 开始采集
    print("=" * 60)
    print("  标定姿势：T-pose")
    print("  请站直，双臂水平展开，保持静止")
    print("=" * 60)
    print()

    for i in range(5, 0, -1):
        print(f"  {i} 秒后开始采集...")
        time.sleep(1.0)

    print(f"  开始采集（{args.duration} 秒）...")
    samples = collect_samples(r, duration=args.duration)

    if len(samples) < 10:
        print(f"  [错误] 采集到的帧数太少（{len(samples)}），请检查 mocap server")
        return

    print(f"  ✓ 采集完成，共 {len(samples)} 帧")
    print()

    # 计算各段长度
    human_lengths = compute_human_lengths(samples)
    if human_lengths is None:
        print("  [错误] 计算骨骼长度失败")
        return

    # 计算 scale
    scale_table = compute_scale_table(human_lengths)

    # 打印结果
    print("=" * 60)
    print("  测量结果")
    print("=" * 60)
    print(f"  {'骨骼段':<12} {'人体(cm)':>10} {'机器人(cm)':>12} {'scale':>10}")
    print("  " + "-" * 46)
    for seg, (bone_a, bone_b) in SEGMENT_PAIRS.items():
        human_len = human_lengths[seg]
        robot_len = ROBOT_SEGMENT_LENGTHS[seg]
        scale = robot_len / human_len
        print(f"  {seg:<12} {human_len*100:>10.1f} {robot_len*100:>12.1f} {scale:>10.4f}")

    print()
    print("  生成的 scale_table:")
    orig = IK_CONFIG_TEMPLATE['human_scale_table']
    print(f"  {'骨骼':<20} {'新scale':>10} {'原scale':>10} {'变化':>10}")
    print("  " + "-" * 52)
    for bone, new_s in scale_table.items():
        old_s = orig[bone]
        change = new_s - old_s
        print(f"  {bone:<20} {new_s:>10.4f} {old_s:>10.4f} {change:>+10.4f}")

    # 生成新 config
    new_config = json.loads(json.dumps(IK_CONFIG_TEMPLATE))
    new_config['human_scale_table'] = scale_table
    new_config['_calibration'] = {
        'person':    args.person_name,
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'method':    'auto_optitrack',
        'n_samples': len(samples),
        'human_measurements_cm': {k: round(v*100, 1) for k, v in human_lengths.items()},
        'robot_segment_lengths_cm': {k: round(v*100, 1) for k, v in ROBOT_SEGMENT_LENGTHS.items()},
    }

    os.makedirs(args.output_dir, exist_ok=True)
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    filename = f"fbx_to_g1_{args.person_name}_{timestamp}.json"
    output_path = os.path.join(args.output_dir, filename)

    with open(output_path, 'w') as f:
        json.dump(new_config, f, indent=4, ensure_ascii=False)

    print()
    print(f"  ✓ 已保存到: {output_path}")
    print()
    print("  使用方法:")
    print(f"  ./optitrack_mocap_server ... --ik-config {output_path}")
    print()


if __name__ == '__main__':
    main()