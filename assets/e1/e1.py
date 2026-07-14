# BSD 3-Clause License
# Copyright (c) 2025-2026, Beijing Noetix Robotics TECHNOLOGY CO.,LTD.
# All rights reserved.

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from NoetixE1.assets import ASSET_DIR

ARMATURE_4315 = 0.033048
ARMATURE_4340 = 0.032
ARMATURE_8112 = 0.04752756
ARMATURE_10020 = 0.068575968

NATURAL_FREQ = 4 * 2.0 * 3.1415926535  # 4Hz

STIFFNESS_4315 = ARMATURE_4315 * NATURAL_FREQ**2
STIFFNESS_4340 = ARMATURE_4340 * NATURAL_FREQ**2
STIFFNESS_8112 = ARMATURE_8112 * NATURAL_FREQ**2
STIFFNESS_10020 = ARMATURE_10020 * NATURAL_FREQ**2

DAMPING_4315 = 2.0 * ARMATURE_4315 * NATURAL_FREQ
DAMPING_4340 = 2.0 * ARMATURE_4340 * NATURAL_FREQ
DAMPING_8112 = 2.0 * ARMATURE_8112 * NATURAL_FREQ
DAMPING_10020 = 2.0 * ARMATURE_10020 * NATURAL_FREQ

E1_23DOF_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        asset_path=f"{ASSET_DIR}/robots/e1/urdf/e1_23dof.urdf",
        activate_contact_sensors=True,
        replace_cylinders_with_capsules=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.85),
        joint_pos={
            "l_leg_hip_yaw_joint": 0.0,
            "l_leg_hip_roll_joint": 0.0,
            "l_leg_hip_pitch_joint": -0.1495,
            "l_leg_knee_joint": 0.3215,
            "l_leg_ankle_pitch_joint": -0.1720,
            "l_leg_ankle_roll_joint": 0.0,
            "r_leg_hip_yaw_joint": 0.0,
            "r_leg_hip_roll_joint": 0.0,
            "r_leg_hip_pitch_joint": -0.1495,
            "r_leg_knee_joint": 0.3215,
            "r_leg_ankle_pitch_joint": -0.1720,
            "r_leg_ankle_roll_joint": 0.0,
            "waist_yaw_joint": 0.0,
            "l_arm_shoulder_pitch_joint": 0.0,
            "l_arm_shoulder_roll_joint": 0.2618,
            "l_arm_shoulder_yaw_joint": 0.0,
            "l_arm_elbow_pitch_joint": 0.0,
            "l_arm_elbow_yaw_joint": 0.0,
            "r_arm_shoulder_pitch_joint": 0.0,
            "r_arm_shoulder_roll_joint": -0.2618,
            "r_arm_shoulder_yaw_joint": 0.0,
            "r_arm_elbow_pitch_joint": 0.0,
            "r_arm_elbow_yaw_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 80.0,
                ".*_hip_roll_joint": 80.0,
                ".*_hip_pitch_joint": 100.0,
                ".*_knee_joint": 100.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 10.0,
                ".*_hip_roll_joint": 10.0,
                ".*_hip_pitch_joint": 12.0,
                ".*_knee_joint": 12.0,
            },
            stiffness={
                ".*_hip_yaw_joint": STIFFNESS_8112 * 2,
                ".*_hip_roll_joint": STIFFNESS_8112 * 2,
                ".*_hip_pitch_joint": STIFFNESS_10020 * 2,
                ".*_knee_joint": STIFFNESS_10020 * 2,
            },
            damping={
                ".*_hip_yaw_joint": DAMPING_8112 * 2,
                ".*_hip_roll_joint": DAMPING_8112 * 2,
                ".*_hip_pitch_joint": DAMPING_10020 * 1.45,
                ".*_knee_joint": DAMPING_10020 * 1.45,
            },
            armature={
                ".*_hip_yaw_joint": ARMATURE_8112,
                ".*_hip_roll_joint": ARMATURE_8112,
                ".*_hip_pitch_joint": ARMATURE_10020,
                ".*_knee_joint": ARMATURE_10020,
            },
        ),
        "waist": ImplicitActuatorCfg(
            effort_limit_sim=60.0,
            velocity_limit_sim=12.0,
            joint_names_expr=["waist_yaw_joint"],
            stiffness=STIFFNESS_4315 * 2,
            damping=DAMPING_4315 * 2,
            armature=ARMATURE_4315,
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_ankle_pitch_joint",
                ".*_ankle_roll_joint",
            ],
            effort_limit_sim=70.0,
            velocity_limit_sim=10.0,
            stiffness=STIFFNESS_4315 * 1.05,
            damping=DAMPING_4315 * 1.05,
            armature=ARMATURE_4315,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_pitch_joint",
                ".*_elbow_yaw_joint",
            ],
            effort_limit_sim=20.0,
            velocity_limit_sim=10.0,
            stiffness={
                ".*_shoulder_pitch_joint": STIFFNESS_4340 * 2,
                ".*_shoulder_roll_joint": STIFFNESS_4340 * 2,
                ".*_shoulder_yaw_joint": STIFFNESS_4340 * 2,
                ".*_elbow_pitch_joint": STIFFNESS_4340 * 1.2,
                ".*_elbow_yaw_joint": STIFFNESS_4340 * 1.2,
            },
            damping={
                ".*_shoulder_pitch_joint": DAMPING_4340 * 2,
                ".*_shoulder_roll_joint": DAMPING_4340 * 2,
                ".*_shoulder_yaw_joint": DAMPING_4340 * 2,
                ".*_elbow_pitch_joint": DAMPING_4340 * 1.2,
                ".*_elbow_yaw_joint": DAMPING_4340 * 1.2,
            },
            armature=ARMATURE_4340,
        ),
    },
)

E1_23DOF_JOINT_NAMES = [
    "l_leg_hip_yaw_joint",
    "l_leg_hip_roll_joint",
    "l_leg_hip_pitch_joint",
    "l_leg_knee_joint",
    "l_leg_ankle_pitch_joint",
    "l_leg_ankle_roll_joint",
    "r_leg_hip_yaw_joint",
    "r_leg_hip_roll_joint",
    "r_leg_hip_pitch_joint",
    "r_leg_knee_joint",
    "r_leg_ankle_pitch_joint",
    "r_leg_ankle_roll_joint",
    "waist_yaw_joint",
    "l_arm_shoulder_pitch_joint",
    "l_arm_shoulder_roll_joint",
    "l_arm_shoulder_yaw_joint",
    "l_arm_elbow_pitch_joint",
    "l_arm_elbow_yaw_joint",
    "r_arm_shoulder_pitch_joint",
    "r_arm_shoulder_roll_joint",
    "r_arm_shoulder_yaw_joint",
    "r_arm_elbow_pitch_joint",
    "r_arm_elbow_yaw_joint",
]

E1_23DOF_ACTION_SCALE = {}
for a in E1_23DOF_CFG.actuators.values():
    e = a.effort_limit_sim
    s = a.stiffness
    names = a.joint_names_expr
    if not isinstance(e, dict):
        e = {n: e for n in names}
    if not isinstance(s, dict):
        s = {n: s for n in names}
    for n in names:
        if n in e and n in s and s[n]:
            E1_23DOF_ACTION_SCALE[n] = 0.25 * e[n] / s[n]

E1_24DOF_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        asset_path=f"{ASSET_DIR}/robots/e1/urdf/e1_24dof.urdf",
        activate_contact_sensors=True,
        replace_cylinders_with_capsules=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.85),
        joint_pos={
            "l_leg_hip_yaw_joint": 0.0,
            "l_leg_hip_roll_joint": 0.0,
            "l_leg_hip_pitch_joint": -0.1495,
            "l_leg_knee_joint": 0.3215,
            "l_leg_ankle_pitch_joint": -0.1720,
            "l_leg_ankle_roll_joint": 0.0,
            "r_leg_hip_yaw_joint": 0.0,
            "r_leg_hip_roll_joint": 0.0,
            "r_leg_hip_pitch_joint": -0.1495,
            "r_leg_knee_joint": 0.3215,
            "r_leg_ankle_pitch_joint": -0.1720,
            "r_leg_ankle_roll_joint": 0.0,
            "waist_yaw_joint": 0.0,
            "waist_roll_joint": 0.0,
            "l_arm_shoulder_pitch_joint": 0.0,
            "l_arm_shoulder_roll_joint": 0.2618,
            "l_arm_shoulder_yaw_joint": 0.0,
            "l_arm_elbow_pitch_joint": 0.0,
            "l_arm_elbow_yaw_joint": 0.0,
            "r_arm_shoulder_pitch_joint": 0.0,
            "r_arm_shoulder_roll_joint": -0.2618,
            "r_arm_shoulder_yaw_joint": 0.0,
            "r_arm_elbow_pitch_joint": 0.0,
            "r_arm_elbow_yaw_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim={
                ".*_hip_yaw_joint": 80.0,
                ".*_hip_roll_joint": 80.0,
                ".*_hip_pitch_joint": 100.0,
                ".*_knee_joint": 100.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 10.0,
                ".*_hip_roll_joint": 10.0,
                ".*_hip_pitch_joint": 12.0,
                ".*_knee_joint": 12.0,
            },
            stiffness={
                ".*_hip_yaw_joint": STIFFNESS_8112 * 2,
                ".*_hip_roll_joint": STIFFNESS_8112 * 2,
                ".*_hip_pitch_joint": STIFFNESS_10020 * 2,
                ".*_knee_joint": STIFFNESS_10020 * 2,
            },
            damping={
                ".*_hip_yaw_joint": DAMPING_8112 * 2,
                ".*_hip_roll_joint": DAMPING_8112 * 2,
                ".*_hip_pitch_joint": DAMPING_10020 * 1.45,
                ".*_knee_joint": DAMPING_10020 * 1.45,
            },
            armature={
                ".*_hip_yaw_joint": ARMATURE_8112,
                ".*_hip_roll_joint": ARMATURE_8112,
                ".*_hip_pitch_joint": ARMATURE_10020,
                ".*_knee_joint": ARMATURE_10020,
            },
        ),
        "waist": ImplicitActuatorCfg(
            effort_limit_sim=60.0,
            velocity_limit_sim=12.0,
            joint_names_expr=["waist_.*_joint"],
            stiffness=STIFFNESS_4315 * 2,
            damping=DAMPING_4315 * 2,
            armature=ARMATURE_4315,
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_ankle_pitch_joint",
                ".*_ankle_roll_joint",
            ],
            effort_limit_sim=70.0,
            velocity_limit_sim=10.0,
            stiffness=STIFFNESS_4315 * 1.05,
            damping=DAMPING_4315 * 1.05,
            armature=ARMATURE_4315,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_pitch_joint",
                ".*_elbow_yaw_joint",
            ],
            effort_limit_sim=20.0,
            velocity_limit_sim=10.0,
            stiffness={
                ".*_shoulder_pitch_joint": STIFFNESS_4340 * 2,
                ".*_shoulder_roll_joint": STIFFNESS_4340 * 2,
                ".*_shoulder_yaw_joint": STIFFNESS_4340 * 2,
                ".*_elbow_pitch_joint": STIFFNESS_4340 * 1.2,
                ".*_elbow_yaw_joint": STIFFNESS_4340 * 1.2,
            },
            damping={
                ".*_shoulder_pitch_joint": DAMPING_4340 * 2,
                ".*_shoulder_roll_joint": DAMPING_4340 * 2,
                ".*_shoulder_yaw_joint": DAMPING_4340 * 2,
                ".*_elbow_pitch_joint": DAMPING_4340 * 1.2,
                ".*_elbow_yaw_joint": DAMPING_4340 * 1.2,
            },
            armature=ARMATURE_4340,
        ),
    },
)

E1_24DOF_JOINT_NAMES = [
    "l_leg_hip_yaw_joint",
    "r_leg_hip_yaw_joint",
    "waist_yaw_joint",
    "l_leg_hip_roll_joint",
    "r_leg_hip_roll_joint",
    "waist_roll_joint",
    "l_leg_hip_pitch_joint",
    "r_leg_hip_pitch_joint",
    "l_arm_shoulder_pitch_joint",
    "r_arm_shoulder_pitch_joint",
    "l_leg_knee_joint",
    "r_leg_knee_joint",
    "l_arm_shoulder_roll_joint",
    "r_arm_shoulder_roll_joint",
    "l_leg_ankle_pitch_joint",
    "r_leg_ankle_pitch_joint",
    "l_arm_shoulder_yaw_joint",
    "r_arm_shoulder_yaw_joint",
    "l_leg_ankle_roll_joint",
    "r_leg_ankle_roll_joint",
    "l_arm_elbow_pitch_joint",
    "r_arm_elbow_pitch_joint",
    "l_arm_elbow_yaw_joint",
    "r_arm_elbow_yaw_joint",
]

E1_24DOF_ACTION_SCALE = {}
for a in E1_24DOF_CFG.actuators.values():
    e = a.effort_limit_sim
    s = a.stiffness
    names = a.joint_names_expr
    if not isinstance(e, dict):
        e = {n: e for n in names}
    if not isinstance(s, dict):
        s = {n: s for n in names}
    for n in names:
        if n in e and n in s and s[n]:
            E1_24DOF_ACTION_SCALE[n] = 0.25 * e[n] / s[n]
