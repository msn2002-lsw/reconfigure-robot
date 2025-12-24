from dataclasses import dataclass
@dataclass
class LegParam:
    """
    定义机器人腿部相位差和占空比参数类
    """
    name: str
    offset: float
    duty_factor: float


R_L1 = 105
R_L2 = 13
R_L3 = 39
R_L4 = 28
R_L5 = 49
R_L6 = 60
R_L7 = 60

C_PX_change = 7
C_PX_init = R_L2 + R_L3 + R_L4 + R_L5 - C_PX_change
C_PY_init = -R_L1 - R_L6
C_PZ_init = -R_L7

initPositionCrawl = {
    "LF": [C_PX_init, C_PY_init, C_PZ_init],
    "RF": [C_PX_init, C_PY_init, C_PZ_init],
    "LB": [C_PX_init, C_PY_init, C_PZ_init],
    "RB": [C_PX_init, C_PY_init, C_PZ_init]
}

# 机器人关节模型初始角度，单位°
Spine_init = 0
J1_init = 0
J2_init = 0
J3_init = 0
J4_init = -90
J5_init = 90
J6_init = -(J3_init + J4_init)

# 站立式机器人关节初始角度(以匍匐式状态为准，因为仿真和实验均是以匍匐式状态为准)
RBQ2_init = 0
RBQ3_init = 0
RBQ4_init = 0
RBQ5_init = -90
RBQ6_init = -90

RFQ2_init = 0
RFQ3_init = 0
RFQ4_init = 0
RFQ5_init = 90
RFQ6_init = -90

LFQ2_init = 0
LFQ3_init = 0
LFQ4_init = 0
LFQ5_init = -90
LFQ6_init = -90

LBQ2_init = 0
LBQ3_init = 0
LBQ4_init = 0
LBQ5_init = 90
LBQ6_init = -90

Spine_Angle = 20  # 脊柱扭动角度，单位°


