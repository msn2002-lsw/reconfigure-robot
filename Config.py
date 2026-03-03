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

# 机器人匍匐式三角步态运动参数
Tripod_Cyc_time = 6  # 周期时间，单位s
Tripod_Cyc_Hig = 65  # 抬腿高度，单位mm
Tripod_Cyc_Disp = 50  # 周期运动距离，单位mm

Diagnoal_Cyc_time = 3  # 对角步态周期时间，单位s
Diagonal_Pz_init = -195  # Z轴初始位置，单位mm
Diagnoal_Cyc_Hig = 20  # 抬腿高度，单位mm
Diagonal_Cyc_Disp = 50  # 对角步态周期运动距离，单位mm
Diagnoal_Px_init = 39  # 对角步态，X轴初始位置，单位mm

Spine_init = 0  # 脊柱初始角度，单位°
Spine_Angle = 10  # 脊柱扭动角度，单位°

# 机器人站立式关节初始角度(单位：°)
init_angle_stand = {
    "RB": [RBQ2_init, RBQ3_init, RBQ4_init, RBQ5_init, RBQ6_init],
    "RF": [RFQ2_init, RFQ3_init, RFQ4_init, RFQ5_init, RFQ6_init],
    "LB": [LBQ2_init, LBQ3_init, LBQ4_init, LBQ5_init, LBQ6_init], 
    "LF": [LFQ2_init, LFQ3_init, LFQ4_init, LFQ5_init, LFQ6_init],
    "SPINE": [Spine_init]
}
# 机器人匍匐式关节初始角度(单位：°)
init_angle_crawl = {
    "RB": [J1_init, J2_init, J3_init, J4_init, J5_init],
    "RF": [J1_init, J2_init, J3_init, J4_init, J5_init],
    "LB": [J1_init, J2_init, J3_init, J4_init, J5_init], 
    "LF": [J1_init, J2_init, J3_init, J4_init, J5_init],
    "SPINE": [Spine_init]
}
# 机器人舵机id
id_group = {
    "RF": [1,2,3,4,5],
    "LF": [6,7,8,9,10],
    "RB": [11,12,13,14,15],
    "LB": [16,17,18,19,20],
    "SPINE": [21]
}
# 机器人匍匐式舵机初始角度
init_servo_angle_crawl = {
    "RF": [425, 630, 330, 445, 210],
    "LF": [420, 545, 557, 343, 280],
    "RB": [350, 640, 512, 435, 405],
    "LB": [465, 495, 500, 461, 457],
    "SPINE": [560]
}
# 机器人站立式舵机初始角度
init_servo_angle_stand = {
    "RF": [425, 255, 344, 80, 575],
    "LF": [420, 900, 557, 711, 663],
    "RB": [350, 280, 512, 835, 775],
    "LB": [465, 800, 500, 100, 850],
    "SPINE": [560]
}
servo_direction_factors = {
    "RF": [-1,  1, 1, -1, -1],
    "LF": [ 1, -1, -1, 1, -1],
    "RB": [-1,  1, -1, 1, -1],
    "LB": [ 1, -1, 1, -1, -1],
    "SPINE": [1]
}