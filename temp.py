import numpy as np
import Tool
import math
import time
import Config
from Gait import spine_plan
def Diagonal_gait_paln(t, t_span):
    """_summary_对角步态规划

    Args:
        t (_type_): _description_
        t_span (_type_): _description_

    Returns:
        _type_: _description_
    """

    if t > t_span:
        t = t_span

    Swing_disp = Config.Diagonal_Cyc_Disp / 2  # 摆动相摆动距离
    H_dis = Config.Diagnoal_Cyc_Hig  # 机器人抬腿高度，单位mm
    High_f = 0.5  # 抬腿时滞空因子

    RBPX_INIT = Config.Diagnoal_Px_init
    RBPY_INIT = -Swing_disp / 2
    RBPZ_INIT = Config.Diagonal_Pz_init

    RFPX_INIT = Config.Diagnoal_Px_init
    RFPY_INIT = Swing_disp / 2
    RFPZ_INIT = Config.Diagonal_Pz_init

    LFPX_INIT = Config.Diagnoal_Px_init
    LFPY_INIT = Swing_disp / 2
    LFPZ_INIT = Config.Diagonal_Pz_init

    LBPX_INIT = Config.Diagnoal_Px_init
    LBPY_INIT = -Swing_disp / 2
    LBPZ_INIT = Config.Diagonal_Pz_init

    T0 = t_span / 2

    RBPX = RBPX_INIT
    RFPX = RFPX_INIT
    LFPX = LFPX_INIT
    LBPX = LBPX_INIT
    if 0 <= t and t <= T0:

        RBPY = RBPY_INIT + Swing_disp * ((t / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi * t / T0))
        LFPY = LFPY_INIT - Swing_disp * ((t / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi * t / T0))

        if 0 <= t and t <= T0 * High_f:
            RBPZ = RBPZ_INIT + H_dis * (t / (T0 * High_f) - 1 / (2 * math.pi) * math.sin(2 * math.pi * t / (T0 * High_f)))
        elif (T0 * High_f < t and t <= T0 * (1 - High_f)):
            RBPZ = RBPZ_INIT + H_dis
        elif (T0 * (1 - High_f) < t and t <= T0):
            RBPZ = RBPZ_INIT + H_dis - H_dis * ((t - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                (2 * math.pi) * math.sin(2 * math.pi * (t - T0 * (1 - High_f)) / (T0 * High_f)))
        LFPZ = RBPZ

        RFPY = RFPY_INIT - Swing_disp / T0 * t
        RFPZ = RFPZ_INIT
        LBPY = LBPY_INIT + Swing_disp / T0 * t
        LBPZ = LBPZ_INIT

    elif T0 < t and t <= T0 * 2:

        RFPY = RFPY_INIT - Swing_disp + Swing_disp * (((t - T0) / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi *
                                                                                                    (t - T0) / T0))
        LBPY = LBPY_INIT + Swing_disp - Swing_disp * (((t - T0) / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi *
                                                                                                    (t - T0) / T0))

        if T0 <= t and t <= T0 + T0 * High_f:
            RFPZ = RFPZ_INIT + H_dis * ((t - T0) / (T0 * High_f) - 1 / (2 * math.pi) * math.sin(2 * math.pi * (t - T0) /
                                                                                                (T0 * High_f)))
        elif (T0 + T0 * High_f < t and t <= T0 + T0 * (1 - High_f)):
            RFPZ = RFPZ_INIT + H_dis
        elif (T0 + T0 * (1 - High_f) < t and t <= T0 * 2):
            RFPZ = RFPZ_INIT + H_dis - H_dis * (((t - T0) - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                (2 * math.pi) * math.sin(2 * math.pi * ((t - T0) - T0 * (1 - High_f)) /
                                                                        (T0 * High_f)))
        LBPZ = RFPZ

        RBPY = RBPY_INIT + Swing_disp - Swing_disp / T0 * (t - T0)
        LFPY = LFPY_INIT - Swing_disp + Swing_disp / T0 * (t - T0)
        RBPZ = RBPZ_INIT
        LFPZ = LFPZ_INIT

    [RBQ2, RBQ3, RBQ4, RBQ5, RBQ6] = Tool.SDH_Stand_inverse_solution(RBPX, RBPY, RBPZ)
    [RFQ2, RFQ3, RFQ4, RFQ5, RFQ6] = Tool.SDH_Stand_inverse_solution(RFPX, RFPY, RFPZ)
    [LFQ2, LFQ3, LFQ4, LFQ5, LFQ6] = Tool.SDH_Stand_inverse_solution(LFPX, LFPY, LFPZ)
    [LBQ2, LBQ3, LBQ4, LBQ5, LBQ6] = Tool.SDH_Stand_inverse_solution(LBPX, LBPY, LBPZ)

    print("LF:",round(LFPX, 3),round(LFPY, 3),round(LFPZ, 3))
    print("LF:", LFQ2, LFQ3, LFQ4, LFQ5, LFQ6)

    print("LB:",round(LBPX, 3),round(LBPY, 3),round(LBPZ, 3))
    print("LB:", LBQ2, LBQ3, LBQ4, LBQ5, LBQ6)

    print("RF:",round(RFPX, 3),round(RFPY, 3),round(RFPZ, 3))
    print("RF:", RFQ2, RFQ3, RFQ4, RFQ5, RFQ6)

    print("RB:",round(RBPX, 3),round(RBPY, 3),round(RBPZ, 3))
    print("RB:", RBQ2, RBQ3, RBQ4, RBQ5, RBQ6)


    Spine_J = 0  # 脊柱扭动角度为0
    Stand_Joint_group = {
        "LF": [LFQ2, LFQ3, LFQ4, LFQ5, LFQ6],
        "LB": [LBQ2, LBQ3, LBQ4, LBQ5, LBQ6],
        "RF": [RFQ2, RFQ3, RFQ4, RFQ5, RFQ6],
        "RB": [RBQ2, RBQ3, RBQ4, RBQ5, RBQ6],
        "SPINE": [Spine_J]
    }

    return Stand_Joint_group

def Tripod_gait_plan3(t):
    """_summary_机器人三角步态规划 2024 12 12 脊柱匀速扭动

    Args:
        t (_type_): _时间流_

    Returns:
        _type_: _Joint_group_
    """

    if t > Config.Tripod_Cyc_time:
        t = Config.Tripod_Cyc_time

    # 运动参数
    Tripod_Cyc_Disp = Config.Tripod_Cyc_Disp
    Tripod_Cyc_Hig = Config.Tripod_Cyc_Hig
    Tripod_Cyc_time = Config.Tripod_Cyc_time

    Swing_dis = Tripod_Cyc_Disp / 4 * 3
    T0 = Tripod_Cyc_time / 4  # 三角步态分为四个小周期，单位s
    V_R = Tripod_Cyc_Disp / Tripod_Cyc_time  # 机器人运动速度，单位mm/s
    H_dis = Tripod_Cyc_Hig  # 机器人抬腿高度，单位mm
    High_f = 0.5  # 抬腿时滞空因子

    # 左前腿初始位置
    LF_PX_init = Config.C_PX_init
    LF_PY_init = Config.C_PY_init + Swing_dis / 2
    LF_PZ_init = Config.C_PZ_init - H_dis / 2
    # 左后腿初始位置
    LB_PX_init = Config.C_PX_init
    LB_PY_init = Config.C_PY_init - Swing_dis / 2 + Swing_dis / 3
    LB_PZ_init = Config.C_PZ_init - H_dis / 2
    # 右前腿初始位置
    RF_PX_init = Config.C_PX_init
    RF_PY_init = Config.C_PY_init + Swing_dis / 2 - Swing_dis / 3 * 2
    RF_PZ_init = Config.C_PZ_init - H_dis / 2
    # 右后腿初始位置
    RB_PX_init = Config.C_PX_init
    RB_PY_init = Config.C_PY_init + Swing_dis / 2
    RB_PZ_init = Config.C_PZ_init - H_dis / 2

    # 脊柱规划
    Spine_J = spine_plan(t, Tripod_Cyc_time, Config.Spine_Angle)

    # 左前腿运动规划
    LFPX = LF_PX_init
    if 0 <= t and t <= T0:
        LFPY = LF_PY_init - Swing_dis * ((t / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi * t / T0))
        if (0 <= t and t <= T0 * High_f):
            LFPZ = LF_PZ_init + H_dis * (t / (T0 * High_f) - 1 / (2 * math.pi) * math.sin(2 * math.pi * t / (T0 * High_f)))
            # 左前腿关节解算
            [LFRJ1, LFRJ2, LFRJ3, LFRJ4, LFRJ5] = Tool.SDH_inverse_solution('LF', Spine_J, 90, LFPX, LFPY, LFPZ)
            # LFRJ2 = LFRJ2 + J2_A_Change * t / (T0 * High_f)
        elif (T0 * High_f < t and t <= T0 * (1 - High_f)):
            LFPZ = LF_PZ_init + H_dis
            # 左前腿关节解算
            [LFRJ1, LFRJ2, LFRJ3, LFRJ4, LFRJ5] = Tool.SDH_inverse_solution('LF', Spine_J, 90, LFPX, LFPY, LFPZ)
            # LFRJ2 = LFRJ2 + J2_A_Change
        elif (T0 * (1 - High_f) < t and t <= T0):
            LFPZ = LF_PZ_init + H_dis - H_dis * ((t - T0 * (1 - High_f)) / (T0 * High_f) - 1 /
                                                    (2 * math.pi) * math.sin(2 * math.pi * (t - T0 * (1 - High_f)) /
                                                                        (T0 * High_f)))
            # 左前腿关节解算
            [LFRJ1, LFRJ2, LFRJ3, LFRJ4, LFRJ5] = Tool.SDH_inverse_solution('LF', Spine_J, 90, LFPX, LFPY, LFPZ)
            # LFRJ2 = LFRJ2 + J2_A_Change - J2_A_Change * (t - T0 * (1 - High_f)) / (T0 * High_f)
    else:
        LFPY = LF_PY_init - Swing_dis + V_R * (t - T0)
        LFPZ = LF_PZ_init
        # 左前腿关节解算
        [LFRJ1, LFRJ2, LFRJ3, LFRJ4, LFRJ5] = Tool.SDH_inverse_solution('LF', Spine_J, 90, LFPX, LFPY, LFPZ)

    # 左后腿运动规划
    LBPX = LB_PX_init
    if 0 <= t and t <= T0:
        LBPY = LB_PY_init - V_R * t
        LBPZ = LB_PZ_init
        # 左后腿关节解算
        [LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5] = Tool.SDH_inverse_solution('LB', Spine_J, 90, LBPX, LBPY, LBPZ)
    elif T0 < t and t <= T0 * 2:
        LBPY = LB_PY_init - V_R * T0 + Swing_dis * (((t - T0) / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi * (t - T0) / T0))
        if T0 < t and t <= T0 + T0 * High_f:
            LBPZ = LB_PZ_init + H_dis * ((t - T0) / (T0 * High_f) - 1 / (2 * math.pi) * math.sin(2 * math.pi * (t - T0) /
                                                                                                (T0 * High_f)))
            # 左后腿关节解算
            [LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5] = Tool.SDH_inverse_solution('LB', Spine_J, 90, LBPX, LBPY, LBPZ)
            # LBRJ2 = LBRJ2 + J2_A_Change * (t - T0) / (T0 * High_f)
        elif (T0 + T0 * High_f < t and t <= T0 + T0 * (1 - High_f)):
            LBPZ = LB_PZ_init + H_dis
            # 左后腿关节解算
            [LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5] = Tool.SDH_inverse_solution('LB', Spine_J, 90, LBPX, LBPY, LBPZ)
            # LBRJ2 = LBRJ2 + J2_A_Change
        elif (T0 + T0 * (1 - High_f) < t and t <= T0 * 2):
            LBPZ = LB_PZ_init + H_dis - H_dis * ((t - (T0 + T0 * (1 - High_f))) / (T0 * High_f) - 1 /
                                                    (2 * math.pi) * math.sin(2 * math.pi * (t - (T0 + T0 * (1 - High_f))) /
                                                                        (T0 * High_f)))
            # 左后腿关节解算
            [LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5] = Tool.SDH_inverse_solution('LB', Spine_J, 90, LBPX, LBPY, LBPZ)
            # LBRJ2 = LBRJ2 + J2_A_Change - J2_A_Change * (t - (T0 + T0 * (1 - High_f))) / (T0 * High_f)
    else:
        LBPY = LB_PY_init - V_R * T0 + Swing_dis - V_R * (t - T0 * 2)
        LBPZ = LB_PZ_init
        # 左后腿关节解算
        [LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5] = Tool.SDH_inverse_solution('LB', Spine_J, 90, LBPX, LBPY, LBPZ)

    # 右前腿规划
    RFPX = RF_PX_init
    if 0 <= t and t <= T0 * 2:
        RFPY = RF_PY_init + V_R * t
        RFPZ = RF_PZ_init
        # 右前腿关节解算
        [RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5] = Tool.SDH_inverse_solution('RF', Spine_J, 90, RFPX, RFPY, RFPZ)
    elif T0 * 2 < t and t <= T0 * 3:
        RFPY = RF_PY_init + V_R * T0 * 2 - Swing_dis * (((t - T0 * 2) / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi *
                                                                                                            (t - T0 * 2) / T0))
        if (T0 * 2 < t and t <= T0 * 2 + T0 * High_f):
            RFPZ = RF_PZ_init + H_dis * ((t - T0 * 2) / (T0 * High_f) - 1 /
                                            (2 * math.pi) * math.sin(2 * math.pi * (t - T0 * 2) / (T0 * High_f)))
            # 右前腿关节解算
            [RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5] = Tool.SDH_inverse_solution('RF', Spine_J, 90, RFPX, RFPY, RFPZ)
            # RFRJ2 = RFRJ2 + J2_A_Change * (t - T0 * 2) / (T0 * High_f)
        elif (T0 * 2 + T0 * High_f < t and t <= T0 * 2 + T0 * (1 - High_f)):
            RFPZ = RF_PZ_init + H_dis
            # 右前腿关节解算
            [RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5] = Tool.SDH_inverse_solution('RF', Spine_J, 90, RFPX, RFPY, RFPZ)
            # RFRJ2 = RFRJ2 + J2_A_Change
        else:
            RFPZ = RF_PZ_init + H_dis - H_dis * ((t - (T0 * 2 + T0 * (1 - High_f))) / (T0 * High_f) - 1 /
                                                    (2 * math.pi) * math.sin(2 * math.pi * (t - (T0 * 2 + T0 * (1 - High_f))) /
                                                                        (T0 * High_f)))
            # 右前腿关节解算
            [RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5] = Tool.SDH_inverse_solution('RF', Spine_J, 90, RFPX, RFPY, RFPZ)
            # RFRJ2 = RFRJ2 + J2_A_Change - J2_A_Change * (t - (T0 * 2 + T0 * (1 - High_f))) / (T0 * High_f)
    else:
        RFPY = RF_PY_init + V_R * T0 * 2 - Swing_dis + V_R * (t - T0 * 3)
        RFPZ = RF_PZ_init
        # 右前腿关节解算
        [RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5] = Tool.SDH_inverse_solution('RF', Spine_J, 90, RFPX, RFPY, RFPZ)

    # 右后腿规划
    RBPX = RB_PX_init
    if 0 <= t and t <= T0 * 3:
        RBPY = RB_PY_init - V_R * t
        RBPZ = RB_PZ_init
        # 右后腿关节解算
        [RBRJ1, RBRJ2, RBRJ3, RBRJ4, RBRJ5] = Tool.SDH_inverse_solution('RB', Spine_J, 90, RBPX, RBPY, RBPZ)
    else:
        RBPY = RB_PY_init - V_R * T0 * 3 + Swing_dis * (((t - T0 * 3) / T0) - 1 / (2 * math.pi) * math.sin(2 * math.pi *
                                                                                                            (t - T0 * 3) / T0))
        if T0 * 3 < t and t <= T0 * 3 + T0 * High_f:
            RBPZ = RB_PZ_init + H_dis * ((t - T0 * 3) / (T0 * High_f) - 1 /
                                            (2 * math.pi) * math.sin(2 * math.pi * (t - T0 * 3) / (T0 * High_f)))
            # 右后腿关节解算
            [RBRJ1, RBRJ2, RBRJ3, RBRJ4, RBRJ5] = Tool.SDH_inverse_solution('RB', Spine_J, 90, RBPX, RBPY, RBPZ)
            # RBRJ2 = RBRJ2 + J2_A_Change * (t - T0 * 3) / (T0 * High_f)
        elif T0 * 3 + T0 * High_f < t and t <= T0 * 3 + T0 * (1 - High_f):
            RBPZ = RB_PZ_init + H_dis
            # 右后腿关节解算
            [RBRJ1, RBRJ2, RBRJ3, RBRJ4, RBRJ5] = Tool.SDH_inverse_solution('RB', Spine_J, 90, RBPX, RBPY, RBPZ)
            # RBRJ2 = RBRJ2 + J2_A_Change
        else:
            RBPZ = RB_PZ_init + H_dis - H_dis * ((t - (T0 * 3 + T0 * (1 - High_f))) / (T0 * High_f) - 1 /
                                                    (2 * math.pi) * math.sin(2 * math.pi * (t - (T0 * 3 + T0 * (1 - High_f))) /
                                                                        (T0 * High_f)))
            # 右后腿关节解算
            [RBRJ1, RBRJ2, RBRJ3, RBRJ4, RBRJ5] = Tool.SDH_inverse_solution('RB', Spine_J, 90, RBPX, RBPY, RBPZ)
            # RBJ2 = RBRJ2 + J2_A_Change - J2_A_Change * (t - (T0 * 3 + T0 * (1 - High_f))) / (T0 * High_f)

    print("RB",RBPX,RBPY,RBPZ)
    print("RF",RFPX,RFPY,RFPZ)
    print("LB",LBPX,LBPY,LBPZ)
    print("LF",LFPX,LFPY,LFPZ)
    print("RB",RBRJ1, RBRJ2, RBRJ3, RBRJ4, RBRJ5)
    print("RF",RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5)
    print("LB",LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5)
    print("LF",LFRJ1, LFRJ2, LFRJ3, LFRJ4, LFRJ5)

    joint_group = {
        "LF": [LFRJ1, LFRJ2, LFRJ3, LFRJ4, LFRJ5],
        "LB": [LBRJ1, LBRJ2, LBRJ3, LBRJ4, LBRJ5],
        "RF": [RFRJ1, RFRJ2, RFRJ3, RFRJ4, RFRJ5],
        "RB": [RBRJ1, RBRJ2, RBRJ3, RBRJ4, RBRJ5],
        "SPINE": [Spine_J]
    }
    # print(Joint_Group)
    return joint_group

def from_crawl_to_stand(pose_target, t):
    """_summary_从爬行姿态到站立姿态的过渡规划

    Args:
        t(int): _time_

    Returns:
        flag(bool): 是否顺利完成变姿
    """
    # for leg_name in Config.id_group:
    #     for i in range(len(Config.id_group[leg_name])):
    #         print(i)
    #         servo_info_target.append([Config.id_group[leg_name][i],Config.init_servo_angle_stand[leg_name][i]])
    # move_servo(servo_info_target, 4000, servo_tool)

    return True