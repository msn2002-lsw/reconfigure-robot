import math
import numpy as np
import Config

def SDH_Stand_inverse_solution(PX, PY, PZ):
    """_summary_站立式机器人运动学逆解 (限制足端俯仰角、翻滚角以及三维坐标信息，共五个自由度，因此可以解出五个角度) 2024 12 12

    Args:
        SDH_name (str): _description_
        PX (_type_): _单位mm_
        PY (_type_): _单位mm_
        PZ (_type_): _单位mm_

    Returns:
        _type_: _Stand_Leg_Joint_Group = [Q2, Q3, Q4, Q5, Q6] 角度制_
    """

    # 此处逆解，已经限制了足端俯仰角、翻滚角，三维坐标信息，限制了五个自由度，因此可以解算出五个角度
    # Q4
    Q4 = math.asin(PY / Config.R_L5)

    # Q5
    Q5 = -Q4

    # Q3
    a = Config.R_L4 + Config.R_L6 + Config.R_L5 * math.cos(Q4)
    Q3 = -math.acos((math.pow(PX, 2) + math.pow(
        (PZ + Config.R_L7), 2) - (math.pow(Config.R_L3, 2) + math.pow(a, 2))) / (2 * Config.R_L3 * a))

    # Q6
    b = -Config.R_L3 * math.sin(Q3) - Config.R_L3 * math.cos(Q3) - a
    d = Config.R_L3 * math.sin(Q3) - Config.R_L3 * math.cos(Q3) - a
    Q6 = -(math.asin(
        (PX + PZ + Config.R_L7) / math.sqrt(math.pow(b, 2) + math.pow(d, 2))) + np.deg2rad(180)) - math.atan2(b, d)

    # Q2
    Q2 = np.deg2rad(-90) - Q3 - Q6

    # 整合数据
    Stand_Leg_Joint_Group = [Q2, Q3, Q4, Q5, Q6]
    Stand_Leg_Joint_Group = np.rad2deg(Stand_Leg_Joint_Group)  # 弧度制-->角度制
    # Stand_Leg_Joint_Group = np.round(Stand_Leg_Joint_Group, 3)  # 控制计算精度

    return Stand_Leg_Joint_Group

def SDH_Stand_Positive_solution(Q2, Q3, Q4, Q5, Q6):
    """_summary_站立式运动学模型正解 角度制 2024 12 12

    Args:
        SDH_name (str): _description_
        Q2 (_type_): _description_
        Q3 (_type_): _description_
        Q4 (_type_): _description_
        Q5 (_type_): _description_
        Q6 (_type_): _description_

    Returns:
        _type_: _位置信息_
        """

    # 角度转换：角度制-->弧度制
    Q2 = math.radians(Q2)
    Q3 = math.radians(Q3)
    Q4 = math.radians(Q4)
    Q5 = math.radians(Q5)
    Q6 = math.radians(Q6)

    # 站立式机器人运动学模型正解
    T01 = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    T12 = np.array([[math.cos(Q2), -math.sin(Q2), 0, Config.R_L3 * math.cos(Q2)],
                    [math.sin(Q2), math.cos(Q2), 0, Config.R_L3 * math.sin(Q2)], [0, 0, 1, 0], [0, 0, 0, 1]])
    T23 = np.array([[math.cos(Q3), 0, -math.sin(Q3), Config.R_L4 * math.cos(Q3)],
                    [math.sin(Q3), 0, math.cos(Q3), Config.R_L4 * math.sin(Q3)], [0, -1, 0, 0], [0, 0, 0, 1]])
    T34 = np.array([[math.cos(Q4), -math.sin(Q4), 0, Config.R_L5 * math.cos(Q4)],
                    [math.sin(Q4), math.cos(Q4), 0, Config.R_L5 * math.sin(Q4)], [0, 0, 1, 0], [0, 0, 0, 1]])
    T45 = np.array([[math.cos(Q5), 0, math.sin(Q5), Config.R_L6 * math.cos(Q5)],
                    [math.sin(Q5), 0, -math.cos(Q5), Config.R_L6 * math.sin(Q5)], [0, 1, 0, 0], [0, 0, 0, 1]])
    T56 = np.array([[math.cos(Q6), -math.sin(Q6), 0, Config.R_L7 * math.cos(Q6)],
                    [math.sin(Q6), math.cos(Q6), 0, Config.R_L7 * math.sin(Q6)], [0, 0, 1, 0], [0, 0, 0, 1]])
    T67 = np.array([[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

    # 传递矩阵计算
    T02 = np.dot(T01, T12)  # np.dot(A, B) 矩阵A左乘于矩阵B
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    T07 = np.dot(T06, T67)

    # 控制书出精度
    T07 = np.round(T07, 3)  # 可以通过np.round()函数控制矩阵元素精度

    P07 = np.array([T07[0][3], T07[1][3], T07[2][3]])  # 提取位置信息，构建位置矩阵

    return P07

def SDH_inverse_solution(SDH_name: str, Q4, Q9, PX, PY, PZ):
    """_机器人关节逆解_

    Args:
        SDH_name (str): _腿部模型名称_ 包含RB RF LB LF，例RB为Right Behind
        Q4 (_type_): _脊柱扭动关节。角度制_
        Q9 (_type_): _腿部俯仰关节，角度制_
        PX (_type_): _单位mm_
        PY (_type_): _单位mm_
        PZ (_type_): _单位mm_

    Returns:
        _type_: _腿部关节解算角度 Leg_Joint_Group = [Q5, Q6, Q7, Q8, Q10] 角度制_
    """
    # 单位转换，角度制-->弧度制
    Q4 = math.radians(Q4)  # Q4脊柱关节
    Q9 = math.radians(Q9)  # Q9腿部俯仰关节，由腿部关节确定

    # 判断腿部，选择对应脊柱角度符号
    if (SDH_name == 'RB' or SDH_name == 'RF'):
        Q4 = Q4
    elif (SDH_name == 'LB' or SDH_name == 'LF'):
        Q4 = -Q4
    else:
        print('SDH_name set fail. Try to detect the function about SDH_inverse_solution')

    # #腿部关节角度解算
    # Q5 Q6
    Q5 = math.asin((PZ + Config.R_L7 * math.sin(Q9)) / Config.R_L3)
    Q6 = -Q5
    # # test
    # print('------', PX, PY, PZ, Q4, Q9)

    # Q8
    a = Config.R_L3 / 2 * math.cos(Q4 + Q5) + (
        Config.R_L2 + Config.R_L4) * math.cos(Q4) + Config.R_L1 * math.sin(Q4) + Config.R_L3 / 2 * math.cos(Q4 - Q5)
    b = Config.R_L3 / 2 * math.sin(Q4 + Q5) + (
        Config.R_L2 + Config.R_L4) * math.sin(Q4) - Config.R_L1 * math.cos(Q4) + Config.R_L3 / 2 * math.sin(Q4 - Q5)
    c = Config.R_L6 + Config.R_L7 * math.cos(Q9)
    # # test
    # detect = ((math.pow((PX - a), 2) + math.pow(
    #     (PY - b), 2) - math.pow(Config.R_L5, 2) - math.pow(c, 2))) / (2 * Config.R_L5 * c)
    # print('------', detect)

    Q8 = -math.acos(((math.pow((PX - a), 2) + math.pow(
        (PY - b), 2) - math.pow(Config.R_L5, 2) - math.pow(c, 2))) / (2 * Config.R_L5 * c))

    # Q7
    d1 = Config.R_L5 + c * (math.cos(Q8) + math.sin(Q8))
    d2 = Config.R_L5 + c * (math.cos(Q8) - math.sin(Q8))
    Q7 = math.asin((PX - a + PY - b) / math.sqrt(math.pow(d1, 2) + math.pow(d2, 2))) - math.atan2(d1, d2) - Q4

    # # Q10，被动踝关节角度计算-->仅仅用于仿真
    # Q10 = -(Q4 + Q7 + Q8)

    # 整合数据
    Leg_Joint_Group = [Q5, Q6, Q7, Q8, Q9]
    Leg_Joint_Group = np.rad2deg(Leg_Joint_Group)  # 弧度制-->角度制
    # Leg_Joint_Group = np.round(Leg_Joint_Group, 7)  # 控制结果输出精度

    return Leg_Joint_Group

def SDH_positive_solution(SDH_name: str, Q4, Q5, Q6, Q7, Q8, Q9):
    """_机器人全向运动学模型正解_

    Args:
        SDH_name (str): _腿部模型名称_ 包含RB RF LB LF,例RB为Right Behind
        Q1 (_角度制_): _偏航角_
        Q2 (_角度制_): _俯仰角_
        Q3 (_角度制_): _翻滚角_
        Q4 (_角度制_): _脊柱摆动关节_
        Q5 (_角度制_): _description_
        Q6 (_角度制_): _description_
        Q7 (_角度制_): _description_
        Q8 (_角度制_): _description_
        Q9 (_角度制_): _description_

    Returns:
        _矩阵_: _足部相较于脊柱躯干中心的位置信息_
    """

    # 单位转换，角度制-->弧度制
    Q4 = math.radians(Q4)  # Q4脊柱关节
    Q5 = math.radians(Q5)
    Q6 = math.radians(Q6)
    Q7 = math.radians(Q7)
    Q8 = math.radians(Q8)
    Q9 = math.radians(Q9)  # Q4-Q9 分别代表机器人脊柱关节及腿部关节

    # 判断腿部，选择对应脊柱角度符号
    if (SDH_name == 'RB' or SDH_name == 'RF'):
        Q4 = Q4
    elif (SDH_name == 'LB' or SDH_name == 'LF'):
        Q4 = -Q4
    else:
        print('SDH_name set fail. Try to detect the function about SDH_positive_solution')

    # #构建SDH传递矩阵
    # # 姿态角传递矩阵
    # T01 = np.array([[math.cos(Q1), 0, math.sin(Q1), 0], [math.sin(Q1), 0, -math.cos(Q1), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    # T12 = np.array([[math.cos(Q2), 0, math.sin(Q2), 0], [math.sin(Q2), 0, -math.cos(Q2), 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    # T23 = np.array([[math.cos(Q3), 0, math.sin(Q3), 0], [math.sin(Q3), 0, -math.cos(Q3), 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    # 机器人关节传递矩阵
    T34 = np.array([[math.cos(Q4), 0, math.sin(Q4), Config.R_L2 * math.cos(Q4)],
                    [math.sin(Q4), 0, -math.cos(Q4), Config.R_L2 * math.sin(Q4)], [0, 1, 0, 0], [0, 0, 0, 1]])
    T45 = np.array([[math.cos(Q5), math.sin(Q5), 0, Config.R_L3 * math.cos(Q5)],
                    [math.sin(Q5), math.cos(Q5), 0, Config.R_L3 * math.sin(Q5)], [0, 0, 1, Config.R_L1], [0, 0, 0, 1]])
    T56 = np.array([[math.cos(Q6), 0, -math.sin(Q6), Config.R_L4 * math.cos(Q6)],
                    [math.sin(Q6), 0, math.cos(Q6), Config.R_L4 * math.sin(Q6)], [0, -1, 0, 0], [0, 0, 0, 1]])
    T67 = np.array([[math.cos(Q7), -math.sin(Q7), 0, Config.R_L5 * math.cos(Q7)],
                    [math.sin(Q7), math.cos(Q7), 0, Config.R_L5 * math.sin(Q7)], [0, 0, 1, 0], [0, 0, 0, 1]])
    T78 = np.array([[math.cos(Q8), 0, -math.sin(Q8), Config.R_L6 * math.cos(Q8)],
                    [math.sin(Q8), 0, math.cos(Q8), Config.R_L6 * math.sin(Q8)], [0, -1, 0, 0], [0, 0, 0, 1]])
    T89 = np.array([[math.cos(Q9), -math.sin(Q9), 0, Config.R_L7 * math.cos(Q9)],
                    [math.sin(Q9), math.cos(Q9), 0, Config.R_L7 * math.sin(Q9)], [0, 0, 1, 0], [0, 0, 0, 1]])

    # 传递矩阵计算
    # T02 = np.dot(T01, T12)  # np.dot(A, B) 矩阵A左乘于矩阵B
    # T03 = np.dot(T02, T23)
    # T04 = np.dot(T03, T34)
    T05 = np.dot(T34, T45)
    T06 = np.dot(T05, T56)
    T07 = np.dot(T06, T67)
    T08 = np.dot(T07, T78)
    T09 = np.dot(T08, T89)

    # 控制书出精度
    T09 = np.round(T09, 3)  # 可以通过np.round()函数控制矩阵元素精度

    # P09 = np.array([T09[0][3], T09[1][3], T09[2][3]])  # 提取位置信息，构建位置矩阵

    return T09

def calculate_init_position_crawl_tripod(swing_distance:float, height:float)-> dict:
    """
    计算三角步态初始位置

    Args:
        swing_distance (float): 支撑相腿移动的距离，单位mm
        height (float): 抬腿高度，单位mm

    Returns:
        dict: 包含每条腿初始位置的字典，形式如下: {leg_name: [x, y, z], ...}
    """
    init_position = {
        "LF": [Config.C_PX_init, Config.C_PY_init + swing_distance/2, Config.C_PZ_init - height/2],
        "RF": [Config.C_PX_init, Config.C_PY_init + swing_distance/2 - swing_distance/3*2, Config.C_PZ_init - height/2],
        "LB": [Config.C_PX_init, Config.C_PY_init - swing_distance/2 + swing_distance/3, Config.C_PZ_init - height/2],
        "RB": [Config.C_PX_init, Config.C_PY_init + swing_distance/2, Config.C_PZ_init - height/2]
    }
    return init_position

if __name__ == "__main__":
    T = SDH_positive_solution('RB', 0, 0, 0, 0, -90, 90)
    print(T)

    print('------------------')
    theta  = SDH_inverse_solution('RB', 0, 90, 129, -165, -60)
    print(theta)
    pass
