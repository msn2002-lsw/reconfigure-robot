from typing import List
import math
import Config

def hold_foot_end_path_plan(length, height, t):
    x = 0
    y = length*(t)
    z = height*(t)
    return [x,y,z]

def swing_foot_end_path_plan(length, height, t):
    """
    _摆动足端路径规划,采用无冲击曲线规划足端轨迹,返回值以步态初始的足端坐标为原点   

    :param length: 步长
    :param height: 抬腿高度
    :param t: 当前时间占周期的比例
    """
    x = 0
    y = length*(t-math.sin(2*math.pi*t)/(2*math.pi))
    if t <= 0.5:
        z = 2*height*(t-math.sin(4*math.pi*t)/(4*math.pi))
    else:
        z = 2*height*(1-t+(math.sin(4*math.pi*t)/(4*math.pi)))

    return [x, y, z]

def isSwing(phi:float,beta:float,t_now:float,t_total:float)->List:
    """
    步态规划,计算各条腿的状态,支撑/摆动,默认了左前腿在周期开始时进入摆动相周期
    :param phi: 相位差 
    :param beta: 占空比 
    :param t_now: 当前时间
    :param t_total: 总时间
    returns: 各条腿的状态：支撑/摆动,摆动相时间占比，支撑相时间占比
    """
    if t_now > t_total:     # 防止t_now超出范围
        t_now = t_total
    t=t_now/t_total  
    # 计算摆动相的周期范围
    tStart = phi
    tEnd = phi + (1-beta)
    if t >= tStart and t <= tEnd:
        return ["swing", (t-tStart)/(tEnd - tStart), 1]
    else:
        if t < tStart:
            return ["hold", (t/tStart), -tStart/beta]
        else:
            return ["hold", ((t-tEnd)/(1-tEnd)), -(1 - tEnd)/beta]

def gait_plan(legInfo: List[Config.LegParam], t_now:float, t_total:float):
    """
    gait_plan 的 Docstring
    Args:
        legInfo: 包含每条腿的相位差和占空比的列表,形式如下:
        t_now: 当前时间
        t_total: 总时间
    Returns:
        legStates: 包含每条腿状态的列表,形式如下: [[leg_name, state, t_phase, portion], ...]
    """
    legStates = []
    for info in legInfo:
        state = isSwing(info.offset, info.duty_factor, t_now, t_total)
        legStates.append([info.name, state[0], state[1], state[2]])
    return legStates

def is_phase_change(legInfo: List[Config.LegParam], t_old:float, t_new:float, t_total:float)->dict:

    change = {}

    for leg in legInfo:
        old_state = isSwing(leg.offset, leg.duty_factor, t_old, t_total)
        new_state = isSwing(leg.offset, leg.duty_factor, t_new, t_total)
        if old_state[0] != new_state[0]:
            change[leg.name] = old_state[2]  # 返回变化前的portion和t_phase 
        else:
            change[leg.name] = False

    return change

def getPositionInGaitCycle(legInfo: List[Config.LegParam], t_now:float, t_total:float, height:float, stepSize:float)-> dict:
    """
    获取腿在步态周期中的位置,默认将步态初始的足端坐标作为原点,若支撑相分开则均匀分布

    Args:
        legInfo (List[Config.LegParam]): 包含每条腿的相位差和占空比的列表
        t_now (float): 当前时间
        t_total (float): 总时间
        height (float): 抬腿高度,单位mm
        stepSize (float): 步长,单位mm
        initPosition (dict): 每条腿的初始位置,形式如下: {leg_name: [x, y, z], ...}
    Returns:
        dict: 包含每条腿位置的字典，形式如下: {leg_name: [x, y, z], ...}
    """
    #计算实际各条腿需要移动的距离
    swingDistance = {}
    for i in legInfo:
        swingDistance[i.name] = stepSize * i.duty_factor

    legStates = gait_plan(legInfo, t_now, t_total)
    # 初始化位置
        
    position = {}

    for legState in legStates:
        leg_name = legState[0]
        state = legState[1]
        t_phase = legState[2]
        portion = legState[3]

        if state == "swing":
            x, y, z = swing_foot_end_path_plan(swingDistance[leg_name]*portion, height, t_phase)
            position[leg_name] = [x, y, z] 
        else:
            x, y, z = hold_foot_end_path_plan(swingDistance[leg_name]*portion, 0, t_phase)
            position[leg_name] = [x, y, z]

    return position

def spine_plan(t_now:float, t_total:float, maxSpineAngle:float)-> float:
    """_summary_机器人三角步态脊柱规划 无冲击曲线角度运动

    Args:
        t_now (_type_): _当前时间_
        t_total (_type_): _总时间_
        maxSpineAngle (_type_): _脊柱最大扭动角度（角度制）_

    Returns:
        _type_: _Spine_J 脊柱扭动角度（以右后腿为例，角度制）_
    """
    if t_now > t_total:
        t_now = t_total

    Spine_Angle = maxSpineAngle
    # if SDH_name == 'RB' or 'RF':
    #     Spine_Angle = self.Spine_Angle
    # elif SDH_name == 'LB' or 'LF':
    #     Spine_Angle = -self.Spine_Angle
    # else:
    #     print('SDH_name set error. Try to detect the method about Tripod_spine_plane')

    t_total = t_total / 4  # 三角步态有四个小周期

    if 0 <= t_now and t_now <= t_total:
        Spine_J = Spine_Angle * ((t_now / t_total) - 1 / (2 * math.pi) * math.sin(2 * math.pi * t_now / t_total))
    elif t_total < t_now and t_now <= t_total * 2:
        Spine_J = Spine_Angle - Spine_Angle * (((t_now - t_total) / t_total) - 1 / (2 * math.pi) * math.sin(2 * math.pi * (t_now - t_total) / t_total))
    elif t_total * 2 < t_now and t_now <= t_total * 3:
        Spine_J = -Spine_Angle * (((t_now - t_total * 2) / t_total) - 1 / (2 * math.pi) * math.sin(2 * math.pi * (t_now - t_total * 2) / t_total))
    elif t_total * 3 < t_now and t_now <= t_total * 4:
        Spine_J = -Spine_Angle + Spine_Angle * (((t_now - t_total * 3) / t_total) - 1 / (2 * math.pi) * math.sin(2 * math.pi *
                                                                                                    (t_now - t_total * 3) / t_total))
    return Spine_J

def plan_in_angle_space(t_now:float, t_total:float, angle_start:dict, angle_target:dict)->dict:
    # 线性插值
    if t_now > t_total:
        t_now = t_total
    ratio = t_now / t_total
    result = {}
    for key in angle_start:
        result[key] = angle_start[key] + (angle_target[key] - angle_start[key]) * ratio
    return result