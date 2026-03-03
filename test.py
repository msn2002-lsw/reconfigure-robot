import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import Gait
import time
import Config
import Tool
from typing import List
from serial_servo import serial_servo
import serial
import Servo

def test_foot_path_plan():

    length = 500
    height = 200
    t_total = 0.5

    x_vals = []
    y_vals = []
    z_vals = []

    time_start = time.time()
    while time.time() - time_start < t_total:
        time_now = time.time() - time_start
        x, y, z = Gait.swing_foot_end_path_plan(length, height, time_now/t_total)
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        time.sleep(0.01)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_vals, y_vals, z_vals, color='red', label='Foot Path')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Swing Foot End Path Planning')
    ax.legend()
    plt.show()
def test_gait_plan():
    legInfo = [
        Config.LegParam(name='LF', offset=0.0, duty_factor=0.75),
        Config.LegParam(name='RF', offset=0.5, duty_factor=0.75),
        Config.LegParam(name='LB', offset=0.75, duty_factor=0.75),
        Config.LegParam(name='RB', offset=0.25, duty_factor=0.75)
    ]
    t_total = 2.0
    time_start = time.time()
    while time.time() - time_start < t_total:
        time_now = time.time() - time_start
        legStates = Gait.gait_plan(legInfo, time_now, t_total)
        print(f'Time: {time_now/t_total:.2f}s, Leg States: {legStates}')
        time.sleep(0.1)

def test_crawl_tripod_gait():
    # 确定相位信息
    legInfo = [
        Config.LegParam(name='LF', offset=0.0, duty_factor=0.75),
        Config.LegParam(name='RF', offset=0.5, duty_factor=0.75),
        Config.LegParam(name='LB', offset=0.25, duty_factor=0.75),
        Config.LegParam(name='RB', offset=0.75, duty_factor=0.75)
    ]
    t_total = 2.0
    time_start = time.time()
    step_size = 50
    height = 65

    init_position = Tool.calculate_init_position_crawl_tripod(0.75, step_size, height)
    # print(f'Initial Position: {init_position}')
    time_now = time.time() - time_start
    phaseChange = Gait.is_phase_change(legInfo, 0, time_now, t_total)
    positions = {}
    angles = {}
    spineangle = 0

    while True:

        spineangle = Gait.spine_plan(time_now, t_total, 10)     # 脊柱扭动规划，最大扭动角度10度
        # print(f'Spine Angle: {spineangle:.2f}°')
        print("time_now:", time_now)

        temp = Gait.getPositionInGaitCycle(legInfo, time_now, t_total, height, step_size)
        for leg_name in temp:
            positions[leg_name] = [temp[leg_name][0] + init_position[leg_name][0], temp[leg_name][1] + init_position[leg_name][1], temp[leg_name][2] + init_position[leg_name][2]]
            # positions[leg_name] = [temp[leg_name][0], temp[leg_name][1], temp[leg_name][2]]
            # print(f'Leg: {leg_name}, Position: {positions[leg_name]}')
            angles[leg_name] = Tool.SDH_inverse_solution(leg_name, spineangle, 90, positions[leg_name][0], positions[leg_name][1], positions[leg_name][2])
            print(f'Leg: {leg_name}, Angles: {angles[leg_name]}')
        angles["SPINE"] = [spineangle]
        angle_to_pwm = Servo.angle_to_pwm_crawl(angles)
        Servo.check_servo_info(angle_to_pwm)
        # print(f'Angle to PWM: {angle_to_pwm}')
        print("")
        if time_now > t_total:
            break
        time.sleep(0.2)
        # print(init_position)
        time_next = time.time() - time_start
        phaseChange = Gait.is_phase_change(legInfo, time_now, time_next, t_total)
        for leg  in legInfo:
            if phaseChange[leg.name]:
                init_position[leg.name] = [init_position[leg.name][0] + 0, init_position[leg.name][1] + leg.duty_factor*step_size*phaseChange[leg.name], init_position[leg.name][2] + 0]                 # 更新腿的初始位置
                # print("Updated Initial Position:", init_position)
        time_now = time_next

def test_stand_diagonal_gait():
    # 确定相位信息,以及步态参数
    legInfo = [
        Config.LegParam(name='LF', offset=0.0, duty_factor=0.5),
        Config.LegParam(name='RF', offset=0.5, duty_factor=0.5),
        Config.LegParam(name='LB', offset=0.5, duty_factor=0.5),
        Config.LegParam(name='RB', offset=0.0, duty_factor=0.5)
    ]
    t_total = Config.Diagnoal_Cyc_time
    step_size = Config.Diagonal_Cyc_Disp
    height = Config.Diagnoal_Cyc_Hig

    time_start = time.time()

    init_position = Tool.calculate_init_position_stand_diagonal(0.5, step_size, height)
    # print(f'Initial Position: {init_position}')
    time_now = time.time() - time_start
    phaseChange = Gait.is_phase_change(legInfo, 0, time_now, t_total)
    positions = {}
    angles = {}
    spineangle = 0

    while True:

        # print(f'Spine Angle: {spineangle:.2f}°')
        print("time_now:", time_now)

        temp = Gait.getPositionInGaitCycle(legInfo, time_now, t_total, height, step_size)
        for leg_name in temp:
            positions[leg_name] = [temp[leg_name][0] + init_position[leg_name][0], temp[leg_name][1] + init_position[leg_name][1], temp[leg_name][2] + init_position[leg_name][2]]
            # positions[leg_name] = [temp[leg_name][0], temp[leg_name][1], temp[leg_name][2]]
            print(f'Leg: {leg_name}, Position: {positions[leg_name]}')
            angles[leg_name] = Tool.SDH_Stand_inverse_solution(positions[leg_name][0], positions[leg_name][1], positions[leg_name][2])
            # print(f'Leg: {leg_name}, Angles: {angles[leg_name]}')
        angles["SPINE"] = [spineangle]
        angle_to_pwm = Servo.angle_to_pwm_stand(angles)
        Servo.check_servo_info(angle_to_pwm)

        print("")
        # print(f'Angle to PWM: {angle_to_pwm}')
        if time_now > t_total:
            break
        time.sleep(0.1)
        # print(init_position)
        time_next = time.time() - time_start
        phaseChange = Gait.is_phase_change(legInfo, time_now, time_next, t_total)
        for leg  in legInfo:
            if phaseChange[leg.name]:
                init_position[leg.name] = [init_position[leg.name][0] + 0, init_position[leg.name][1] + leg.duty_factor*step_size*phaseChange[leg.name], init_position[leg.name][2] + 0]                 # 更新腿的初始位置
                # print("Updated Initial Position:", init_position)
        time_now = time_next
    pass

def init_to_target_position_crawl(servo_tool:serial_servo.SerialServo, target_position:dict, spine_angle:float, time:int):
    """
    将机器人从当前位置移动到目标位置, 直接移动不做轨迹规划
    Args:
        servo_tool (serial_servo.SerialServo): 串口舵机控制对象
        target_position (dict): 目标位置字典，形式如下: {leg_name: [x, y, z], ...}
        spine_angle (float): 脊柱角度，单位°
        time (int): 移动时间，单位ms
    Returns:
        angles (dict): 目标关节角度字典，形式如下: {leg_name: [J2, J3, J4, J5, J6], ...}
    """
    angles = {}
    for leg_name in target_position:
        angles[leg_name] = Tool.SDH_inverse_solution(leg_name, spine_angle, 90, target_position[leg_name][0], target_position[leg_name][1], target_position[leg_name][2])
    angles["SPINE"] = [spine_angle]
    pwm_angles = Servo.angle_to_pwm_crawl(angles)
    # move_servo(pwm_angles, time, servo_tool)
    return pwm_angles

def init_to_target_position_stand(servo_tool:serial_servo.SerialServo, target_position:dict, spine_angle:float, time:int):
    """
    将机器人从当前位置移动到目标位置, 直接移动不做轨迹规划
    Args:
        servo_tool (serial_servo.SerialServo): 串口舵机控制对象
        target_position (dict): 目标位置字典，形式如下: {leg_name: [x, y, z], ...}
        spine_angle (float): 脊柱角度，单位°
        time (int): 移动时间，单位ms
    Returns:
        angles (dict): 目标关节角度字典，形式如下: {leg_name: [J2, J3, J4, J5, J6], ...}
    """
    angles = {}
    for leg_name in target_position:
        angles[leg_name] = Tool.SDH_Stand_inverse_solution(target_position[leg_name][0], target_position[leg_name][1], target_position[leg_name][2])
    angles["SPINE"] = [spine_angle]
    pwm_angles = Servo.angle_to_pwm_stand(angles)
    # move_servo(pwm_angles, time, servo_tool)
    return pwm_angles

def get_servo_info(id:List, servo_tool:serial_servo.SerialServo)-> List:
    servo_info = []
    for i in id:
        servo_info.append([i, servo_tool.read_servo_position(i)])
        # servo_info.append([i, servo_tool.read_servo_temp(i)])
        servo_info.append([i, servo_tool.read_servo_voltage(i)])
    return servo_info

def move_servo(servo_info:List[list], time: int, servo_tool:serial_servo.SerialServo):
    for i in range(len(servo_info)):
        servo_tool.move_servo_immediate(servo_info[i][0], servo_info[i][1], time)

if __name__ == "__main__":
    # test_foot_path_plan()
    # test_gait_plan()
    # test_get_position_in_gait_cycle()
    serial_port = serial.Serial('/dev/ttyUSB0', 115200)
    servo_tool = serial_servo.SerialServo(serial_port)

    servo_id = list(range(1,22)) 
    servo_info = get_servo_info(servo_id, servo_tool)
    print(servo_info)
    time.sleep(1)

    servo_info_target = []
    # for leg_name in Config.id_group:
    #     for i in range(len(Config.id_group[leg_name])):
    #         print(i)
    #         servo_info_target.append([Config.id_group[leg_name][i],Config.init_servo_angle_crawl[leg_name][i]])
    # move_servo(servo_info_target, 4000, servo_tool)
    # time.sleep(4)

    # test_crawl_tripod_gait()
    # angles = init_to_target_position(servo_tool, Tool.calculate_init_position_crawl_tripod(0.75, 50, 65), 0, 4000)
    # print(angles)
    # check_servo_info(angles)

    # test_stand_diagonal_gait()
    # servo_info = [[15,405]]
    # move_servo(servo_info, 4000, servo_tool)
    # time.sleep(4)

    # servo_info_target.clear()
    # # 先将腿向脊柱侧收回一部分
    # move_servo([[2,400],[7,800],[12,400],[17,740], [4,340], [9,460], [14,550], [19,350]], 2000, servo_tool)    
    # time.sleep(2)
    # for leg_name in Config.id_group:
    #     for i in range(len(Config.id_group[leg_name])):
    #         print(i)
    #         servo_info_target.append([Config.id_group[leg_name][i],Config.init_servo_angle_stand[leg_name][i]])
    # move_servo(servo_info_target, 4000, servo_tool)
    # time.sleep(4)

    # # 等待运动结束
    # time.sleep(2)
    # servo_info_target.clear()

    # # 转变匍匐式
    # for leg_name in Config.id_group:
    #     for i in range(len(Config.id_group[leg_name])):
    #         print(i)
    #         servo_info_target.append([Config.id_group[leg_name][i],Config.init_servo_angle_crawl[leg_name][i]])
    # move_servo(servo_info_target, 4000, servo_tool)
    # time.sleep(4)
    # pass