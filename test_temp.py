import time
import Config
import temp
import Servo
from serial_servo import serial_servo
import Tool
import serial

def Diagonal_gait_control_example(servo_tool:serial_servo.SerialServo):
    """_summary_站立式对角步态控制示例 2026 12 7 
    """
    Time_start = time.time()

    # 进入周期循环
    while True:
        Time_current = time.time() - Time_start # 获取当前时间
        print('t=', round(Time_current, Config.Diagnoal_Cyc_time), 's')

        Joint_group = temp.Diagonal_gait_paln(Time_current, Config.Diagnoal_Cyc_time)   # 获取当前关节角度
        angle_to_pwm = Servo.angle_to_pwm_crawl(Joint_group)    # 将关节角度转换为舵机角度
        Servo.check_servo_info(angle_to_pwm)    # 检查舵机信息
        Servo.move_servo(angle_to_pwm, 200, servo_tool)  # 舵机运动
        time.sleep(0.2) # 暂停一段时间，模拟舵机运动时间

        if Time_current >= Config.Diagnoal_Cyc_time:
            break

def Tripod_gait_control_example(servo_tool:serial_servo.SerialServo):
    """
    summary_机器人匍匐式三角步态运动控制示例 2026 1 27
    """
    Time_start = time.time()

    while True:
        Time_current = time.time() - Time_start
        print('t=', round(Time_current, Config.Tripod_Cyc_time), 's')

        Joint_group = temp.Tripod_gait_plan3(Time_current)
        angle_to_pwm = Servo.angle_to_pwm_crawl(Joint_group)
        Servo.check_servo_info(angle_to_pwm)
        Servo.move_servo(angle_to_pwm, 200, servo_tool)  # 舵机运动
        time.sleep(0.2)

        if Time_current >= Config.Tripod_Cyc_time:
            break

def init_to_target_position(servo_tool:serial_servo.SerialServo, target_position:dict, spine_angle:float, time:int, mode:str):

    """
    将机器人从当前位置移动到目标位置, 直接移动不做轨迹规划
    Args:
        servo_tool (serial_servo.SerialServo): 串口舵机控制对象
        target_position (dict): 目标位置字典，形式如下: {leg_name: [x, y, z], ...}
        spine_angle (float): 脊柱角度，单位°
        time (int): 移动时间，单位ms
    """

    angles = {}
    for leg_name in target_position:
        if mode == 'crawl':
            angles[leg_name] = Tool.SDH_inverse_solution(leg_name, spine_angle, 90, target_position[leg_name][0], target_position[leg_name][1], target_position[leg_name][2])
        elif mode == 'stand':
            angles[leg_name] = Tool.SDH_Stand_inverse_solution(target_position[leg_name][0], target_position[leg_name][1], target_position[leg_name][2])
    angles["SPINE"] = [spine_angle]
    pwm_angles = Servo.angle_to_pwm_crawl(angles)
    print("Target Angles:", angles)
    print("PWM Angles:", pwm_angles)
    Servo.check_servo_info(pwm_angles)

    Servo.move_servo(pwm_angles, time, servo_tool)

if __name__ == "__main__":

    serial_port = serial.Serial('/dev/ttyUSB0', 115200)
    servo_tool = serial_servo.SerialServo(serial_port)
    target_position_crawl = Tool.calculate_init_position_crawl_tripod(0.75, Config.Tripod_Cyc_Disp, Config.Tripod_Cyc_Hig)
    target_position_stand = Tool.calculate_init_position_stand_diagonal(0.5, Config.Diagonal_Cyc_Disp, Config.Diagnoal_Cyc_Hig)

    # init_to_target_position(servo_tool, target_position_crawl, 0, 4000, "crawl")
    # Tripod_gait_control_example(servo_tool)
    # Tripod_gait_control_example(servo_tool)
    # Tripod_gait_control_example(servo_tool)
    # Tripod_gait_control_example(servo_tool)

    init_to_target_position(servo_tool, target_position_stand, 0, 4000, "stand")
    Diagonal_gait_control_example(servo_tool)