import Config
from serial_servo import serial_servo

def angle_to_pwm_crawl(joint_angles: dict) -> list:

    """将关节角度转换为舵机PWM信号,匍匐式
    Args:
        joint_angles (dict): 包含每条腿关节角度的字典，形式如下: {leg_name: [J2, J3, J4, J5, J6], ...}
    Returns:
        pwm_angles (list): 包含每个舵机ID及其对应PWM信号的列表，形式如下: [(servo_id, pwm), ...]
    """

    pwm_angles = []
    for name in joint_angles:
        for i in range(len(joint_angles[name])):
            delta = (joint_angles[name][i] - Config.init_angle_crawl[name][i]) * Config.servo_direction_factors[name][i]
            pwm = int((delta / 240) * 1000 + Config.init_servo_angle_crawl[name][i])
            # print(f'Leg: {name}, Joint {i+1}, Angle: {joint_angles[name][i]:.2f}°, DELTA: {int((delta/240)*1000)} , PWM: {pwm}')
            pwm_angles.append([Config.id_group[name][i], pwm])
    return pwm_angles

def angle_to_pwm_stand(joint_angles: dict) -> list:

    """将关节角度转换为舵机PWM信号,站立式
    Args:
        joint_angles (dict): 包含每条腿关节角度的字典，形式如下: {leg_name: [J2, J3, J4, J5, J6], ...}
    Returns:
        pwm_angles (list): 包含每个舵机ID及其对应PWM信号的列表，形式如下: [(servo_id, pwm), ...]
    """

    pwm_angles = []
    for name in joint_angles:
        for i in range(len(joint_angles[name])):
            delta = (joint_angles[name][i] - Config.init_angle_stand[name][i]) * Config.servo_direction_factors[name][i]
            pwm = int((delta / 240) * 1000 + Config.init_servo_angle_crawl[name][i])
            # print(f'Leg: {name}, Joint {i+1}, Angle: {joint_angles[name][i]:.2f}°, DELTA: {int((delta/240)*1000)} , PWM: {pwm}')
            pwm_angles.append((Config.id_group[name][i], pwm))
    return pwm_angles

def check_servo_info(pwm_angles: list):
    """
    检查舵机PWM信号是否在合理范围内,如果有超出范围的则进行警告提示并修改为边界值
    Args:
        pwm_angles (List[List]): 包含每个舵机ID及其对应PWM信号的列表，形式如下: [(servo_id, pwm), ...]
    """
    for i in range(len(pwm_angles)):
        if pwm_angles[i][1] < 0:
            print(f'Warning: Servo ID {pwm_angles[i][0]} PWM {pwm_angles[i][1]} out of range!')
            pwm_angles[i][1] = 0
        if pwm_angles[i][1] > 1000:
            print(f'Warning: Servo ID {pwm_angles[i][0]} PWM {pwm_angles[i][1]} out of range!')
            pwm_angles[i][1] = 1000

def move_servo(servo_info:list, time: int, servo_tool:serial_servo.SerialServo):
    """
    移动舵机到指定角度位置
    Args:
        pwm_angles (List[List]): 包含每个舵机ID及其对应PWM信号的列表，形式如下: [(servo_id, pwm), ...]
        time (int): 移动时间，单位ms
        servo_tool (serial_servo.SerialServo): 串口舵机控制对象
    """

    for i in range(len(servo_info)):
        servo_tool.move_servo_immediate(servo_info[i][0], servo_info[i][1], time)
