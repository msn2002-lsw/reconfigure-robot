import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import Gait
import time
import Config
import Tool

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

def test_get_position_in_gait_cycle():
    legInfo = [
        Config.LegParam(name='LF', offset=0.0, duty_factor=0.75),
        Config.LegParam(name='RF', offset=0.5, duty_factor=0.75),
        Config.LegParam(name='LB', offset=0.75, duty_factor=0.75),
        Config.LegParam(name='RB', offset=0.25, duty_factor=0.75)
    ]
    t_total = 2.0
    time_start = time.time()
    time_now = time.time() - time_start

    init_position = Tool.calculate_init_position_crawl_tripod(60, 30)
    print(f'Initial Position: {init_position}')
    phaseChange = Gait.is_phase_change(legInfo, 0, time_now, t_total)
    positions = {}
    angles = {}
    spineangle = 0

    while True:

        spineangle = Gait.spine_plan(time_now, t_total, 20)     # 脊柱扭动规划，最大扭动角度20度
        # print(f'Spine Angle: {spineangle:.2f}°')

        temp = Gait.getPositionInGaitCycle(legInfo, time_now, t_total, 68, 50)
        for leg_name in temp:
            positions[leg_name] = [temp[leg_name][0] + init_position[leg_name][0], temp[leg_name][1] + init_position[leg_name][1], temp[leg_name][2] + init_position[leg_name][2]]
            print(f'Leg: {leg_name}, Position: {positions[leg_name]}')
            angles[leg_name] = Tool.SDH_inverse_solution(leg_name, spineangle, 90, positions[leg_name][0], positions[leg_name][1], positions[leg_name][2])
            print(f'Leg: {leg_name}, Angles: {angles[leg_name]}')
        # print(f'Time: {time_now/t_total:.2f}s, Leg Positions: {positions}')
        if time_now > t_total:
            break
        time.sleep(0.1)
        # print(init_position)
        time_next = time.time() - time_start
        phaseChange = Gait.is_phase_change(legInfo, time_now, time_next, t_total)
        for leg  in legInfo:
            if phaseChange[leg.name]:
                init_position[leg.name] = [init_position[leg.name][0] + 0, init_position[leg.name][1] + leg.duty_factor*80*phaseChange[leg.name], init_position[leg.name][2] + 0]                 # 更新腿的初始位置
        time_now = time_next

if __name__ == "__main__":
    # test_foot_path_plan()
    # test_gait_plan()
    test_get_position_in_gait_cycle()
    pass