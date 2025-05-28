"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
import User_Function as Fun
import cubic_spline_planner
import math
import can

k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle

show_animation = True


class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    # if last_target_idx >= current_target_idx:
    #     current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    delta = np.clip(delta, -np.radians(30.0), np.radians(30.0))

    return delta, current_target_idx, error_front_axle


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

# def calc_target_index(state, cx, cy, last_index=0, window_size=10):
    # 计算前轴位置
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # 限制搜索范围在最后找到的索引周围的窗口内
    start_idx = max(0, last_index - window_size)
    end_idx = min(len(cx), last_index + window_size)

    # 在限定范围内搜索最近点
    dx = [fx - cx[i] for i in range(start_idx, end_idx)]
    dy = [fy - cy[i] for i in range(start_idx, end_idx)]
    d = np.hypot(dx, dy)
    target_idx = start_idx + np.argmin(d)

    # 计算误差
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[np.argmin(d)], dy[np.argmin(d)]], front_axle_vec)

    return target_idx, error_front_axle

def calc_nearest_index(state, cx, cy, cyaw, last_index=0, window_size=20):
    # 限制搜索范围在最后找到的索引周围的窗口内
    start_idx = max(0, last_index)
    end_idx = min(len(cx), last_index + window_size)

    # 在限定范围内搜索最近点
    dx = [state.x - icx for icx in cx[start_idx:end_idx]]
    dy = [state.y - icy for icy in cy[start_idx:end_idx]]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    # 找到最近点的索引
    ind = start_idx + d.index(mind)

    # 计算最近点的距离
    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle > 0:
        mind *= -1

    return ind, mind

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def smooth_path(cx, cy, alpha=0.1, beta=0.1, iterations=10):
    """对路径点进行平滑处理"""
    for _ in range(iterations):
        for i in range(1, len(cx) - 1):
            cx[i] = alpha * cx[i] + (1 - alpha) * (cx[i - 1] + cx[i + 1]) / 2
            cy[i] = alpha * cy[i] + (1 - alpha) * (cy[i - 1] + cy[i + 1]) / 2
            cx[i] -= beta * (cx[i] - (cx[i - 1] + cx[i + 1]) / 2)
            cy[i] -= beta * (cy[i] - (cy[i - 1] + cy[i + 1]) / 2)
    return cx, cy



def main():
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    ax = [0.0, 50.0, 100.0]
    ay = [0.0, 0.0, 0.0]
    # start = (3811675.0330035696, -45916.88186437613)
    # point1 = (3811710.9065166865, -45913.29451189097)
    # point2 = (3811709.375020633, -45861.74138243857)

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    # cx, cy, cyaw, s = Fun.generate_spiral_path(start, point1, point2, spacing=1, shrink_amount=1.9)
    # cx, cy = smooth_path(cx, cy, alpha=0.1, beta=0.1, iterations=1)
    # changed_indices, change_count = Fun.check_yaw_changes(cx, cy, cyaw, s)
    target_speed = 10.0 / 3.6  # [m/s]

    max_simulation_time = 10000.0

    # Initial state
    state = State(x=-1.0, y=-8.0, yaw=0, v=0.0)
    # state = State(x=3811675, y=-45918, yaw=np.radians(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx = 0 

    threshold_distance = 1
   

    while max_simulation_time >= time and last_idx > target_idx:
        # prev_x, prev_y = cx[changed_indices[0] - 1], cy[changed_indices[0] - 1]
        # distance = np.hypot(state.x - prev_x, state.y - prev_y)
        # if distance < threshold_distance:
        #     for i in range(20):
        #         print(f"Loop iteration: {i + 1}")
        #         ai = pid_control( -10/3.6, state.v)
        #         di= -np.pi /4
        #         state.update(ai, di)
        #     changed_indices.pop(0)

        ai = pid_control(target_speed, state.v)
        
        di, target_idx,e = stanley_control(state, cx, cy, cyaw)
        print(di,e)
        range1 = (0.03, 0.1)
        range2 = (0.1, 0.5235)
        output1 = (500, 690)
        output2 = (690, 705)
        max_output = 710

        outputs = Fun.compute_output(abs(di), range1, range2, output1, output2, 500,max_output)
        # print(outputs)
        # print(f'{state.x},{state.y}')


        state.update(ai, di)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  # pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
