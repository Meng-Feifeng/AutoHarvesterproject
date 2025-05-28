"""


"""
import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import cubic_spline_planner
import DataBase as DB
import stanley_controller as Stan
import User_Function as Fun

# Best Q matrix:
# [[13.37434018  0.          0.        ]
#  [ 0.         13.37434018  0.        ]
#  [ 0.          0.         26.54586852]]
# Best R matrix:
# [[17.94252273  0.        ]
#  [ 0.         17.94252273]]
# LQR parameter
Q = np.eye(3)
Q[0, 0] = 3
Q[1, 1] = 3
Q[2, 2] = 6
Q[0, 0] = 13
Q[1, 1] = 13
Q[2, 2] = 26
R = np.eye(2)
R[0, 0] = 1
R[1, 1] = 1
# R[0, 0] = 17
# R[1, 1] = 17

# parameters
dt = 0.1  # time tick[s]
W = 0.7  # [m]
b = 0.1  # [m]
v_target=3

GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 1 / 3.6  # stop speed

show_animation = True
# show_animation = False
show_flag = False
show_flag = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def update(state, v_r, v_l, dt ):

    state.x = state.x + 0.5 * v_r * math.cos(state.yaw) * dt + 0.5 * v_l * math.cos(state.yaw) * dt
    state.y = state.y + 0.5 * v_r * math.sin(state.yaw) * dt + 0.5 * v_l * math.sin(state.yaw) * dt
    state.yaw = state.yaw + v_r / (W + b) * dt - v_l / (W + b) * dt
    state.v = 0.5 * v_r + 0.5 * v_l

    return state

def update(state, v, di, dt ):

    state.x = state.x + 0.5 * v * math.cos(state.yaw) * dt + 0.5 * v * math.cos(state.yaw) * dt
    state.y = state.y + 0.5 * v * math.sin(state.yaw) * dt + 0.5 * v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + (di + 2*v)/ 2 / (W + b) * dt - (2*v - di)/ 2 / (W + b) * dt
    state.v = v

    return state

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn

def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eigVals, eigVecs = la.eig(A - B @ K)

    return K, X, eigVals

def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, last_ind):

    ind, e = calc_nearest_index(state, cx, cy, cyaw, last_ind)
    # ind, e = calc_nearest_index(state, cx, cy, cyaw)

    k = ck[ind]
    v_r_r = v_target/3.6- k * v_target/3.6*0.4
    v_r_l = v_target/ 3.6 + k * v_target / 3.6 * 0.4
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.zeros((3, 3))
    A[0, 0] = 1.0
    A[0, 2] = -v_target/3.6*math.sin(cyaw[ind])*dt
    A[1, 1] = 1.0
    A[1, 2] = v_target/3.6*math.cos(cyaw[ind])*dt
    A[2, 2] = 1.0
    # print(A)

    B = np.zeros((3, 2))
    B[0, 0] = 0.5*math.cos(cyaw[ind])*dt
    B[0, 1] = 0.5*math.cos(cyaw[ind])*dt
    B[1, 0] = 0.5*math.sin(cyaw[ind])*dt
    B[1, 1] = 0.5*math.sin(cyaw[ind])*dt
    B[2, 0] = 1/(0.8)*dt
    B[2, 1] = -1/(0.8)*dt

    K, _, _ = dlqr(A, B, Q, R)

    x = np.zeros((3, 1))

    x[0, 0] = state.x-cx[ind]
    x[1, 0] = state.y-cy[ind]
    x[2, 0] = th_e
    vr = (-K @ x)[0, 0] + v_r_r
    vf = (-K @ x)[1, 0] + v_r_l

    # vr = v_r_r
    # vf = v_r_l
    # print(f"vr:{vr},vf:{vf}")

    return vr, vf, ind, e, th_e

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

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
    if angle < 0:
        mind *= -1

    return ind, mind

def plot_car(x, y, yaw, steer=0.0, truck_color="-k"):  # pragma: no cover
    # Vehicle parameters
    LENGTH = 4.88  # [m]
    WIDTH = 2.255  # [m]
    BACK_TO_WHEEL = 2  # [m]
    TRACK_LENGTH = 2.5  # [m]
    TRACK_WIDTH = 0.5  # [m]
    TREAD = 1.255  # [m]
    TRACK_OFFSET = 0.2  # [m] 履带向内偏移量
    FORWARD_OFFSET = 0.2  # [m] 履带向前偏移量

    outline = np.array(
        [[-BACK_TO_WHEEL, (LENGTH - BACK_TO_WHEEL), (LENGTH - BACK_TO_WHEEL), -BACK_TO_WHEEL, -BACK_TO_WHEEL],
         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    left_track = np.array(
        [[-TRACK_LENGTH / 2 + FORWARD_OFFSET, TRACK_LENGTH / 2 + FORWARD_OFFSET, TRACK_LENGTH / 2 + FORWARD_OFFSET, -TRACK_LENGTH / 2 + FORWARD_OFFSET, -TRACK_LENGTH / 2 + FORWARD_OFFSET],
         [TREAD / 2 + TRACK_WIDTH - TRACK_OFFSET, TREAD / 2 + TRACK_WIDTH - TRACK_OFFSET, TREAD / 2 - TRACK_OFFSET, TREAD / 2 - TRACK_OFFSET, TREAD / 2 + TRACK_WIDTH - TRACK_OFFSET]])

    right_track = np.copy(left_track)
    right_track[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])

    left_track = (left_track.T.dot(Rot1)).T
    right_track = (right_track.T.dot(Rot1)).T
    outline = (outline.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    left_track[0, :] += x
    left_track[1, :] += y
    right_track[0, :] += x
    right_track[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truck_color)
    plt.plot(np.array(left_track[0, :]).flatten(),
             np.array(left_track[1, :]).flatten(), truck_color)
    plt.plot(np.array(right_track[0, :]).flatten(),
             np.array(right_track[1, :]).flatten(), truck_color)
    plt.plot(x, y, "*")

def closed_loop_prediction(cx, cy, cyaw, ck, v, goal):
    T = 100.0  # max simulation time

    # 设初始状态
    state = State(x=3811705, y=-45917, yaw=0.0, v=0)
    state = State(x=-0, y=-2, yaw=0.0, v=0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    e_list = [0]
    e_th_list = [0]
    v = [state.v]
    v_vr = [0]
    v_vl = [0]
    delta = [0]
    t = [0.0]

    e, e_th = 0.0, 0.0
    last_ind =0

    while T >= time:

        vr, vl, target_ind, e, e_th = lqr_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, last_ind)
        last_ind = target_ind
        di = vr - vl
        # if abs(di) < 0.05 :
        #     di = 0
        di = np.clip(di, -2, 2)

        print(f"{di:.3f}")
        
        vv = 0.5 * (vr + vl)

        # state = update(state, vr, vl, dt)
        state = update(state, vv, di, dt)

        time = time + dt

        # input("按任意键继续...")

        if check_goal(state, goal, target_ind, len(cx)):
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        e_list.append(e)
        e_th_list.append(e_th)
        v.append(state.v)
        v_vr.append(vr)
        v_vl.append(vl)
        delta.append(di)
        t.append(time)

        if target_ind % 1 == 0 and show_animation and show_flag:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-g", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(x[-1], y[-1], yaw[-1])
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v, v_vr, v_vl, delta, e_list, e_th_list

def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal or isstop:
        return True

    return False

def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)

    direction = 1.0

    # Set stop point
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0

    return speed_profile

def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    print(ck)

    return cx, cy, cyaw, ck, s
def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck, s
def get_line_self_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 20.0, 10.0, 5.0]
    ay = [0.0, 0.0, 1.0, 2.0, 3.0, 5.0, 5.0]
    ax = [0.0, 5.0, 10.0]
    ay = [0.0, 0.0, 0.0]
    # ax = [3811964.597807532, 3811990.674834497]
    # ay = [-46323.20031018404, -46322.2083096687]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    return cx, cy, cyaw, ck, s


# 初始化遗传算法参数
population_size = 50
generations = 100
mutation_rate = 0.1
crossover_rate = 0.7

# 适应度函数
def fitness(Q_values, R_value, cx, cy, cyaw, ck, v_target, goal):
    global Q, R
    try:
        # 修改全局Q和R矩阵的值
        Q = np.zeros((3, 3))
        R = np.zeros((2, 2))
        Q[0, 0], Q[1, 1], Q[2, 2] = Q_values[0], Q_values[0], Q_values[1]
        R[0, 0] = R_value
        R[1, 1] = R_value
        
        # 运行路径跟踪仿真
        t, x, y, yaw, v, v_vr, v_vl, delta, e, e_th = closed_loop_prediction(cx, cy, cyaw, ck, v_target / 3.6, goal)
        
        # 转换为NumPy数组
        e = np.array(e)
        e_th = np.array(e_th)
        v_vr = np.array(v_vr)
        v_vl = np.array(v_vl)
        
        # 计算跟踪误差和航向偏差
        tracking_error = np.sum(e**2 + e_th**2)
        
        # 计算控制能量
        control_effort = np.sum(v_vr**2 + v_vl**2)
        
        # 检查并截断不合理的数值
        if not np.isfinite(tracking_error) or not np.isfinite(control_effort) or tracking_error > 1e10 or control_effort > 1e10:
            raise ValueError("Invalid value encountered in fitness calculation")
        
        # 适应度值是跟踪误差和控制能量的加权和
        fitness_value = tracking_error + control_effort
        
        return -fitness_value  # 目标是最小化该值
    except Exception as e:
        print(f"Error in fitness calculation: {e}")
        return -np.inf  # 返回极小值以处理异常情况

# 初始化种群
def initialize_population(population_size):
    population = []
    for _ in range(population_size):
        Q_values = np.random.rand(2) * 10  # 随机生成Q矩阵的两个对角线元素
        Q_values = np.clip(Q_values, 0.1, None)  # 确保Q矩阵的对角线元素大于零
        R_value = np.random.rand() * 10 + 0.1  # 确保R矩阵的对角线元素大于零并相等
        population.append((Q_values, R_value))
    return population

# 选择适应度最高的个体
def select(population, fitnesses):
    idx = np.argsort(fitnesses)[::-1]
    return population[idx[0]]

# 交叉操作
def crossover(parent1, parent2, crossover_rate):
    if np.random.rand() < crossover_rate:
        cut_point = np.random.randint(1, len(parent1[0]))
        child1_Q = np.concatenate((parent1[0][:cut_point], parent2[0][cut_point:]))
        child2_Q = np.concatenate((parent2[0][:cut_point], parent1[0][cut_point:]))
        child1_R = (parent1[1] + parent2[1]) / 2
        child2_R = child1_R  # 确保R矩阵的对角线元素相等
        return (np.clip(child1_Q, 0.1, None), np.clip(child1_R, 0.1, None)), (np.clip(child2_Q, 0.1, None), np.clip(child2_R, 0.1, None))
    else:
        return parent1, parent2

# 变异操作
def mutate(individual, mutation_rate):
    Q_values, R_value = individual
    if np.random.rand() < mutation_rate:
        Q_values[np.random.randint(len(Q_values))] += np.abs(np.random.randn())
        Q_values = np.clip(Q_values, 0.1, None)  # 确保Q矩阵的对角线元素大于零
    if np.random.rand() < mutation_rate:
        R_value += np.abs(np.random.randn())
        R_value = max(R_value, 0.1)  # 确保R矩阵的对角线元素大于零
    return (Q_values, R_value)

# 遗传算法主循环
def genetic_algorithm(cx, cy, cyaw, ck, v_target, goal, population_size, generations, mutation_rate, crossover_rate):
    population = initialize_population(population_size)
    for generation in range(generations):
        fitnesses = np.array([fitness(Q_values, R_value, cx, cy, cyaw, ck, v_target, goal) for Q_values, R_value in population])
        new_population = []
        for _ in range(population_size // 2):
            parent1 = select(population, fitnesses)
            parent2 = select(population, fitnesses)
            child1, child2 = crossover(parent1, parent2, crossover_rate)
            child1 = mutate(child1, mutation_rate)
            child2 = mutate(child2, mutation_rate)
            new_population.extend([child1, child2])
        population = new_population
        best_individual = select(population, fitnesses)
        print(f"Generation {generation + 1}: Best fitness = {max(fitnesses)}")
    return best_individual

def main():
    print("LQR steering control tracking start!!")
    
    cx, cy, cyaw, ck, s = get_line_self_course(0.1)

    # start = (3811675.0330035696, -45916.88186437613)
    # point1 = (3811710.9065166865, -45913.29451189097)
    # point2 = (3811709.375020633, -45861.74138243857)

    # cx, cy, cyaw, s = Fun.generate_straight_path(start, point1, point2, spacing=1, shrink_amount=1.9)
    # cx, cy = Fun.smooth_path(cx, cy, alpha=0.1, beta=0.01, iterations=1)
    # ck = [0]*len(cx)
    # changed_indices, change_count = Fun.check_yaw_changes(cx, cy, cyaw, s)
    # print(changed_indices)
    goal = [cx[-1], cy[-1]]

    t, x, y, yaw, v, v_vr, v_vl = closed_loop_prediction(cx, cy, cyaw, ck, v_target/3.6, goal)


    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        # plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(t, [np.rad2deg(iyaw) for iyaw in yaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(t, v, "-r", label="Centroid Speed (v)")
        plt.plot(t, v_vr, "-g", label="Right Track Speed (v_vr)")
        plt.plot(t, v_vl, "-b", label="Left Track Speed (v_vl)")
        plt.grid(True)
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [km/h]")

        plt.show()

def main1():
    print("LQR steering control tracking start!!")

    cx, cy, cyaw, ck, s = get_line_self_course(0.01)
    goal = [cx[-1], cy[-1]]

    t, x, y, yaw, v, v_vr, v_vl, delta, e , e_th = closed_loop_prediction(cx, cy, cyaw, ck, v_target/3.6, goal)

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots(1)
        plt.plot(t, [np.rad2deg(iyaw) for iyaw in yaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(t, v, "-r", label="Centroid Speed (v)")
        plt.plot(t, v_vr, "-g", label="Right Track Speed (v_vr)")
        plt.plot(t, v_vl, "-b", label="Left Track Speed (v_vl)")
        plt.plot(t, delta, "-y", label="Differential Wheel Speed (delta)")
        plt.grid(True)
        plt.legend()
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [km/h]")

        plt.show()

def main2():
    print("LQR steering control tracking start!!")

    # 获取参考路径
    cx, cy, cyaw, ck, s = get_line_self_course(0.01)
    goal = [cx[-1], cy[-1]]

    # 运行遗传算法
    best_Q_values, best_R_value = genetic_algorithm(cx, cy, cyaw, ck, v_target, goal, population_size, generations, mutation_rate, crossover_rate)

    # 打印最优的Q和R矩阵
    Q = np.zeros((3, 3))
    R = np.zeros((2, 2))
    Q[0, 0], Q[1, 1], Q[2, 2] = best_Q_values[0], best_Q_values[0], best_Q_values[1]
    R[0, 0] = best_R_value
    R[1, 1] = best_R_value
    print("Best Q matrix:")
    print(Q)
    print("Best R matrix:")
    print(R)

if __name__ == '__main__':
    # main()
    main1()
    # main2()