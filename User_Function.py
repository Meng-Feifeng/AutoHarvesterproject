import matplotlib.pyplot as plt
import numpy as np
import math
import can

def generate_spiral_path(start, point1, point2, spacing=1, shrink_amount=2):
    """
    生成一个从给定起点和两个定位点开始的螺旋路径。

    参数：
    start: 起点 (x, y) 坐标元组
    point1: 定位点1 (x, y) 坐标元组
    point2: 定位点2 (x, y) 坐标元组
    spacing: 离散点之间的距离
    shrink_amount: 每次缩短的固定距离

    返回：
    xs, ys, yaws, indices: 每个离散点的x坐标列表、y坐标列表、航向角列表和索引列表
    """

    def generate_segment(start, end, spacing):
        """生成线段的点并计算线段的航向角和长度"""
        x = np.linspace(start[0], end[0], int(np.linalg.norm(end - start) / spacing) + 1)
        y = np.linspace(start[1], end[1], int(np.linalg.norm(end - start) / spacing) + 1)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        yaw = math.atan2(dy, dx)
        points = [(xi, yi, yaw) for xi, yi in zip(x, y)]
        length = np.linalg.norm(end - start)
        return points, yaw, length

    # 初始化路径列表
    path_points = []

    # 生成第一条直线：起点 -> 定位点1
    segment, yaw1, length1 = generate_segment(np.array(start), np.array(point1), spacing)
    path_points.extend(segment)

    # 生成第二条直线：定位点1 -> 定位点2
    segment, yaw2, length2 = generate_segment(np.array(point1), np.array(point2), spacing)
    path_points.extend(segment)

    # 初始化用于后续计算的变量
    yaws = [yaw1, yaw2]
    lengths = [length1, length2]
    last_end = np.array(point2)

    # 生成后续的螺旋路径
    while min(lengths) > shrink_amount:
        for i in range(2):  # 交替处理两个方向
            if lengths[i] > shrink_amount:
                yaw = yaws[i] + math.pi  # 反方向
                length = lengths[i] - shrink_amount  # 减少固定的长度
                lengths[i] = length  # 更新长度

                # 计算新的终点位置
                end_x = last_end[0] + length * math.cos(yaw)
                end_y = last_end[1] + length * math.sin(yaw)
                new_end = np.array([end_x, end_y])

                # 生成新的线段
                segment, new_yaw, new_length = generate_segment(last_end, new_end, spacing)
                path_points.extend(segment)
                last_end = new_end
                yaws[i] = new_yaw  # 更新航向角

    # 将数据拆分到单独的列表中
    _indices = []
    cx = []
    cy = []
    cyaw = []

    for idx, (x, y, yaw) in enumerate(path_points):
        _indices.append(idx)
        cx.append(x)
        cy.append(y)
        cyaw.append(yaw)

    return cx, cy, cyaw, _indices

def smooth_path(cx, cy, alpha=0.1, beta=0.1, iterations=10):
    """对路径点进行平滑处理"""
    for _ in range(iterations):
        for i in range(1, len(cx) - 1):
            cx[i] = alpha * cx[i] + (1 - alpha) * (cx[i - 1] + cx[i + 1]) / 2
            cy[i] = alpha * cy[i] + (1 - alpha) * (cy[i - 1] + cy[i + 1]) / 2
            cx[i] -= beta * (cx[i] - (cx[i - 1] + cx[i + 1]) / 2)
            cy[i] -= beta * (cy[i] - (cy[i - 1] + cy[i + 1]) / 2)
    return cx, cy

def check_yaw_changes(xs, ys, yaws, indices):
    """
    检查并记录航向角变化的点。

    参数:
    xs : list of float
        x坐标列表
    ys : list of float
        y坐标列表
    yaws : list of float
        航向角列表
    indices : list of int
        点的索引列表

    返回:
    (list of int, int)
        变化的点的索引列表和变化的总次数
    """
    prev_yaw = None
    yaw_change_indices = []
    count = 0

    # 遍历坐标点、航向角等，并检查航向角是否变化
    for idx, x, y, yaw in zip(indices, xs, ys, yaws):
        if prev_yaw is not None and yaw != prev_yaw:
            # 如果航向角与前一个不同，记录该点的索引
            yaw_change_indices.append(idx)
            # print(f"Index: {idx}, X: {x:.2f}, Y: {y:.2f}, Yaw: {yaw:.2f} (Yaw changed)")
            count += 1
        prev_yaw = yaw

    return yaw_change_indices, count

def generate_straight_path(start, point1, point2, spacing=0.5, shrink_amount=2):
    """
    生成一个从给定起点和两个定位点开始的直线套形路径。

    参数：
    start: 起点 (x, y) 坐标元组
    point1: 定位点1 (x, y) 坐标元组
    point2: 定位点2 (x, y) 坐标元组
    spacing: 离散点之间的距离
    shrink_amount: 每次缩短的固定距离

    返回：
    xs, ys, yaws, indices: 每个离散点的x坐标列表、y坐标列表、航向角列表和索引列表
    """

    def generate_segment(start, end, spacing):
        """生成线段的点并计算线段的航向角和长度"""
        x = np.linspace(start[0], end[0], int(np.linalg.norm(end - start) / spacing) + 1)
        y = np.linspace(start[1], end[1], int(np.linalg.norm(end - start) / spacing) + 1)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        yaw = math.atan2(dy, dx)
        points = [(xi, yi, yaw) for xi, yi in zip(x, y)]
        length = np.linalg.norm(end - start)
        return points, yaw, length
    
    # 初始化路径列表
    path_points = []

    # 生成第一条直线：起点 -> 定位点1
    segment, yaw1, length1 = generate_segment(np.array(start), np.array(point1), spacing)
    path_points.extend(segment)

    # 生成第二条直线：定位点1 -> 定位点2
    segment, yaw2, length2 = generate_segment(np.array(point1), np.array(point2), spacing)
    path_points.extend(segment)

    # 初始化用于后续计算的变量
    yaws = [yaw1, yaw2]
    lengths = [length1, length2]
    last_end = np.array(point2)

    # 生成后续的螺旋路径
    while length2 > shrink_amount:
        for i in range(2):  # 交替处理两个方向
            if i == 0:
                yaw = yaws[i] + math.pi  # 反方向
                length = length1

                # 计算新的终点位置
                end_x = last_end[0] + length * math.cos(yaw)
                end_y = last_end[1] + length * math.sin(yaw)
                new_end = np.array([end_x, end_y])

                # 生成新的线段
                segment, new_yaw, new_length = generate_segment(last_end, new_end, spacing)
                path_points.extend(segment)
                last_end = new_end
                yaws[i] = new_yaw  # 更新航向角

            elif i == 1:
                yaw = yaws[i] + math.pi  # 反方向
                length = length2 - shrink_amount  # 减少固定的长度
                length2 = length  # 更新长度

                # 计算新的终点位置
                end_x = last_end[0] + length * math.cos(yaw)
                end_y = last_end[1] + length * math.sin(yaw)
                new_end = np.array([end_x, end_y])

                # 生成新的线段
                segment, new_yaw, new_length = generate_segment(last_end, new_end, spacing)
                path_points.extend(segment)
                last_end = new_end
                yaws[i] = new_yaw  # 更新航向角

    # 将数据拆分到单独的列表中
    _indices = []
    cx = []
    cy = []
    cyaw = []

    for idx, (x, y, yaw) in enumerate(path_points):
        _indices.append(idx)
        cx.append(x)
        cy.append(y)
        cyaw.append(yaw)

    return cx, cy, cyaw, _indices

def remap_heading_angle(heading_deg):

    """
    重新映射来自卫星导航系统的航向角。

    参数:
    - heading_deg: float, 原始航向角度，以度为单位，北为 0°,
      顺时针增加到 360°。

    返回:
    - remapped_angle: float, 重新映射后的航向角度，以弧度为单位，北为 0,
      顺时针增加到 π（南），逆时针减少到 -π（南）。
    """
    # 将航向角从度转换为弧度
    heading_rad = math.radians(heading_deg)
    
    # 重新映射角度
    if heading_deg <= 180:
        # 从北 (0°) 顺时针到南 (180°)
        remapped_angle = heading_rad
    else:
        # 从北 (0°) 逆时针到南 (180°)
        remapped_angle = -(2 * math.pi - heading_rad)

    return remapped_angle

def create_can_message_CarSpeed(CarSpeed):
    """
    车速推杆CAN消息创建。

    参数:
    CarSpeed (int): 车速值，范围-64-64。

    返回:
    can.Message: 整合好的CAN消息。
    """
    position = CarSpeed
    if 0 < int(position) <= 128: 
        data = [0xF5, 0x04, 0x58, 0xEF, 0xFF, 0x00, 0x00, 0x00]
        data[5] = (256 - int(position)) & 0xFF
        checksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6]
        data[7] = (checksum + 0xE8) & 0xFF
        message = can.Message(arbitration_id=0x03E8, data=data, is_extended_id=False)
    elif -128 <= int(position) <= 0: 
        data = [0xF5, 0x04, 0x58, 0xEF, 0x00, 0x00, 0x00, 0x00]
        data[5] = (-int(position)) & 0xFF
        checksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6]
        data[7] = (checksum + 0xE8) & 0xFF
        message = can.Message(arbitration_id=0x03E8, data=data, is_extended_id=False)

    return message
def create_can_2_message(di, value):

    # 确定转向
    if di > 0.0001:
        direction = 0x02  # 右转
    elif di < -0.0001:
        direction = 0x01  # 左转
    else:
        direction = 0x00  # 不动
    
    # 构建CAN消息
    data = [(value >> 8) & 0xFF, value & 0xFF, direction, 0x01]  # 确保速度差为正值且符合字节大小
    message = can.Message(arbitration_id=0x9F351F2, data=data, is_extended_id=True)
    
    return message


GOAL_DIS = 0.5  # goal distance
STOP_SPEED = 1 / 3.6  # stop speed

def check_goal(state, goal, tind, nind):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    isstop = (abs(state.v) <= STOP_SPEED)

    if isgoal and isstop:
        return True

    return False

def compute_output(input, range1, range2, output1, output2, min_output, max_output):
    """
    计算输出值，基于两个线性缩放范围和对应的输出值范围。
    
    :param x: 输入值
    :param range1: 第一段线性缩放的输入范围 (start1, end1)
    :param range2: 第二段线性缩放的输入范围 (start2, end2)
    :param output1: 第一段线性缩放的输出值范围 (start_output1, end_output1)
    :param output2: 第二段线性缩放的输出值范围 (start_output2, end_output2)
    :param min_output: 超出 range1[1] 小于 π 时的最小输出值
    :param max_output: 超出 range2[1] 小于 π 时的最大输出值
    :return: 计算的输出值
    """
    if input < range1[0]:
        return min_output
    elif range1[0] <= input <= range1[1]:
        # 线性缩放输出值
        result = output1[0] + (output1[1] - output1[0]) / (range1[1] - range1[0]) * (input - range1[0])
        return result
    elif range2[0] < input <= range2[1]:
        # 线性缩放输出值
        result = output2[0] + (output2[1] - output2[0]) / (range2[1] - range2[0]) * (input - range2[0])
        return result
    elif range2[1] < input <= np.pi:
        return max_output
    else:
        print("输入值应在 0 到 π 之间")

def check_values_same(di, outputs, prev_di, prev_outputs):

    # 如果没有上一次的值，直接通过，并保存当前值
    if prev_di is None or prev_outputs is None:
        prev_di = di
        prev_outputs = outputs
        return True

    # 判断当前 di 和上一次的 di 符号是否相同
    same_sign = (di >= 0 and prev_di >= 0) or (di < 0 and prev_di < 0)
    # 判断当前 outputs 和上一次的 outputs 值是否相同
    same_value = outputs == prev_outputs

    # 通过判断符号相同且值相同
    return same_sign and same_value

def create_weighted_moving_average_filter(window_size):
    data = []
    weights = [i + 1 for i in range(window_size)]  # 加权递增
    total_weight = sum(weights)

    def filter(new_value):
        if len(data) >= window_size:
            data.pop(0)  # 移除最早的元素
        data.append(new_value)
        
        # 计算加权移动平均值
        weighted_sum = sum(data[i] * weights[i] for i in range(len(data)))
        filtered_value = weighted_sum / total_weight
        
        return filtered_value
    
    return filter

def calculate_distance_direction_yaw_error(x, y, yaw, cx, cy, cyaw):
    """
    计算点到参考曲线的最短距离，方向（左为正，右为负），以及航向角偏差。

    参数:
    x (float): 点的x坐标。
    y (float): 点的y坐标。
    yaw (float): 点的航向角（弧度）。
    cx (list of float): 参考曲线的x坐标列表。
    cy (list of float): 参考曲线的y坐标列表。
    cyaw (list of float): 参考曲线的航向角列表（弧度）。

    返回:
    float: 带方向的最短距离（左为正，右为负）。
    float: 点的航向角与曲线上最近点的航向角之间的偏差（弧度）。
    """
    # 将列表转换为Numpy数组以进行向量化操作
    cx = np.array(cx)
    cy = np.array(cy)
    cyaw = np.array(cyaw)

    # 计算点到曲线上每个点的距离
    distances = np.sqrt((cx - x) ** 2 + (cy - y) ** 2)
    min_index = np.argmin(distances)
    min_distance = distances[min_index]

    # 计算方向，使用向量的叉积来确定
    # 从曲线上最近点到点的方向向量
    direction_vector = np.array([x - cx[min_index], y - cy[min_index]])

    # 在曲线上最近点的切线向量
    if min_index == len(cx) - 1:
        tangent_vector = np.array([cx[min_index] - cx[min_index - 1], cy[min_index] - cy[min_index - 1]])
    else:
        tangent_vector = np.array([cx[min_index + 1] - cx[min_index], cy[min_index + 1] - cy[min_index]])

    # 计算切线向量和方向向量的叉积
    cross_prod = np.cross(tangent_vector, direction_vector)

    # 根据叉积的符号确定左正右负
    sign = np.sign(cross_prod)
    directional_distance = min_distance * sign

    # 计算航向角偏差
    yaw_error = yaw - cyaw[min_index]
    # 规范化航向角偏差到 [-π, π]
    yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi  

    return directional_distance, yaw_error

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = cyaw[ind] - math.atan2(dyl, dxl)
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    if angle < 0:
        mind *= -1

    return ind, mind, angle

def passcalc_nearest_index(state, cx, cy, cyaw, last_index=0, window_size=20):
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

    # angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    angle = cyaw[ind] - math.atan2(dyl, dxl)
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    if angle < 0:
        mind *= -1

    return ind, mind, angle


def control_steering(current_yaw, target_yaw, tolerance):
    # 计算航向角偏差
    yaw_error = target_yaw - current_yaw

    # 判断是否达到目标航向角范围内
    if abs(yaw_error) <= tolerance:
        print("Reached target yaw. Stop steering adjustment.")
        direction = 0  # 不动

    # 根据航向角偏差进行左右转控制调整
    # if yaw_error > 0:
    #     # 向左转
    #     print("Steer left to adjust yaw.")
    #     direction = -1  # 左转
    else:
        # 向右转
        print("Steer right to adjust yaw.")
        direction = 1  # 右转
    
    return direction

def calculate_new_point(x, y, yaw_radians, backward_distance, right_distance):
    """
    计算从指定点后方及右侧一定距离的新点的坐标。
    
    参数:
    x (float): 初始点的 x 坐标。
    y (float): 初始点的 y 坐标。
    yaw_radians (float): 当前点的航向角，单位为弧度。
    backward_distance (float): 从初始点后方移动的距离，单位为米。
    right_distance (float): 从初始点右侧移动的距离，单位为米。
    
    返回:
    tuple: 包含新点的 (x, y) 坐标的元组。
    
    描述:
    根据给定的航向角，这个函数计算从初始点向后及向右移动指定距离的新位置。
    航向角 'yaw_radians' 是从正北方向顺时针测量的角度。
    向后移动通过增加π弧度（180度）来调整方向，向右移动通过增加π/2弧度（90度）来调整方向。
    然后使用三角函数计算新的 x 和 y 坐标。
    """
    # 计算后方的新坐标
    backward_yaw = yaw_radians + math.pi
    backward_x = x + backward_distance * math.cos(backward_yaw)
    backward_y = y + backward_distance * math.sin(backward_yaw)

    # 计算右侧的新坐标
    right_yaw = yaw_radians + math.pi / 2
    new_x = backward_x + right_distance * math.cos(right_yaw)
    new_y = backward_y + right_distance * math.sin(right_yaw)

    return new_x, new_y

def map_value(input_value, input_range, output_range, min_value, max_value):
    """
    将输入值从范围 input_range 映射到新的范围 output_range。
    如果输入值小于 input_range 的最小值，则返回最小值 min_value；
    如果输入值大于 input_range 的最大值，则返回最大值 max_value；
    如果输入值在 input_range 之间，则返回线性映射后的值。

    参数:
    input_value (float): 要映射的输入值，应该在 [0, 2] 范围内。
    input_range (tuple): 输入值的范围 (input_min, input_max)。
    output_range (tuple): 输出值的范围 (output_min, output_max)。
    min_value (float): 输入值小于 input_range 的最小值时返回的最小值。
    max_value (float): 输入值大于 input_range 的最大值时返回的最大值。

    返回:
    float: 映射后的值，或者是最小值/最大值。
    """
    
    input_min, input_max = input_range
    output_min, output_max = output_range
    
    # 判断输入值是否小于 input_min
    if input_value < input_min:
        return min_value
    # 判断输入值是否大于 input_max
    elif input_value > input_max:
        return max_value
    
    # 输入值在 input_min 和 input_max 之间，进行线性插值计算映射值
    mapped_value = output_min + ((input_value - input_min) / (input_max - input_min)) * (output_max - output_min)
    
    return int(mapped_value)

def create_header_control_message(action):
    """
    根据输入的操作类型创建控制割台动作的CAN消息。

    参数:
    action (int): 割台动作类型。0表示升起割台，1表示落下割台。

    返回:
    can.Message: 根据指定动作创建的CAN消息对象。
    """

    # 根据输入参数设置value值
    if action == 0:
        value = 1010  # 提升割台的值
    elif action == 1:
        value = 930  # 落下割台的值
    else:
        raise ValueError("Invalid action. Use 0 for lifting and 1 for lowering.")  # 无效的动作参数抛出异常

    # 初始化data数组，初始值为8个字节
    data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    # 将value的高8位存入data[1]
    data[1] = (value >> 8) & 0xFF
    # 将value的低8位存入data[2]
    data[2] = value & 0xFF

    # 创建一个CAN消息对象
    # arbitration_id为0x9F352F2，数据为data数组，使用扩展ID
    message = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)

    # 返回创建的CAN消息
    return message

def outputs_map(di,e,e_th,input_range,output_range,input_range_2,output_range_2,input_range_3,output_range_3,input_range_4,output_range_4,max_output,max_output_2,max_output_3,max_output_4,min_output):
    if 0.06 <= e <= 0.17 and abs(e_th) <= np.radians(5.7):
        outputs = map_value(abs(di), input_range, output_range, min_output, max_output)
    elif 0 <= e <= 0.19 and abs(e_th) <= np.radians(7):
        outputs = map_value(abs(di), input_range_2, output_range_2, min_output, max_output_2)
    elif -0.06<=e <= 0.27:
        outputs = map_value(abs(di), input_range_3, output_range_3, min_output, max_output_3)
    else:
        outputs = map_value(abs(di), input_range_4, output_range_4, min_output, max_output_4)
    return outputs

