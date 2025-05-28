import numpy as np
import matplotlib.pyplot as plt

# 定义车辆类
class Vehicle:
    def __init__(self, x, y, theta, v=1.0):
        self.x = x  # 车辆x坐标
        self.y = y  # 车辆y坐标
        self.theta = theta  # 车辆朝向
        self.v = v  # 车辆速度

    def move(self, steering_angle, dt=0.1):
        # 更新车辆位置，使用前轮转向模型
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += steering_angle * dt

# 纯追踪算法
class PurePursuit:
    def __init__(self, look_ahead_distance=1.0):
        self.look_ahead_distance = look_ahead_distance  # 追踪点的前瞻距离

    def get_steering_angle(self, vehicle, path):
        # 找到距离车辆最近的路径点
        closest_point = self.find_closest_point(vehicle, path)
        # 计算车辆到该点的距离
        dx = closest_point[0] - vehicle.x
        dy = closest_point[1] - vehicle.y
        distance_to_target = np.sqrt(dx**2 + dy**2)

        # 查找追踪点（距离车辆一定距离的点）
        look_ahead_point = self.find_lookahead_point(path, vehicle, self.look_ahead_distance)

        # 计算追踪点相对车辆的角度
        angle = np.arctan2(look_ahead_point[1] - vehicle.y, look_ahead_point[0] - vehicle.x) - vehicle.theta
        # 将角度限制在合理的范围内
        steering_angle = np.arctan2(2.0 * np.sin(angle) / distance_to_target, 1.0)

        return steering_angle

    def find_closest_point(self, vehicle, path):
        # 遍历路径，找到最接近车辆的点
        closest_point = None
        min_distance = float('inf')
        for point in path:
            dx = point[0] - vehicle.x
            dy = point[1] - vehicle.y
            distance = np.sqrt(dx**2 + dy**2)
            if distance < min_distance:
                min_distance = distance
                closest_point = point
        return closest_point

    def find_lookahead_point(self, path, vehicle, look_ahead_distance):
        # 遍历路径，找到与车辆前瞻距离相等的点
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]
            distance = np.sqrt((p2[0] - vehicle.x)**2 + (p2[1] - vehicle.y)**2)
            if distance >= look_ahead_distance:
                # 计算插值，找到前瞻点
                ratio = (look_ahead_distance - np.sqrt((p1[0] - vehicle.x)**2 + (p1[1] - vehicle.y)**2)) / (distance)
                look_ahead_point = [p1[0] + ratio * (p2[0] - p1[0]), p1[1] + ratio * (p2[1] - p1[1])]
                return look_ahead_point
        return path[-1]  # 如果路径不足前瞻距离，返回最后一个点

# 定义路径
path = np.array([[i, np.sin(i / 2)] for i in np.linspace(0, 20, 100)])

# 初始化车辆
vehicle = Vehicle(x=0, y=0, theta=0)

# 初始化纯追踪算法
pure_pursuit = PurePursuit(look_ahead_distance=1.0)

# 存储车辆路径和目标路径
vehicle_trajectory = []
target_trajectory = list(path)

# 模拟车辆运动
for _ in range(500):
    steering_angle = pure_pursuit.get_steering_angle(vehicle, path)
    vehicle.move(steering_angle)

    # 存储位置
    vehicle_trajectory.append([vehicle.x, vehicle.y])

# 绘制路径和车辆轨迹
vehicle_trajectory = np.array(vehicle_trajectory)
plt.plot(path[:, 0], path[:, 1], label="Path", color="blue")
plt.plot(vehicle_trajectory[:, 0], vehicle_trajectory[:, 1], label="Vehicle Trajectory", color="red")
plt.scatter(vehicle.x, vehicle.y, color="green", label="Final Position")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Pure Pursuit Path Tracking")
plt.grid(True)
plt.show()
