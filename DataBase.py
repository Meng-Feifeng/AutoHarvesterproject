# DataBase.py
data = 0  # 示例全局变量

# 实时经纬高，速度，航向
RT_longitude = 0
RT_latitude = 0
RT_altitude = 0

RT_speed = 0
RT_course = 0
RT_course_f = 0

RT_x = 0
RT_y = 0

# import threading
# data_lock = threading.Lock()

rlongitude = 0
rlatitude = 0

RT_left = 0
RT_right = 0

x_axis_acc = 0              # X 轴加速度
y_axis_acc = 0              # Y 轴加速度
z_axis_acc = 0              # Z 轴加速度
x_axis_angle = 0            # X 轴矫正后角度
y_axis_angle = 0            # Y 轴矫正后角度
z_axis_angle = 0            # Z 轴矫正后角度
x_axis_euler = 0            # X 轴欧拉角度
x_axis_euler = 0            # Y 轴欧拉角度
inter_tempe = 0             # 内部温度
install_location = 0        # 设备安装方位号：0~23 代表 1~24 种方向
route_direction = 0         # 车辆中线方向
GPS_heading = 0             # GPS 航向角（源自 GPHDT）
heading_offset = 0          # 航向偏移
stop_heading_offset = 0     # 停车航向偏移
IMU_ini_state = 0           # IMU 初始化状态：0 --- 未开始初始化，1 --- 初始化中，2 --- 初始化完成
fusion_heading = 0          # 融合航向角度偏差（GPS 航向与车辆航向）

z_axis_pals = 0


# 导航线拟合
cx= []
cy= []
cyaw= [] 
ck= []
s = []
spacing = 0.2           # 生成路径离散点间距
shrinkAmount = 1.8      # 路径间距
fitClickCount = 0
yawChangedIndices = []

start_cx = []
start_cy = []
start_cyaw = []
start_ck = []
start_s = []
end_cx = []
end_cy = []
end_cyaw = []
end_ck = []
end_s = []


# 自动驾驶
deviation = 0
thresholdDistance_1 = 0.2
thresholdDistance_2 = 2.5
thresholdDistance_3 = 1.5
reverseSpeed = -40
forwardSpeed = 45
forwardLowSpeed = 40
leftValveOpening = 0
rightValveOpening = 0
limit_distance_0_2 = 0.2
limit_distance_0_5 = 0.5
limit_distance_1 = 1
limit_distance_2 = 2
limit_distance_2_5 = 2.5
limit_distance_1_5 = 1.5
limit_distance_3 = 3

allowable_deviation = 0.03      # 消除0附近的控制抖动，减少左右摇摆和震荡
range1 = (0.01, 0.3)
output1 = (510, 600)
range2 = (0.3, 1.8)
output2 = (660, 680)
min_output = 500
max_output = 700
range1_1 = (0.02, 0.8)
output1_1 = (530, 600)
range2_1 = (0.8, 1.4)
output2_1 = (600, 680)
min_output_1 = 500
max_output_1 = 700

gain = 2
gain2 = 1
input_range = (0.01, 1.8)
output_range = (510, 680)
input_range_2 = (0.01, 1.9)
output_range_2 = (650, 670)
input_range_3 = (0.01, 1.9)
output_range_3 = (660, 680)
input_range_4 = (0.01, 1.9)
output_range_4 = (610, 680)
min_output_2 = 500
max_output_2 = 675
min_output_3 = 500
max_output_3 = 685
min_output_4 = 500
max_output_4 = 690
forward_back_flag = True
Forward_limit_distance = 0.6
Back_limit_distance = 2.5

tolerance = 0.05        # 倒车航向角控制范围
toleranceyaw = 0.1
backValveOpen = 700
TurnValveOpen_700 = 700
TurnValveOpen_680 = 680
TurnValveOpen_676 = 676
TurnValveOpen_670 = 670



final_motion = 0.1

# 自动驾驶状态参数
target_ind = 0
last_ind = 0
target_mind = 0

# 割台参数
header_up = 0
header_down = 0

# 轨迹记录
record_longitude_list = []
record_latitude_list = []
record_speed_list = []
record_course_list = []
record_deviation_list = []
record_eth_deviation_list = []
record_time1_list = []
record_time2_list = []
record_left_right = []
flag_left = False
flag_right = False

record_left_speed_list = []
record_right_speed_list = []

record_reallongitude_list = []
record_reallatitude_list = []

control_state_time_list = []
control_state_output_list = []
control_state_di_list = []

record_z_axis_angle_list = []
record_z_axis_pals_list = []
record_setspont_list = []

record_lateraldeviation_list = []
record_coursedeviation_list = []

# 运行标记
flag_TrackRecord = False     # 轨迹记录开关
flag_Auto = False            # 自动驾驶暂停开关
flag_goal = False 

prev_di = 0
prev_outputs = None

header_flag = False

start_cx_cy_flag = False
end_cx_cy_flag = False

# 车速测试
TargetCarSpeed = 0

# 天线位置变换
lateral_offset = 0.2
longitudinal_offset = 2

# PI 控制器参数
K_p = 1270.0       # 比例增益
K_i = 0.0       # 积分增益
K_d = 100.0

u_min = 500             # 输出限制
u_max = 680

setpoint = 0