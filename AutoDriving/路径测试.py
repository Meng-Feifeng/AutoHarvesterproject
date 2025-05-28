"""
基于回字形路径生成参考点、参考线测试用

"""
import pandas as pd
from pyproj import CRS, Transformer
import cubic_spline_planner
import matplotlib.pyplot as plt
import numpy as np
import User_Function as Fun
import DataBase as DB

def read_excel_to_lists(file_path):
    # 使用pandas读取Excel文件
    df = pd.read_excel(file_path, engine='openpyxl')
    
    # 假设Excel文件中有'longitude'和'latitude'这两列
    # 将这些列的数据转存到列表中
    longitude_list = df['longitude'].tolist()
    latitude_list = df['latitude'].tolist()
    course_list = df['course'].tolist()
    
    return longitude_list, latitude_list, course_list

def transform_coordinates(longitude_list, latitude_list):
    transformer = Transformer.from_crs(CRS("EPSG:4326"), CRS("EPSG:4547"))
    
    transformed_longitude_list = []
    transformed_latitude_list = []
    
    # 遍历列表并进行坐标转换
    for lon, lat in zip(longitude_list, latitude_list):
        x, y = transformer.transform(lat, lon)  # 注意：transform函数首先接收纬度，然后是经度
        
        transformed_longitude_list.append(x)
        transformed_latitude_list.append(y)
    
    return transformed_longitude_list, transformed_latitude_list

def transform_coordinates_course(longitude_list, latitude_list, course_list):
    transformer = Transformer.from_crs(CRS("EPSG:4326"), CRS("EPSG:4547"))
    
    transformed_longitude_list = []
    transformed_latitude_list = []
    transformed_course_list = []
    
    # 遍历列表并进行坐标转换
    for lon, lat, iyaw in zip(longitude_list, latitude_list, course_list):
        x, y = transformer.transform(lat, lon)  # 注意：transform函数首先接收纬度，然后是经度
        yaw = Fun.remap_heading_angle(iyaw)
        
        transformed_longitude_list.append(x)
        transformed_latitude_list.append(y)
        transformed_course_list.append(yaw)
    
    return transformed_longitude_list, transformed_latitude_list, transformed_course_list

def compute_trajectory(x_list, y_list, yaw_list, start_idx, end_idx, step=1):
    line_segments = []
    yaw_changed_indices = []
    should_draw = True
    color_index = 0  # 初始颜色索引

    # 定义一组颜色，用于绘制线条
    colors = ['red', 'green', 'yellow', 'blue', 'purple', 'orange', 'cyan', 'magenta']
    colors = ['red', 'green', 'yellow', 'blue']

    for i in range(start_idx, end_idx):
        if i > start_idx and np.abs(yaw_list[i] - yaw_list[i-1]) > np.pi /6:
            # should_draw = not should_draw
            yaw_changed_indices.append(i)
            color_index = (color_index + 1) % len(colors)  # 更换颜色

        if should_draw and (i - start_idx) % step == 0:
            dx = np.cos(yaw_list[i]) * 1.5
            dy = np.sin(yaw_list[i]) * 1.5
            lx1 = x_list[i] + dx - np.sin(yaw_list[i]) * 1.0
            ly1 = y_list[i] + dy + np.cos(yaw_list[i]) * 1.0
            lx2 = x_list[i] + dx + np.sin(yaw_list[i]) * 1.0
            ly2 = y_list[i] + dy - np.cos(yaw_list[i]) * 1.0
            line_segments.append(((lx1, ly1), (lx2, ly2), colors[color_index]))
    
    return line_segments, yaw_changed_indices

def main():
    _is_show = True
# 0820打点
# 34.29110959,108.06999382
# 34.29122245,108.06999056
# 34.29122003,108.06988061
    longitude1 = [108.06999382, 108.06999056, 108.06988061]
    latitude1 = [34.29110959, 34.29122245, 34.29122003]
    longitude1 = [108.06999410, 108.07002195, 108.06988351]
    latitude1 = [34.29111032, 34.29122274, 34.29124103]
# 34.29092257,108.07177948
# 34.29091898,108.07164599
# 34.29086683,108.07163788
    longitude1 = [108.07177948, 108.07164599, 108.07163788]
    latitude1 = [34.29092257, 34.29091898, 34.29086683]
    x1, y1 = transform_coordinates(longitude1, latitude1)
    # cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(x1, y1, ds=0.1)

    start = (x1[0],y1[0])
    point1 = (x1[1],y1[1])
    point2 = (x1[2],y1[2])
    cx, cy, cyaw, cs = Fun.generate_spiral_path(start, point1, point2, spacing=0.1, shrink_amount=1.8)
    yawChangedInd, change_count = Fun.check_yaw_changes(cx, cy, cyaw, cs)

    # index = 0
    x_1, y_1 = Fun.calculate_new_point(cx[yawChangedInd[0]], cy[yawChangedInd[0]], cyaw[yawChangedInd[0]], 0, 2)
    x_2, y_2 = cx[yawChangedInd[0]], cy[yawChangedInd[0]]
    ax=[x_1, x_2]
    ay=[y_1, y_2]
    cx1, cy1, cyaw1, ck1, s1 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    x_3, y_3 = Fun.calculate_new_point(cx[yawChangedInd[0]], cy[yawChangedInd[0]], cyaw[yawChangedInd[0]], 5, 0.05)
    x_4, y_4 = Fun.calculate_new_point(cx[yawChangedInd[0]], cy[yawChangedInd[0]], cyaw[yawChangedInd[0]], 0, 0.05)
    ax=[x_3, x_4]
    ay=[y_3, y_4]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    x_3_4, y_3_4 = Fun.calculate_new_point(cx[yawChangedInd[0]], cy[yawChangedInd[0]], cyaw[yawChangedInd[0]], 5, -0.05)
    x_4_4, y_4_4 = Fun.calculate_new_point(cx[yawChangedInd[0]], cy[yawChangedInd[0]], cyaw[yawChangedInd[0]], 0, -0.05)
    ax=[x_3_4, x_4_4]
    ay=[y_3_4, y_4_4]
    cx2_4, cy2_4, cyaw2_4, ck2_4, s2_4 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    x_3_5, y_3_5 = Fun.calculate_new_point(cx[yawChangedInd[1]], cy[yawChangedInd[1]], cyaw[yawChangedInd[1]], 0, 2)
    x_4_5, y_4_5 = cx[yawChangedInd[1]], cy[yawChangedInd[1]]
    ax=[x_3_5, x_4_5]
    ay=[y_3_5, y_4_5]
    cx2_5, cy2_5, cyaw2_5, ck2_5, s2_5 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    x_5, y_5 = Fun.calculate_new_point(cx[yawChangedInd[0]], cy[yawChangedInd[0]], cyaw[yawChangedInd[0]], 0, 5)
    x_6, y_6 = cx[0], cy[0]
    ax=[x_6, x_5]
    ay=[y_6, y_5]
    cx3, cy3, cyaw3, ck3, s3 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    index = 1
    x_7, y_7 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], cyaw[yawChangedInd[index]], 5, 0)
    x_8, y_8 = Fun.calculate_new_point(cx[yawChangedInd[index+1]], cy[yawChangedInd[index+1]], cyaw[yawChangedInd[index+1]], 0, 3)
    ax=[x_7, x_8]
    ay=[y_7, y_8]
    cx4, cy4, cyaw4, ck4, s4 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    x_9, y_9 = Fun.calculate_new_point(cx[yawChangedInd[index-1]], cy[yawChangedInd[index-1]], cyaw[yawChangedInd[index-1]], 5, 0)
    x_10, y_10 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], cyaw[yawChangedInd[index]], 0, 3)
    ax=[x_9, x_10]
    ay=[y_9, y_10]
    cx5, cy5, cyaw5, ck5, s5 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    x_11, y_11 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 0, DB.Forward_limit_distance+1)
    x_111, y_111 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 0.2, DB.Forward_limit_distance)
    x_112, y_112 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 0.8, DB.Forward_limit_distance/2)
    x_113, y_113 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 1.5, DB.Forward_limit_distance/4)
    x_114, y_114 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 2.5, DB.Forward_limit_distance/8)
    x_115, y_115 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 5, 0)
    x_116, y_116 = Fun.calculate_new_point(cx[yawChangedInd[index]], cy[yawChangedInd[index]], 
                                         cyaw[yawChangedInd[index]], 6, 0)
    ax=[x_11,x_111, x_112, x_113, x_114, x_115, x_116]
    ay=[y_11,y_111, y_112, y_113, y_114, y_115, y_116]
    cx6, cy6, cyaw6, ck6, s6 = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    # 绘制结果
    if _is_show:
        # 配置字体为 SimHei（黑体）
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.rcParams['axes.unicode_minus'] = False  # 解决负号无法显示的问题

        # 全局设置字体大小
        plt.rcParams.update({
            'font.size': 24,               # 基本字体大小
            'axes.titlesize': 30,          # 标题字体大小
            'axes.labelsize': 26,          # 坐标轴标签字体大小
            'xtick.labelsize': 24,         # X轴刻度字体大小
            'ytick.labelsize': 24,         # Y轴刻度字体大小
            'legend.fontsize': 24,         # 图例字体大小
        })
        
        plt.figure()
        plt.plot(cx, cy, label='规划路径', color='blue', linewidth=1)
        plt.scatter([cx[0], cx[-1]], [cy[0], cy[-1]], color='blue', label='起点、终点')
        plt.scatter([cx[i] for i in yawChangedInd], [cy[i] for i in yawChangedInd], color='red', label='转弯点')

        # plt.plot(cx1, cy1, color='red', label='前向参考路径')
        # plt.plot(cx2, cy2, color='green', label='后向参考路径')
        # plt.plot(cx2_4, cy2_4, color='purple', label='前向参考路径')
        # plt.plot(cx2_5, cy2_5, color='yellow', label='前向参考路径')

        # plt.plot(cx3, cy3, color='red', label='前向路径')
        # plt.plot(cx4, cy4, color='y', label='前向路径')
        # plt.plot(cx5, cy5, color='g', label='后向路径')

        # plt.plot(cx6, cy6, color='red', label='测试路径')

        plt.legend()
        plt.xlabel('X 轴  →正向为北')
        plt.ylabel('Y 轴  →正向为西')
        plt.title('路径可视化')
        plt.grid(True)
        plt.axis('equal')  # 保持X和Y的刻度一致
        plt.show()

if __name__ == "__main__":
    main()

