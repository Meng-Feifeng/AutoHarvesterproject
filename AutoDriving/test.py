import math

def find_closest_indices(cx, cy, yawChangedIndices, new_x_list, new_y_list):
    closest_indices = []  # 存储每个关键点最近的实际点的索引
    
    # 遍历所有的关键点索引
    for index in yawChangedIndices:
        ref_x = cx[index]  # 获取关键点的 x 坐标
        ref_y = cy[index]  # 获取关键点的 y 坐标
        
        min_distance = float('inf')
        closest_index = -1
        
        # 遍历实际路径的每个点，计算距离
        for i, (x, y) in enumerate(zip(new_x_list, new_y_list)):
            distance = math.sqrt((x - ref_x) ** 2 + (y - ref_y) ** 2)
            
            # 更新最短距离和对应的索引
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # 记录当前关键点最近的实际点索引
        closest_indices.append(closest_index)
    
    return closest_indices

# 示例使用
cx = [1, 2, 3, 4, 5]  # 参考路径的 x 坐标
cy = [1, 2, 3, 4, 5]  # 参考路径的 y 坐标
yawChangedIndices = [0, 2, 4]  # 关键点的索引
new_x_list = [0.9, 2.1, 2.9, 3.8, 4.9]  # 实际路径的 x 坐标
new_y_list = [1.1, 1.9, 3.1, 3.7, 5.0]  # 实际路径的 y 坐标

closest_indices = find_closest_indices(cx, cy, yawChangedIndices, new_x_list, new_y_list)
print(closest_indices)
