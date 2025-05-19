"""
604
增加Windows平台识别。
Win系统下需插入PCAN设备(USBtoCAN)。
程序统一一个CAN总线bus，不必每个线程创建一次。

从偏差小于阈值修改为输出小于阈值为零。
修改最小输出起点变更，为550。
增加输出值和上一次输出值异号时，强制输出为零。
增加轮速检测和记录。

611
回字形路径，右后转弯调头
修改为后退转弯航向控制，增加后退直线跟踪，前进上线准备
增加终点控制

612
路径跟踪使用617G参数方案

824
取消后退准备
输出控制改为单范围4条
0.05 <= e <= 0.12 --> 0.04 <= e <= 0.13
增加割台控制标志 header_flag，增加复选框控件控制标记置位
控制输出记录
修改换线模式，前进用前进参考路径，后退用后退参考路径，起点终点路径单独列出
增加PID控制器
后退完整路径跟踪

"""

import os
import sys
import platform
import serial
import serial.tools.list_ports
import time
# import pynmea2
import can
import threading

import numpy as np
import cubic_spline_planner
import User_Function as Fun
import stanley_controller as Stan
import simulation_pyCAN as Simu
from datetime import datetime,timedelta,timezone
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from UI_824 import Ui_MainWindow  # 导入ui.py 中的 Ui_MaiuinWindow 界面类
from pyproj import CRS, Transformer
from collections import deque
import pandas as pd
import DataBase as DB

class MainWindow(QMainWindow, Ui_MainWindow):  # 继承 QMainWindow 类和 Ui_MainWindow 界面类
    def __init__(self,bus):
        super().__init__()
        self.setupUi(self)  # 继承 Ui_MainWindow 界面类
        if platform.system() == 'Linux':
            self.showMaximized()
        self.bus = bus
        self.count = 0

        self.pushButton_serial_GNSS.clicked.connect(self.click_pushButton_GNSS)
        self.pushButton_AutoDriving.clicked.connect(self.click_pushButton_AutoDriving)
        self.pushButton_AutoSuspend.clicked.connect(self.click_pushButton_AutoSuspend)
        self.pushButton_fit.clicked.connect(self.click_pushButton_Fit_navigation_line)
        self.pushButton_TrackRecord.clicked.connect(self.click_pushButton_TrackRecord)
        self.pushButton_valve_Confirm.clicked.connect(self.click_pushButton_valve_Confirm)
        self.pushButton_recordpoint.clicked.connect(self.click_pushButton_recordpoint)

        self.pushButton_CAN.clicked.connect(self.click_pushButton_CAN)
        self.pushButton_header_up.clicked.connect(self.click_pushButton_header_up)
        self.pushButton_header_down.clicked.connect(self.click_pushButton_header_down)

        self.pushButton_Reel_UP.pressed.connect(self.pressed_pushButton_Reel_UP)
        self.pushButton_Reel_UP.released.connect(self.released_pushButton_Reel_UP)
        self.pushButton_Reel_Down.pressed.connect(self.pressed_pushButton_Reel_Down)
        self.pushButton_Reel_Down.released.connect(self.released_pushButton_Reel_Down)
        self.checkBox_headerControl.stateChanged.connect(self.checkbox_header_control_changed)

        # 创建转换器
        crs_wgs = CRS("EPSG:4326")
        crs_gauss_kruger = CRS("EPSG:4547")
        self.transformer = Transformer.from_crs(crs_wgs, crs_gauss_kruger)

        # 创建工作线程
        self.workerautodriving = Worker_AutoDriving(self.bus)
        self.can_thread = Worker_CAN(self.bus)


#region pushButton CAN operation

    def checkbox_header_control_changed(self, state):
        if state == 2:  # 2 表示复选框被勾选
            DB.header_flag = True
            print("割台自动状态")
        else:  # 0 表示复选框未被勾选
            DB.header_flag = False
            print("割台手动状态")

    def pressed_pushButton_Reel_UP(self):
        # 拨禾轮升按钮按下时执行的代码
        data = [0x01, 0x01, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)
        self.bus.send(message)
        
    def released_pushButton_Reel_UP(self):
        # 拨禾轮升按钮释放时执行的代码
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)
        self.bus.send(message)

    def pressed_pushButton_Reel_Down(self):
        # 拨禾轮降按钮按下时执行的代码
        data = [0x01, 0x02, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)
        self.bus.send(message)
        
    def released_pushButton_Reel_Down(self):
        # 拨禾轮降按钮释放时执行的代码
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)
        self.bus.send(message)

    def click_pushButton_header_up(self):
            value = 1010
            if 0 <= int(value) <= 10000: 
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = (int(value) >> 8) & 0xFF
                data[2] = int(value) & 0xFF
                message = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)
                self.bus.send(message)
            else:
                print("数字不对")

    def click_pushButton_header_down(self):
            value = 930
            if 0 <= int(value) <= 10000: 
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = (int(value) >> 8) & 0xFF
                data[2] = int(value) & 0xFF
                message = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)
                self.bus.send(message)
            else:
                print("数字不对")

    def click_pushButton_CAN(self):
        if self.can_thread.isRunning():
            self.can_thread.stop()
            self.can_thread.wait()
            self.pushButton_CAN.setText('CAN接收关')
        else:
            self.can_thread = Worker_CAN(self.bus)
            self.can_thread.message_received.connect(self.handle_message)
            self.can_thread.start()
            self.pushButton_CAN.setText('CAN接收开')

    def handle_message(self, msg):      # CAN消息接收处理
        if(msg.arbitration_id == 0x9F363D2):
            ADC1_value = (msg.data[0] << 8) + msg.data[1]
            ADC2_value = (msg.data[2] << 8) + msg.data[3]
            ADC3_value = (msg.data[4] << 8) + msg.data[5]
            self.label_ADC.setText(f"ADC1:{ADC1_value}\nADC2:{ADC2_value}\nADC3:{ADC3_value}")

        elif(msg.arbitration_id == 0x11111):
            self.label_ADC.setText(f"CAN超时")

    def closeEvent(self, event):
        if self.can_thread.isRunning():
            self.can_thread.stop()
            self.can_thread.wait()
        if self.bus:
            self.bus.shutdown()
        super().closeEvent(event)

#endregion

#region pushButton GNSS operation

    def click_pushButton_GNSS(self):  # 点击 pushButton_serial_GNSS 触发
        try:
            if self.pushButton_serial_GNSS.text() == "串口关":
                self.label_serial_state.setText("串口状态:1")
                self.pushButton_serial_GNSS.setText('串口开')
                self.gnss_start()
            else:
                self.label_serial_state.setText("串口状态:0")
                self.pushButton_serial_GNSS.setText('串口关')
                self.gnss_stop()
        except KeyboardInterrupt:
            print("程序已停止")
    
            print("程序已停止")

    def click_pushButton_Fit_navigation_line(self):
        try:
            if(len(self.lineEdit_pointA.text())!=0 and len(self.lineEdit_pointB.text())!=0 and len(self.lineEdit_pointC.text())==0):
                latitude1, longitude1 = map(float, self.lineEdit_pointA.text().split(','))
                latitude2, longitude2 = map(float, self.lineEdit_pointB.text().split(','))
                x1, y1 = self.transformer.transform(latitude1, longitude1)
                x2, y2 = self.transformer.transform(latitude2, longitude2)
                ax = [x1, x2]
                ay = [y1, y2]
                DB.cx,DB.cy,DB.cyaw,DB.ck,DB.s =cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
                # print(DB.cx,DB.cy,DB.cyaw)
                if DB.fitClickCount != 0:
                    self.label_Fit_flag.setText(f"已拟合直线{DB.fitClickCount}")
                else:
                    self.label_Fit_flag.setText("已拟合直线")
                DB.fitClickCount += 1

            elif(len(self.lineEdit_pointA.text())!=0 and len(self.lineEdit_pointB.text())!=0 and len(self.lineEdit_pointC.text())!=0):
                # 解析输入并进行坐标转换
                latitude1, longitude1 = map(float, self.lineEdit_pointA.text().split(','))
                latitude2, longitude2 = map(float, self.lineEdit_pointB.text().split(','))
                latitude3, longitude3 = map(float, self.lineEdit_pointC.text().split(','))

                x1, y1 = self.transformer.transform(latitude1, longitude1)
                x2, y2 = self.transformer.transform(latitude2, longitude2)
                x3, y3 = self.transformer.transform(latitude3, longitude3)

                pointA = (x1, y1)
                pointB = (x2, y2)
                pointC = (x3, y3)

                DB.cx,DB.cy,DB.cyaw,DB.s = Fun.generate_spiral_path(pointA, pointB, pointC,
                                            spacing = DB.spacing, shrink_amount = DB.shrinkAmount)
                # DB.cx,DB.cy = Fun.smooth_path(DB.cx, DB.cy, alpha=0.1, beta=0.1, iterations=1)
                DB.ck = [0]*len(DB.cx)

                DB.yawChangedIndices, change_count = Fun.check_yaw_changes(DB.cx, DB.cy, DB.cyaw, DB.s)
                print(f"共{change_count}个转弯点")

                # 起点直线
                cx_1, cy_1 = DB.cx[0], DB.cy[0]
                cx_2, cy_2 = Fun.calculate_new_point(DB.cx[DB.yawChangedIndices[0]], DB.cy[DB.yawChangedIndices[0]], 
                                                            DB.cyaw[DB.yawChangedIndices[0]], 0, 5)
                ax = [cx_1, cx_2]
                ay = [cy_1, cy_2]
                DB.start_cx, DB.start_cy, DB.start_cyaw, DB.start_ck, DB.start_s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
                DB.start_cx_cy_flag = True
                print("起点直线已拟合")

                # 终点直线
                distance = np.hypot(DB.cx[DB.yawChangedIndices[-1]] - DB.cx[-1], DB.cy[DB.yawChangedIndices[-1]] - DB.cy[-1])
                if distance > 0.8:          # 车辆幅宽2m，取1.8m有效作业宽度，取一半0.9再减一点
                    DB.end_cx_cy_flag = True
                    # 最后一条路径，后5米前2米两个点
                    goal_cx_1, goal_cy_1 = Fun.calculate_new_point(DB.cx[DB.yawChangedIndices[-1]], DB.cy[DB.yawChangedIndices[-1]], 
                                                                   DB.cyaw[DB.yawChangedIndices[-1]], 5, 0)
                    goal_cx_2, goal_cy_2 = Fun.calculate_new_point(DB.cx[-1], DB.cy[-1], DB.cyaw[-1], -2, 0)
                    x4=[goal_cx_1, goal_cx_2]
                    y4=[goal_cy_1, goal_cy_2]
                    DB.end_cx, DB.end_cy, DB.end_cyaw, DB.end_ck, DB.end_s = cubic_spline_planner.calc_spline_course(x4, y4, ds=0.1)
                    print("终点直线已拟合")
                else:
                    DB.end_cx_cy_flag = False
                    print("终点直线不存在")

                if DB.fitClickCount != 0:
                    self.label_Fit_flag.setText(f"已拟合曲线{DB.fitClickCount}")
                else:
                    self.label_Fit_flag.setText("已拟合曲线")
                DB.fitClickCount += 1
            else:
                self.label_Fit_flag.setText("未拟合")
                DB.fitClickCount = 0

        except ValueError as e:
            print("Error in input format:", e)
    
    def click_pushButton_recordpoint(self):
        if self.count == 0:
            self.lineEdit_pointA.setText(f"{DB.RT_altitude},{DB.RT_longitude}")
            self.count+=1
        elif self.count == 1:
            self.lineEdit_pointB.setText(f"{DB.RT_altitude},{DB.RT_longitude}")
            self.count+=1
        elif self.count == 2:
            self.lineEdit_pointC.setText(f"{DB.RT_altitude},{DB.RT_longitude}")
            self.count=0

#endregion

#region pushButton AutoDriving

    def click_pushButton_valve_Confirm(self):
        # 参数配置
        try:
            text = self.lineEdit_valve_parameter1.text()
            var1, var2, var3, var4, var5, var6 = text.split(',')
            DB.input_range = (float(var1), float(var2))
            DB.output_range = (float(var3), float(var4))
            DB.min_output = (int(var5))
            DB.max_output = (int(var6))
            text = self.lineEdit_valve_parameter2.text()
            var1, var2, var3, var4, var5, var6 = text.split(',')
            DB.input_range_2 = (float(var1), float(var2))
            DB.output_range_2 = (float(var3), float(var4))
            DB.min_output_2 = (int(var5))
            DB.max_output_2 = (int(var6))
            text = self.lineEdit_valve_parameter3.text()
            var1, var2, var3, var4, var5, var6 = text.split(',')
            DB.input_range_3 = (float(var1), float(var2))
            DB.output_range_3 = (float(var3), float(var4))
            DB.min_output_3 = (int(var5))
            DB.max_output_3 = (int(var6))
            text = self.lineEdit_valve_parameter4.text()
            var1, var2, var3, var4, var5, var6 = text.split(',')
            DB.input_range_4 = (float(var1), float(var2))
            DB.output_range_4 = (float(var3), float(var4))
            DB.min_output_4 = (int(var5))
            DB.max_output_4 = (int(var6))
            text = self.lineEdit_valve_parameter5.text()
            DB.Forward_limit_distance = float(text)
            text = self.lineEdit_valve_parameter6.text()
            DB.Back_limit_distance = float(text)


        except:
            print('转换出错')
        

    def click_pushButton_AutoDriving(self):

        if self.workerautodriving.isRunning() :
            self.workerautodriving.stop()  # 停止线程
            self.workerautodriving.wait()  # 等待线程真正结束
            self.pushButton_AutoDriving.setText("自动驾驶关")
            self.label_AutoPrompt.setText("驾驶停止")
            self.pushButton_AutoSuspend.setText("自动驾驶暂停")
            message = Fun.create_can_2_message(0,500)
            self.bus.send(message)
            message = Fun.create_can_message_CarSpeed(0)
            self.bus.send(message)

            DB.flag_TrackRecord = False
            self.pushButton_TrackRecord.setText('轨迹记录关')
            self.trackRecord()
        else:
            self.workerautodriving = Worker_AutoDriving(self.bus)  # 重新创建线程对象
            self.workerautodriving.update_signal.connect(self.endPrompt_Display)
            self.workerautodriving.target_ind_signal.connect(self.Prompt_Display)
            self.workerautodriving.control_state_signal.connect(self.ConState_Display)
            self.workerautodriving.Auto_unexpected_shutdown_signal.connect(self.Unexpected_Shutdown)
            self.workerautodriving.Auto_finished_signal.connect(self.Autodriving_finished)
            self.workerautodriving.start()  # 启动线程
            self.pushButton_AutoDriving.setText("自动驾驶开")
            self.label_AutoPrompt.setText("自动驾驶中")
            self.pushButton_AutoSuspend.setText("自动驾驶中")

            DB.flag_TrackRecord = True
            self.pushButton_TrackRecord.setText('轨迹记录开')

    def click_pushButton_AutoSuspend(self):
        if self.workerautodriving.isRunning() and DB.flag_Auto:
            DB.flag_Auto = False
            self.pushButton_AutoSuspend.setText("自动驾驶暂停")
            message = Fun.create_can_2_message(0,500)
            self.bus.send(message)
        elif self.workerautodriving.isRunning() and DB.flag_Auto == False:
            DB.flag_Auto = True
            self.pushButton_AutoSuspend.setText("自动驾驶中")
        else:
            self.pushButton_AutoSuspend.setText("自动驾驶暂停")

    def Prompt_Display(self, ind, dist):     
        self.label_index_num.setText(f"索引：{ind},距离：{dist:.3f}")
    
    def ConState_Display(self, di, output, lateral, course):
        self.label_AutoState.setText(f"控制状态：{(di):.3f},{output}")
        DB.control_state_time_list.append(datetime.now())
        DB.control_state_di_list.append(di)
        DB.control_state_output_list.append(output)
        DB.record_lateraldeviation_list.append(lateral)
        DB.record_coursedeviation_list.append(course)

    def endPrompt_Display(self, value):     
        self.label_AutoPrompt.setText(f"驾驶结束{value}")

    def Unexpected_Shutdown(self):
        # 程序意外结束或崩溃
        # 控制结束，车辆停止
        message = Fun.create_can_2_message(0,500)
        self.bus.send(message)
        message = Fun.create_can_message_CarSpeed(0)
        self.bus.send(message)

        self.pushButton_AutoDriving.setText("自动驾驶关")
        self.label_AutoPrompt.setText("驾驶停止")
        self.pushButton_AutoSuspend.setText("自动驾驶暂停")
        
        DB.flag_TrackRecord = False
        self.pushButton_TrackRecord.setText('轨迹记录关')
        self.trackRecord_error()

    def Autodriving_finished(self):
        self.pushButton_AutoDriving.setText("自动驾驶关")
        self.label_AutoPrompt.setText("驾驶停止")
        self.pushButton_AutoSuspend.setText("自动驾驶暂停")
#endregion

#region pushButton Record operation

    def click_pushButton_TrackRecord(self):
        if self.pushButton_TrackRecord.text() == "轨迹记录关":
            DB.flag_TrackRecord = True
            self.pushButton_TrackRecord.setText('轨迹记录开')
        else:
            DB.flag_TrackRecord = False
            self.pushButton_TrackRecord.setText('轨迹记录关')

            df1 = pd.DataFrame({'Time': DB.record_time1_list, 'longitude': DB.record_longitude_list, 'latitude':DB.record_latitude_list,
                                'left-right':DB.record_left_right,'speed':DB.record_speed_list,
                                'course':DB.record_course_list,'deviation':DB.record_deviation_list,
                                'ethdeviation':DB.record_eth_deviation_list, 'rlongitude': DB.record_reallongitude_list, 
                                'rlatitude':DB.record_reallatitude_list, 'z axis angle':DB.record_z_axis_angle_list, 
                                'z axis pals':DB.record_z_axis_pals_list, 'left speed':DB.record_left_speed_list,
                                'right speed':DB.record_right_speed_list, 'lateral_dev':DB.record_lateraldeviation_list,
                                'course_dev':DB.record_coursedeviation_list})
            current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
            parameter1 = self.lineEdit_valve_parameter1.text()
            parameter2 = self.lineEdit_valve_parameter2.text()
            parameter3 = self.lineEdit_valve_parameter3.text()
            parameter4 = self.lineEdit_valve_parameter4.text()
            parameter5 = self.lineEdit_valve_parameter5.text()
            parameter6 = self.lineEdit_valve_parameter6.text()
            file1_name = f"824S_{current_time}.xlsx"

            current_dir = os.path.dirname(os.path.abspath(__file__))  
            out_dir = os.path.join(current_dir, "out")  
            if not os.path.exists(out_dir):  
                os.makedirs(out_dir)  
            full_file1_path = os.path.join(out_dir, file1_name) 
            df1.to_excel(full_file1_path, index=False, engine='openpyxl')

            df2 = pd.DataFrame({'Time': DB.control_state_time_list, 'control-state': DB.control_state_di_list, 
                                    'output':DB.control_state_output_list})
            file2_name = f"824S_{current_time}_control.xlsx"
            full_file2_path = os.path.join(out_dir, file2_name) 
            df2.to_excel(full_file2_path, index=False, engine='openpyxl')

            DB.record_longitude_list = []
            DB.record_latitude_list = []
            DB.record_left_right = []
            DB.record_speed_list = []
            DB.record_course_list = []
            DB.record_deviation_list = []
            DB.record_eth_deviation_list = []
            DB.record_reallongitude_list = []
            DB.record_reallatitude_list = []
            DB.record_time1_list = []

            DB.control_state_time_list = []
            DB.control_state_di_list = []
            DB.control_state_output_list = []

            DB.record_z_axis_angle_list = []
            DB.record_z_axis_pals_list = []
            DB.record_left_speed_list = []
            DB.record_right_speed_list = []
            DB.record_lateraldeviation_list = []
            DB.record_coursedeviation_list = []

    def trackRecord(self):
        df1 = pd.DataFrame({'Time': DB.record_time1_list, 'longitude': DB.record_longitude_list, 'latitude':DB.record_latitude_list,
                            'left-right':DB.record_left_right,'speed':DB.record_speed_list,
                            'course':DB.record_course_list,'deviation':DB.record_deviation_list,
                            'ethdeviation':DB.record_eth_deviation_list, 'rlongitude': DB.record_reallongitude_list, 
                            'rlatitude':DB.record_reallatitude_list, 'z axis angle':DB.record_z_axis_angle_list, 
                            'z axis pals':DB.record_z_axis_pals_list, 'left speed':DB.record_left_speed_list,
                            'right speed':DB.record_right_speed_list, 'lateral_dev':DB.record_lateraldeviation_list,
                            'course_dev':DB.record_coursedeviation_list})
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        parameter1 = self.lineEdit_valve_parameter1.text()
        parameter2 = self.lineEdit_valve_parameter2.text()
        parameter3 = self.lineEdit_valve_parameter3.text()
        parameter4 = self.lineEdit_valve_parameter4.text()
        parameter5 = self.lineEdit_valve_parameter5.text()
        parameter6 = self.lineEdit_valve_parameter6.text()
        file1_name = f"824_{current_time}_para{parameter5}_{parameter6}.xlsx"

        current_dir = os.path.dirname(os.path.abspath(__file__))  
        out_dir = os.path.join(current_dir, "out")  
        if not os.path.exists(out_dir):  
            os.makedirs(out_dir)  
        full_file1_path = os.path.join(out_dir, file1_name) 
        df1.to_excel(full_file1_path, index=False, engine='openpyxl')

        df2 = pd.DataFrame({'Time': DB.control_state_time_list, 'control-state': DB.control_state_di_list, 
                                'output':DB.control_state_output_list})
        file2_name = f"824_{current_time}_control.xlsx"
        full_file2_path = os.path.join(out_dir, file2_name) 
        df2.to_excel(full_file2_path, index=False, engine='openpyxl')

        DB.record_longitude_list = []
        DB.record_latitude_list = []
        DB.record_left_right = []
        DB.record_speed_list = []
        DB.record_course_list = []
        DB.record_deviation_list = []
        DB.record_eth_deviation_list = []
        DB.record_reallongitude_list = []
        DB.record_reallatitude_list = []
        DB.record_time1_list = []

        DB.control_state_time_list = []
        DB.control_state_di_list = []
        DB.control_state_output_list = []

        DB.record_z_axis_angle_list = []
        DB.record_z_axis_pals_list = []
        DB.record_left_speed_list = []
        DB.record_right_speed_list = []
        DB.record_lateraldeviation_list = []
        DB.record_coursedeviation_list = []


#endregion

#region GNSS and Serial operation

    def gnss_start(self):
        try:
            self.gnss_work = Worker_gnss()
            self.gnss_work.finished_signal.connect(self.gnss_worker_finished)
            self.gnss_work.mistake_message_transmit.connect(self.mistake_message_show)
            self.gnss_work.ggameg_signal.connect(self.gnss_ggainformation_show)
            self.gnss_work.imumeg_signal.connect(self.gnss_imuinformation_show)
            self.gnss_work.start()

        except serial.serialutil.SerialException as error:
            self.mistake_message_show(error)
    
    def gnss_stop(self):
        if hasattr(self, 'gnss_work') and self.gnss_work is not None:
            self.gnss_work.should_run = False
            self.gnss_work.quit()
            self.gnss_work.wait()
        # if hasattr(self, 'serial_work') and self.serial_work is not None:
        #     self.serial_work.should_run = False
        #     self.serial_work.quit()
        #     self.serial_work.wait()

    def gnss_worker_finished(self):
        self.gnss_work = None

    def serial_worker_finished(self):
        self.serial_work = None

    def mistake_message_show(self, message):
        QMessageBox.information(self, "错误", "错误原因{}".format(message))
    
    def gnss_ggainformation_show(self, ggainformation):
        try:
            items = ggainformation.split(',')

            # GPS时间起始点，1980年1月6日
            gps_epoch = datetime(1980, 1, 6, tzinfo=timezone.utc)
            # 计算总秒数
            total_seconds = int(items[1]) * 7 * 24 * 3600 + float(items[2])
            # GPS时间和UTC时间之间的已知差异，最新情况需要确认
            leap_seconds = 18  # 当前已知的GPS和UTC时间差为18秒
            # 计算UTC时间，减去闰秒差
            utc_time = gps_epoch + timedelta(seconds=(total_seconds - leap_seconds))
            # 如果需要转换为当地时间（例如北京时间），添加时区转换
            local_time = utc_time.astimezone(timezone(timedelta(hours=8)))  # 修改为所需时区
            formatted_time = local_time.strftime("%m-%d %H:%M:%S")

            self.label_UTCTime.setText(f"时间：{formatted_time}")
            self.label_lon.setText(f"经度：{items[4]}")
            self.label_lat.setText(f"纬度：{items[3]}")
            self.label_alt.setText(f"海拔：{items[5]}")
            self.label_speed.setText(f"速度：{(float(items[6])*3.6):.2f} km/h")
            self.label_cource.setText(f"航向：{items[7]} °")
            self.label_positionmode.setText(f"定位模式：{items[10]}")

            if len(DB.cx) != 0:
                e, e_th = Fun.calculate_distance_direction_yaw_error(DB.RT_x, DB.RT_y, Fun.remap_heading_angle(DB.RT_course), DB.cx, DB.cy, DB.cyaw)
                self.label_error.setText(f"偏差：{e:.3f},{e_th:.3f}")
            else:
                self.label_error.setText(f"偏差")
        except:
            print("定位数据有误")

    def gnss_imuinformation_show(self, imuinformation):
        try:
            items = imuinformation.split(',')
            # self.label_IMU_X_a.setText(f"X轴加速度：{items[1]}")
            # self.label_IMU_Y_a.setText(f"Y轴加速度：{items[2]}")
            # self.label_IMU_Z_a.setText(f"Z轴加速度：{items[3]}")
            # self.label_X_d.setText(f"X角度：{items[4]}")
            # self.label_Y_d.setText(f"Y角度：{items[5]}")
            self.label_Z_d.setText(f"Z角度：{items[6]} ")
            # self.label_GPS_heading.setText(f"GPS航向：{items[12]}")
            self.label_IMU_state.setText(f"IMU状态：{items[15]}")
            items[16] = items[16].split('*')[0]
            self.label_heading_error.setText(f"航向角偏差：{items[16]}")
            self.label_Z_ang_vel.setText(f"z角速度：{DB.z_axis_pals:.4f}")
            self.label_left_right.setText(f"左：{DB.RT_left:.2f}右：{DB.RT_right:.2f}")

        except:
            print("imu数据有误")

    def speedinformation_show(self, speedinformation):
        try:
            items = speedinformation.split(',')
            DB.RT_left = int(items[1]) * np.pi * 0.18 * 3.6 / 60
            DB.RT_right = int(items[2]) * np.pi * 0.18 * 3.6 / 60
            self.label_left_right.setText(f"左：{DB.RT_left:.2f}右：{DB.RT_right:.2f}")
        except:
            print("速度数据有误")

#endregion



class Worker_CAN(QThread):
    message_received = pyqtSignal(can.Message)

    def __init__(self,bus):
        super().__init__()
        self.is_running = True  # 控制循环运行的标志位
        self.bus = bus

    def run(self):

        while self.is_running:
            recv_message = self.bus.recv(1)
            if recv_message != None:
                self.message_received.emit(recv_message)
            else :
                msg = can.Message(arbitration_id=0x11111, data=[0xff], is_extended_id=True)
                self.message_received.emit(msg)

    def stop(self):
        self.is_running = False  # 修改标志位以停止循环

class Worker_CarSpeed_Test(QThread):
    update_signal = pyqtSignal(int)

    def __init__(self,bus):
        super().__init__()
        self.is_running = True  # 控制循环运行的标志位
        self.speed_deviation = 0.005
        self.speed_step = 1
        self.bus = bus

    def run(self):

        while self.is_running:
            if DB.RT_speed > (DB.TargetCarSpeed + self.speed_deviation) :
                data = [0xF4, 0x04, 0x58, 0xEF, 0x00, 0x00, 0x00, 0x00]
                data[5] = int(self.speed_step) & 0xFF  
                checksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6]
                data[7] = (checksum + 0xE8) & 0xFF
                message = can.Message(arbitration_id=0x3E8, data=data, is_extended_id=False)
                self.bus.send(message)
            elif DB.RT_speed < (DB.TargetCarSpeed - self.speed_deviation) :
                data = [0xF4, 0x04, 0x58, 0xEF, 0xFF, 0x00, 0x00, 0x00]
                data[5] = (256 - self.speed_step) & 0xFF  
                checksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6]
                data[7] = (checksum + 0xE8) & 0xFF
                message = can.Message(arbitration_id=0x3E8, data=data, is_extended_id=False)
                self.bus.send(message)
            else:
                print("speed deviation has been reduced to zero.")

            time.sleep(0.1)  # 可以根据需要调整发送频率

    def stop(self):
        self.is_running = False  # 修改标志位以停止循环

class Worker_AutoDriving(QThread):
    update_signal = pyqtSignal(int)
    target_ind_signal = pyqtSignal(int, float)
    control_state_signal = pyqtSignal(float, int, float, float)
    Auto_unexpected_shutdown_signal = pyqtSignal()
    Auto_finished_signal = pyqtSignal()

    def __init__(self,bus):
        super().__init__()
        self.is_running = True  # 控制循环运行的标志位
        self.bus = bus

        # PI 控制器参数
        self.setpoint = 0.0          # 目标值        # 初始条件
        self.current_var = 0.0  # 实际值
        self.integral = 0.0          # 积分初值
        self.prev_time = 0.0
        self.prev_error = 0.0

    def run(self):

        DB.flag_Auto = True
        DB.header_flag = False

        bForward = True
        bForward_low = False
        bBack = False
        bBackStop = False
        bEndC = False

        FSend_flag = True
        FlowSend_flag = False
        BSend_flag = False
        BStopSend_flag = False
        EndSend_flag = False

        end_of_Stop = False

        cx_1, cy_1, cx_2, cy_2 = 0,0,0,0
        F_cx, F_cy, F_cyaw, F_ck, F_s = [], [], [], [], []
        B_cx, B_cy, B_cyaw, B_ck, B_s = [], [], [], [], []

        di, outputs = 0,0
        di_prev = 0.0

        distance = 0
        goal_flag = False
        end_flag = False

        DB.target_ind = 0
        DB.target_mind = 0

        lateral_deviation=0
        course_deviation=0

        state = Simu.State(x=DB.RT_x, y=DB.RT_y, yaw=Fun.remap_heading_angle(DB.RT_course), v=DB.RT_speed)
        e, e_th= 0.0, 0.0
        cyawIndices1 = DB.yawChangedIndices.copy()
        cyawIndices2 = DB.yawChangedIndices.copy()

        '''
        循环开始
        '''
        while self.is_running:

            DB.target_ind, DB.target_mind, _= Fun.calc_nearest_index(state, DB.cx, DB.cy, DB.cyaw)

            if len(cyawIndices1) == 0:
                x, y = DB.cx[-1], DB.cy[-1]
                distance = np.hypot(state.x - x, state.y - y)
                if not end_flag :
                    bForward = False
                    bEndC = True
                    end_flag = True
                    print("转弯点数列长度归零")
            else:
                x, y = DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]]
                distance = np.hypot(state.x - x, state.y - y)
            self.target_ind_signal.emit(DB.target_ind, distance)
            
            # 运行状态转换
            if bForward and (distance <= 0.5 or ((cyawIndices1[0] - 1 - DB.target_ind) < 3) if len(cyawIndices1) > 0 else False ):
                # 生成后向路径
                x_1, y_1 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 0, DB.Forward_limit_distance+1)
                x_2, y_2 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 0.2, DB.Forward_limit_distance)
                x_3, y_3 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 0.8, DB.Forward_limit_distance/2)
                x_4, y_4 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 1.5, DB.Forward_limit_distance/4)
                x_5, y_5 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 2.5, DB.Forward_limit_distance/8)
                x_6, y_6 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 5, 0)
                x_7, y_7 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                    DB.cyaw[cyawIndices1[0]], 6, 0)
                ax=[x_1, x_2, x_3, x_4, x_5, x_6,x_7]
                ay=[y_1, y_2, y_3, y_4, y_5, y_6,y_7]
                B_cx, B_cy, B_cyaw, B_ck, B_s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

                bForward = False
                bForward_low = True
                bBack = False
                bBackStop = False

            elif bForward_low and distance >= DB.Forward_limit_distance :
                bForward = False
                bForward_low = False
                bBack = True
                bBackStop = False
                    
            elif bBack :
                target_yaw = DB.cyaw[cyawIndices1[0]] + np.pi
                if target_yaw > np.pi:
                    target_yaw -= 2 * np.pi
                elif target_yaw < -np.pi:
                    target_yaw += 2 * np.pi
                yaw_error = target_yaw - Fun.remap_heading_angle(DB.RT_course)

                if (abs(yaw_error) <= DB.tolerance) and (distance >= DB.Back_limit_distance):
                    # 正常结束
                    bForward = False
                    bForward_low = False
                    bBack = False
                    bBackStop = True

                if distance >= DB.Back_limit_distance + 1:
                    # 极限结束
                    bForward = False
                    bForward_low = False
                    bBack = False
                    bBackStop = True

            elif bBackStop and end_of_Stop :
                if len(cyawIndices1) > 1:
                    cx_1, cy_1 = Fun.calculate_new_point(DB.cx[cyawIndices1[0]], DB.cy[cyawIndices1[0]], 
                                                            DB.cyaw[cyawIndices1[0]], 6, 0)
                    cx_2, cy_2 = Fun.calculate_new_point(DB.cx[cyawIndices1[1]], DB.cy[cyawIndices1[1]], 
                                                            DB.cyaw[cyawIndices1[1]], 0, 5)
                    ax = [cx_1, cx_2]
                    ay = [cy_1, cy_2]
                    F_cx, F_cy, F_cyaw, F_ck, F_s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
                else:
                    bEndC = True
                    bForward = False
                    bForward_low = False
                    bBack = False
                    bBackStop = False

                bForward = True
                bForward_low = False
                bBack = False
                bBackStop = False

                end_of_Stop = False

                cyawIndices1.pop(0)
            
            elif bEndC:
                if DB.end_cx_cy_flag:
                    distance = np.hypot(state.x - DB.end_cx[-1], state.y - DB.end_cy[-1])
                    if distance < 0.5:
                        goal_flag = True

                        # 控制结束，车辆停止
                        message = Fun.create_can_message_CarSpeed(0)    # 推杆回零，车速降低
                        self.bus.send(message)

                        di, outputs = 0, 500
                        message = Fun.create_can_2_message(di,int(outputs))
                        self.bus.send(message)
                else:
                    goal_flag = True

                    # 控制结束，车辆停止
                    message = Fun.create_can_message_CarSpeed(0)    # 推杆回零，车速降低
                    self.bus.send(message)

                    di, outputs = 0, 500
                    message = Fun.create_can_2_message(di,int(outputs))
                    self.bus.send(message)
                    

            # 状态更新
            state.x = DB.RT_x
            state.y = DB.RT_y
            state.yaw = Fun.remap_heading_angle(DB.RT_course)
            state.v = DB.RT_speed
            # state.v = DB.RT_left * 0.5 + DB.RT_right * 0.5

            if DB.flag_Auto :
                if bForward:
                    # 前进
                    if(FSend_flag):
                        message = Fun.create_can_message_CarSpeed(DB.forwardSpeed)
                        self.bus.send(message)

                        if DB.header_flag:
                            message = Fun.create_header_control_message(1)
                            self.bus.send(message)

                        FSend_flag = False
                        FlowSend_flag = True
                        BSend_flag = True
                        BStopSend_flag = True
                        EndSend_flag = True

                        if len(cyawIndices2) == len(DB.yawChangedIndices):
                            DB.start_cx_cy_flag = True
                            print('起始路径')  
                        else:
                            DB.start_cx_cy_flag = False
                            print('中间路径')

                        print('前进')

                    if (DB.RT_speed > 0.2 ):    
                    # if (True):
                        if DB.start_cx_cy_flag:
                            vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                        state, DB.start_cx, DB.start_cy, DB.start_cyaw, DB.start_ck, e, e_th)
                            # di, outputs = self.forward_control(vr,vl,e,e_th,1)
                            di, outputs = self.PI_control(vr, vl, 1)
                            # 左右转换时保证舵机先回零
                            if di * di_prev < 0 :
                                di, outputs = 0, 500
                            di_prev = di
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)
                        else:
                            vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                        state, F_cx, F_cy, F_cyaw, F_ck, e, e_th)
                            # di, outputs = self.forward_control(vr,vl,e,e_th,1)
                            di, outputs = self.PI_control(vr, vl, 1)
                            # 左右转换时保证舵机先回零
                            if di * di_prev < 0 :
                                di, outputs = 0, 500
                            di_prev = di
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)
                        lateral_deviation = e
                        course_deviation = e_th

                elif bForward_low :
                    if(FlowSend_flag):
                        message = Fun.create_can_message_CarSpeed(DB.forwardLowSpeed)
                        self.bus.send(message)

                        if DB.header_flag:
                            message = Fun.create_header_control_message(0)
                            self.bus.send(message)

                        FSend_flag = True
                        FlowSend_flag = False
                        BSend_flag = True
                        BStopSend_flag = True
                        EndSend_flag = True

                        print('低速')

                    if (DB.RT_speed > 0.1):
                        if DB.start_cx_cy_flag:
                            vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                        state, DB.start_cx, DB.start_cy, DB.start_cyaw, DB.start_ck, e, e_th)
                            # di, outputs = self.forward_control(vr,vl,e,e_th,1)
                            di, outputs = self.PI_control(vr, vl, 1)
                            # 左右转换时保证舵机先回零
                            if di * di_prev < 0 :
                                di, outputs = 0, 500
                            di_prev = di
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)
                        else:
                            vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                        state, F_cx, F_cy, F_cyaw, F_ck, e, e_th)
                            # di, outputs = self.forward_control(vr,vl,e,e_th,1)
                            di, outputs = self.PI_control(vr, vl, 1)
                            # 左右转换时保证舵机先回零
                            if di * di_prev < 0 :
                                di, outputs = 0, 500
                            di_prev = di
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)
                    lateral_deviation = np.hypot(state.x - DB.cx[cyawIndices2[0]], state.y - DB.cy[cyawIndices2[0]])
                    course_deviation = 0.01

                elif bBack:
                    if(BSend_flag):
                        message = Fun.create_can_message_CarSpeed(DB.reverseSpeed)
                        self.bus.send(message)

                        if DB.header_flag:
                            message = Fun.create_header_control_message(1)
                            self.bus.send(message)

                        FSend_flag = True
                        FlowSend_flag = True
                        BSend_flag = False
                        BStopSend_flag = True
                        EndSend_flag = True

                        print('后退')

                    if (DB.RT_speed > 0.1):
                        target_yaw = DB.cyaw[cyawIndices2[0]] + np.pi
                        if target_yaw > np.pi:
                            target_yaw -= 2 * np.pi
                        elif target_yaw < -np.pi:
                            target_yaw += 2 * np.pi
                        yaw_error = target_yaw - Fun.remap_heading_angle(DB.RT_course)

                        if (abs(yaw_error) <= DB.tolerance) and (distance >= DB.Back_limit_distance):
                            # 正常结束
                            di, outputs = 0, 500
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)

                        elif distance >= DB.Back_limit_distance + 1:
                            # 极限结束
                            di, outputs = 0, 500
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)

                        else:
                            vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                        state, B_cx, B_cy, B_cyaw, B_ck, e, e_th)
                            # di, outputs = self.forward_control(vr,vl,e,e_th,0)
                            di, outputs = self.PI_control(vr, vl, 0)
                            # 左右转换时保证舵机先回零
                            if di * di_prev < 0 :
                                di, outputs = 0, 500
                            di_prev = di
                            message = Fun.create_can_2_message(di, int(outputs))
                            self.bus.send(message)
                    lateral_deviation = np.hypot(state.x - DB.cx[cyawIndices2[0]], state.y - DB.cy[cyawIndices2[0]])
                    course_deviation = 0.01

                elif bBackStop:
                    if(BStopSend_flag):
                        message = Fun.create_can_message_CarSpeed(5)
                        self.bus.send(message)

                        if DB.header_flag:
                            message = Fun.create_header_control_message(1)
                            self.bus.send(message)

                            di, outputs = 0, 500
                            message = Fun.create_can_2_message(di,int(outputs))
                            self.bus.send(message)

                        FSend_flag = True
                        FlowSend_flag = True
                        BSend_flag = True
                        BStopSend_flag = False
                        EndSend_flag = True

                        start_time = None
                        end_of_Stop = False

                        print("停车")

                    lateral_deviation = np.hypot(state.x - DB.cx[cyawIndices2[0]], state.y - DB.cy[cyawIndices2[0]])
                    course_deviation = 0.01

                    # 不中断程序延时等待
                    if start_time is None:
                        start_time = datetime.now()

                    current_time = datetime.now()
                    elapsed_time = current_time - start_time

                    if elapsed_time >= timedelta(seconds=2):
                        start_time = None
                        end_of_Stop = True

                        cyawIndices2.pop(0)
                    

                elif bEndC:
                    if(EndSend_flag):
                        message = Fun.create_can_message_CarSpeed(DB.forwardSpeed)
                        self.bus.send(message)

                        if DB.header_flag:
                            message = Fun.create_header_control_message(1)
                            self.bus.send(message)

                        FSend_flag = False
                        FlowSend_flag = False
                        BSend_flag = False
                        BStopSend_flag = False
                        EndSend_flag = False

                        print('最终路径')

                    if (DB.RT_speed > 0.1):
                        vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                    state, DB.end_cx, DB.end_cy, DB.end_cyaw, DB.end_ck, e, e_th)
                        lateral_deviation = e
                        course_deviation = e_th
                        # di, outputs = self.forward_control(vr,vl,e,e_th,1)
                        di, outputs = self.PI_control(vr, vl, 1)
                        # 左右转换时保证舵机先回零
                        if di * di_prev < 0 :
                            di, outputs = 0, 500
                        di_prev = di
                        message = Fun.create_can_2_message(di,int(outputs))
                        self.bus.send(message)

            if goal_flag:
                print("goal")
                time.sleep(0.4)
                self.update_signal.emit(DB.data)  # 发送信号
                DB.data+=1
        
            self.control_state_signal.emit(di, int(outputs), lateral_deviation, course_deviation)
            time.sleep(0.1)

        '''
        循环结束
        '''        
        self.Auto_finished_signal.emit()

    def forward_control(self, vr, vl, e, e_th, direction):
        """
        前向控制函数，根据输入参数计算轮速差和输出值，区分车辆的前进或后退输出值。

        参数:
        vr (float): 右轮速度。
        vl (float): 左轮速度。
        e (float): 横向误差（偏差）。
        e_th (float): 航向误差（偏差角度）。
        direction (int): 行驶方向，1 表示前向，0 表示后向。

        返回:
        di (float): 计算得到的轮速差。
        outputs (float): 根据轮速差 di 和误差范围映射得到的输出控制值。
        """
        if direction == 1:
            di = vr - vl    # 前向
        else:
            di = vl - vr    # 后向
        di = 0 if abs(di) < DB.allowable_deviation else di
        di = np.clip(di, -2, 2)
        
        if 0.06 <= e <= 0.17 and abs(e_th) <= np.radians(5.7):
            outputs = Fun.map_value(abs(di), DB.input_range, DB.output_range, DB.min_output, DB.max_output)
        elif 0 <= e <= 0.19 and abs(e_th) <= np.radians(7):
            outputs = Fun.map_value(abs(di), DB.input_range_2, DB.output_range_2, DB.min_output_2, DB.max_output_2)
        elif -0.06 <= e <= 0.27:
            outputs = Fun.map_value(abs(di), DB.input_range_3, DB.output_range_3, DB.min_output_3, DB.max_output_3)
        else:
            outputs = Fun.map_value(abs(di), DB.input_range_4, DB.output_range_4, DB.min_output_4, DB.max_output_4)
        
        return di, outputs
    
    def PI_control(self, vr, vl, direction):
        """
        执行 PI 控制器计算并限制输出值的函数。

        参数:
        vr : float
            右侧轮子的目标速度（前进方向时的目标速度）。
        vl : float
            左侧轮子的目标速度（前进方向时的目标速度）。
        direction : int
            控制方向标志。1 表示前向控制，其他值表示后向控制。

        返回:
        di : float
            目标轮速差（偏差）。
        outputs : int
            限制在 500 到 700 之间的控制器输出值，且为整数。
        """
        if direction == 1:
            di = vr - vl    # 前向
            delta = DB.RT_right - DB.RT_left
        else:
            di = vl - vr    # 后向
            delta = DB.RT_left - DB.RT_right
        # 零区限制，消除上线微调时的左右剧烈摆动
        di = 0 if abs(di) < DB.allowable_deviation else di
        di = np.clip(di, -2, 2)
        # 差值为（目标值）为零时不控制
        if di == 0:
            di, outputs = 0, 500
            return di, outputs

        self.setpoint = di
        self.process_variable = delta
        
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt > 1:
            dt = 0.1
        error = self.setpoint - self.process_variable
        error = abs(error)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        control_output = DB.K_p * error + DB.K_i * self.integral + DB.K_d * derivative
        self.prev_time = current_time
        self.prev_error = error
        outputs = int(max(DB.u_min, min(control_output, DB.u_max)))

        if 0.03<di<0.3 and outputs<=510:
            outputs = Fun.map_value(abs(di), DB.input_range, DB.output_range, DB.min_output, DB.max_output)
        
        return di, outputs


    def stop(self):
        self.is_running = False  # 修改标志位以停止循环
        
class Worker_gnss(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)
    ggameg_signal = pyqtSignal(str)
    imumeg_signal = pyqtSignal(str)
    message_generated = threading.Event()

    def __init__(self):
        super(Worker_gnss, self).__init__()
        self.should_run = True
        # 初始化队列存储角度值和时间戳
        self.angles = deque(maxlen=10)  # 假设存储最近100个角度值
        self.timestamps = deque(maxlen=10)

        # 创建坐标转换器
        crs_wgs = CRS("EPSG:4326")
        crs_gauss_kruger = CRS("EPSG:4547")
        self.transformer = Transformer.from_crs(crs_wgs, crs_gauss_kruger)

        self.baudrate = 115200
        if platform.system() == 'Linux':
            self.port = "/dev/ttyUSB0"
        elif platform.system() == 'Windows':
            self.port = "com17"
            # self.port = "com20"

    def run(self):
        try:
            if self.baudrate is not None and self.port is not None:
                ser = serial.Serial(self.port, self.baudrate, timeout=1)
                # ser.write("$MOFF\r\n".encode()) 
                # time.sleep(0.1)
                # ser.write("$MASC,MPOS,10\r\n".encode())
                # ser.write("$MASC,MIMU,10\r\n".encode())

            while self.should_run:
                raw_message = ser.readline().decode('utf-8').rstrip()
                if raw_message.startswith("$MPOS"):
                    mpos_data = raw_message
                    self.ggameg_signal.emit(mpos_data)
                    try:
                        items = mpos_data.split(',')
                        DB.RT_longitude = float(items[4])
                        DB.RT_latitude = float(items[3])
                        DB.RT_altitude = float(items[5])
                        DB.RT_speed = float(items[6])
                        DB.RT_course = float(items[7])
                        DB.RT_x, DB.RT_y = self.transformer.transform(DB.RT_latitude, DB.RT_longitude)
                        DB.rlongitude = float(items[13])
                        DB.rlatitude = float(items[12])
                    except:
                        print("MPOS转换出错！")

                if raw_message.startswith("$MIMU"):
                    mimu_data = raw_message
                    self.imumeg_signal.emit(mimu_data)
                    try:
                        items = mimu_data.split(',')
                        items[16] = items[16].split('*')[0]
                        DB.x_axis_acc = float(items[1])
                        DB.y_axis_acc = float(items[2])
                        DB.z_axis_acc = float(items[3])
                        DB.x_axis_angle = float(items[4])
                        DB.y_axis_angle = float(items[5])
                        DB.z_axis_angle = float(items[6])
                        DB.x_axis_euler = float(items[7])
                        DB.x_axis_euler = float(items[8])
                        DB.inter_tempe = float(items[9])
                        DB.install_location = float(items[10])
                        DB.route_direction = float(items[11])
                        DB.GPS_heading = float(items[12])
                        DB.heading_offset = float(items[13])
                        DB.stop_heading_offset = float(items[14])
                        DB.IMU_ini_state = float(items[15])
                        DB.fusion_heading = float(items[16])

                        DB.z_axis_pals = self.Angular_Velocity_Calculation(DB.z_axis_angle)
                        DB.RT_left, DB.RT_right = self.calculate_track_speeds(DB.RT_speed,DB.z_axis_pals,2)
                    except:
                        print("MIMU转换出错！")
                    if DB.flag_TrackRecord :
                        DB.record_longitude_list.append(DB.RT_longitude)
                        DB.record_latitude_list.append(DB.RT_latitude)
                        if (DB.flag_left == False and DB.flag_right == False ):
                            DB.record_left_right.append(0)
                        elif (DB.flag_left == False and DB.flag_right == True ):
                            DB.record_left_right.append(1)
                        elif (DB.flag_left == True and DB.flag_right == False ):
                            DB.record_left_right.append(2)
                        DB.record_speed_list.append(DB.RT_speed)
                        DB.record_course_list.append(DB.RT_course)
                        if len(DB.cx) != 0:
                            e, e_th = Fun.calculate_distance_direction_yaw_error(DB.RT_x, DB.RT_y, Fun.remap_heading_angle(DB.RT_course), DB.cx, DB.cy, DB.cyaw)
                        DB.record_deviation_list.append(e)
                        DB.record_eth_deviation_list.append(e_th)
                        DB.record_reallongitude_list.append(DB.rlongitude)
                        DB.record_reallatitude_list.append(DB.rlatitude)
                        
                        DB.record_z_axis_angle_list.append(DB.z_axis_angle)
                        DB.record_z_axis_pals_list.append(DB.z_axis_pals)
                        DB.record_left_speed_list.append(DB.RT_left)
                        DB.record_right_speed_list.append(DB.RT_right)

                        DB.record_time1_list.append(datetime.now())

        except Exception as err_1:
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            self.finished_signal.emit()
            ser.close()

    def Angular_Velocity_Calculation(self, current_angle):
        # 角度归一化函数
        def normalize_angle(angle, prev_angle):
            delta = angle - prev_angle
            if delta > 180:
                angle -= 360
            elif delta < -180:
                angle += 360
            return angle
        # 计算中心差分
        def central_difference(values, timestamps):
            # delta_t = np.diff(timestamps)
            central_diff = (values[2:] - values[:-2]) / (timestamps[2:] - timestamps[:-2])
            return central_diff
        # current_angle = DB.z_axis_angle
        current_time = time.time()
        # 归一化角度值
        if len(self.angles) > 0:
            current_angle = normalize_angle(current_angle, self.angles[-1])
        # 将当前角度值和时间戳加入队列
        self.angles.append(np.radians(current_angle))  # 转换为弧度
        self.timestamps.append(current_time)
        # 如果队列中有足够的数据点，计算角速度和角加速度
        if len(self.angles) > 2:
            # 计算角速度（使用中心差分）
            angular_velocity = central_difference(np.array(self.angles), np.array(self.timestamps))
            current_angular_velocity = angular_velocity[-1]  # 获取最新的角速度
            # print(f"当前角速度 (rad/s): {current_angular_velocity:.4f}")
        return current_angular_velocity
    
    def calculate_track_speeds(self, v_c, omega_z, W):
        '''
        参数：
        v_c: 质心速度，单位：m/s
        omega_z: 航向角速度，单位：rad/s
        W: 车辆宽度，单位：m
        '''
        v_L = v_c - (W * omega_z) / 1.2
        v_R = v_c + (W * omega_z) / 1.2
        return v_L, v_R
        
class Worker_serial(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)
    speed_signal = pyqtSignal(str)

    def __init__(self):
        super(Worker_serial, self).__init__()
        self.should_run = True
        self.baudrate = 115200
        if platform.system() == 'Linux':
            self.port = "/dev/ttyACM0"
        elif platform.system() == 'Windows':
            self.port = "com22"

    def run(self):
        try:
            if self.baudrate is not None and self.port is not None:
                ser = serial.Serial(self.port, self.baudrate, timeout=1)
                time.sleep(0.5)
            while self.should_run:
                raw_message_speed = ser.readline().decode('utf-8').rstrip()
                if raw_message_speed.startswith("$speed"):
                    self.speed_signal.emit(raw_message_speed)
                try:
                    items = raw_message_speed.split(',')
                    DB.RT_left = float(items[1]) * np.pi * 0.18 * 3.6 / 60
                    DB.RT_right = float(items[2]) * np.pi * 0.18 * 3.6 / 60

                    if DB.flag_TrackRecord :
                        DB.record_left_speed_list.append(DB.RT_left)
                        DB.record_right_speed_list.append(DB.RT_right)
                        DB.record_time2_list.append(datetime.now())
                except:
                    print("数据转换出错！")
        except Exception as err_1:
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            self.finished_signal.emit()
            ser.close()

def main():
    if platform.system() == 'Linux':
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)   # 启用高DPI（每英寸点数）缩放
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)      # 启用高分辨率图标
    app = QApplication(sys.argv)

    # 启动CAN
    try:
        if platform.system() == 'Linux':
            os.system('clear')
            print("Linux 系统\n尝试初始化 SocketCAN...")
            bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)
            print("SocketCAN 已成功初始化")

        elif platform.system() == 'Windows':
            # os.system('cls')
            print("Windows 系统\n尝试初始化 PCAN 设备...")
            bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
            print("PCAN 设备已成功初始化")

            # print("Windows 系统，尝试初始化 SLCAN 设备...")
            # bus = can.interface.Bus(bustype='slcan', channel='COM12', bitrate=250000)
            # print("SLCAN 设备已成功初始化")
        else:
            raise EnvironmentError("不支持的操作系统平台")
    except Exception as e:
        print(f"错误: {e}")
        print("当前系统下不存在 CAN 设备")
        bus = can.interface.Bus(interface='virtual', channel='test')
        print("已初始化虚拟 CAN 设备进行测试")

    window = MainWindow(bus)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

