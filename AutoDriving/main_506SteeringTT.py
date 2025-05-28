import os
import sys
import platform
import serial
# import datetime
import socket
import base64
import serial.tools.list_ports
import time
import pynmea2
import can
import threading

import numpy as np
import cubic_spline_planner
import simulation_pyCAN as Sim
import User_Function as Fun
from datetime import datetime,timedelta,timezone
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from UI_Steering import Ui_MainWindow  # 导入ui.py 中的 Ui_MaiuinWindow 界面类
from pyproj import CRS, Transformer
from collections import deque
import pandas as pd
import DataBase as DB


class MainWindow(QMainWindow, Ui_MainWindow):  # 继承 QMainWindow 类和 Ui_MainWindow 界面类
    def __init__(self,bus):
        super().__init__()
        self.setFocusPolicy(Qt.StrongFocus)  # 允许通过键盘和鼠标点击获取焦点
        self.setupUi(self)  # 继承 Ui_MainWindow 界面类
        if platform.system() == 'Linux':
            self.showMaximized()
        
        self.bus = bus

        self.pushButton_serial_GNSS.clicked.connect(self.click_pushButton_GNSS)
        self.pushButton_TrackRecord.clicked.connect(self.click_pushButton_TrackRecord)

        self.pushButton_CAN.clicked.connect(self.click_pushButton_CAN)
        self.pushButton_Steer_left.pressed.connect(self.pressed_pushButton_Steer_left)
        self.pushButton_Steer_left.released.connect(self.released_pushButton_Steer_left)
        self.pushButton_Steer_right.pressed.connect(self.pressed_pushButton_Steer_right)
        self.pushButton_Steer_right.released.connect(self.released_pushButton_Steer_right)

        # 创建坐标转换器
        crs_wgs = CRS("EPSG:4326")
        crs_gauss_kruger = CRS("EPSG:4547")
        # EPSG:4547，中国大陆2000国家大地坐标系 / 3度带投影 / 第19带
        # 基于CGCS2000（中国大陆2000国家大地坐标系）的高斯-克吕格（Gauss-Kruger）投影系统，用于第19带，每带覆盖3度经度。 
        # 数据和地图通常会被投影到以东经108度为中心的区域（即第19带），采用CGCS2000作为其地理坐标系统的基准。
        self.transformer = Transformer.from_crs(crs_wgs, crs_gauss_kruger)

        # 创建工作线程
        self.workerautodriving = Worker_AutoDriving()  # 创建工作线程
        # self.positionRecord = Worker_Position_Record()
        self.can_thread = Worker_CAN(self.bus)

        


#region pushButton CAN operation

    def pressed_pushButton_Steer_left(self):
        # 左转按钮按下时执行的代码
        try:
            speed_data = self.lineEdit_SteerTest.text()
            if 500 <= int(speed_data) <= 1000: 
                self.label_SteerTest.setStyleSheet("background-color: green") # 指示区分
                data = [0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00]
                data[0] = (int(speed_data) >> 8) & 0xFF
                data[1] = int(speed_data) & 0xFF
                message = can.Message(arbitration_id=0x9F351F2, data=data, is_extended_id=True)
                try:
                    self.bus.send(message)
                except:
                    print("当前系统下不存在CAN设备")
                DB.flag_left = True
            else:
                print("数字不对")
        except ValueError:  # 如果转换失败（即文本不是一个整数）
            print("数字转换出错")
        
    def released_pushButton_Steer_left(self):
        # 左转按钮释放时执行的代码
        self.label_SteerTest.setStyleSheet("background-color: gray")
        data = [0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F351F2, data=data, is_extended_id=True)
        try:
            self.bus.send(message)
        except:
            print("当前系统下不存在CAN设备")
        DB.flag_left = False

    def pressed_pushButton_Steer_right(self):
        # 右转按钮按下时执行的代码
        try:
            speed_data = self.lineEdit_SteerTest.text()
            if 500 <= int(speed_data) <= 1000: 
                self.label_SteerTest.setStyleSheet("background-color: red")
                data = [0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00]
                data[0] = (int(speed_data) >> 8) & 0xFF
                data[1] = int(speed_data) & 0xFF
                message = can.Message(arbitration_id=0x9F351F2, data=data, is_extended_id=True)
                try:
                    self.bus.send(message)
                except:
                    print("当前系统下不存在CAN设备")
                DB.flag_right = True
            else:
                print("数字不对")
        except ValueError:  # 如果转换失败（即文本不是一个整数）
            print("数字转换出错")

    def released_pushButton_Steer_right(self):
        # 右转按钮释放时执行的代码
        self.label_SteerTest.setStyleSheet("background-color: gray")
        data = [0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F351F2, data=data, is_extended_id=True)
        try:
            self.bus.send(message)
        except:
            print("当前系统下不存在CAN设备")
        DB.flag_right = False

    def click_pushButton_CAN(self):
        if self.can_thread.isRunning():
            self.can_thread.stop()
            self.can_thread.wait()
            self.pushButton_CAN.setText('CAN测试关')
        else:
            self.can_thread = Worker_CAN(self.bus)
            self.can_thread.message_received.connect(self.handle_message)
            self.can_thread.start()
            self.pushButton_CAN.setText('CAN测试开')

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
        try:
            self.bus.shutdown()
        except:
            print("当前系统下不存在CAN设备")
            
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
    
    def click_pushButton_Ntrip(self):
        try:
            if self.pushButton_serial_Ntrip.text() == "差分关":
                self.label_serial_state.setText("串口状态:3")
                self.pushButton_serial_Ntrip.setText('差分开')
                self.ntrip_start()
            else:
                self.label_serial_state.setText("串口状态:2")
                self.pushButton_serial_Ntrip.setText('差分关')
                self.ntrip_stop()
        except KeyboardInterrupt:
            print("程序已停止")

    def click_pushButton_pointA(self):
        self.lineEdit_pointA_lon.setText(f"{DB.RT_longitude}")
        self.lineEdit_pointA_lat.setText(f"{DB.RT_latitude}")
        DB.pointAx, DB.pointAy = self.transformer.transform(DB.RT_latitude, DB.RT_longitude)

    def click_pushButton_pointB(self):
        self.lineEdit_pointB_lon.setText(f"{DB.RT_longitude}")
        self.lineEdit_pointB_lat.setText(f"{DB.RT_latitude}")
        DB.pointBx, DB.pointBy = self.transformer.transform(DB.RT_latitude, DB.RT_longitude)

    def click_pushButton_Fit_navigation_line(self):
        DB.pointx.append(DB.pointAx)
        DB.pointx.append(DB.pointBx)
        DB.pointy.append(DB.pointAy)
        DB.pointy.append(DB.pointBy)
        DB.cx, DB.cy, DB.cyaw, DB.ck, DB.s = cubic_spline_planner.calc_spline_course(DB.pointx, DB.pointy, ds=1)
        self.label_Fit_flag.setText("已拟合")


#endregion

#region pushButton AutoDriving

    def click_pushButton_AutoDriving(self):

        if self.workerautodriving.isRunning():
            self.workerautodriving.stop()  # 停止线程
            self.workerautodriving.wait()  # 等待线程真正结束
            self.pushButton_AutoDriving.setText("自动驾驶关")
        else:
            self.workerautodriving = Worker_AutoDriving()  # 重新创建线程对象
            self.workerautodriving.update_signal.connect(self.endPrompt_Display)
            self.workerautodriving.start()  # 启动线程
            self.pushButton_AutoDriving.setText("自动驾驶开")
            self.label_AutoPrompt.setText("驾驶中")

    def endPrompt_Display(self, value):     # 目前测试用
        self.label_AutoPrompt.setText(f"驾驶结束{value}")
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
                            'right speed':DB.record_right_speed_list, 'setpoint':DB.record_setspont_list})
            current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
            parameter = self.lineEdit_SteerTest.text()
            file1_name = f"Steering_{current_time}_{parameter}.xlsx"

            current_dir = os.path.dirname(os.path.abspath(__file__))  
            out_dir = os.path.join(current_dir, "out")  
            if not os.path.exists(out_dir):  
                os.makedirs(out_dir)  
            full_file1_path = os.path.join(out_dir, file1_name) 
            df1.to_excel(full_file1_path, index=False, engine='openpyxl')

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
            DB.record_setspont_list = []



#endregion

#region GNSS operation

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

    def gnss_worker_finished(self):
        self.gnss_work = None

    def ntrip_start(self):
        try:
            self.ntrip_work = Worker_Ntrip(self.gnss_work)
            self.ntrip_work.finished_signal.connect(self.ntrip_worker_finished)
            self.ntrip_work.mistake_message_transmit.connect(self.mistake_message_show)
            self.ntrip_work.start()
        except serial.serialutil.SerialException as error:
            self.mistake_message_show(error)

    def ntrip_stop(self):
        if hasattr(self, 'ntrip_work') and self.gnss_work is not None:
            self.ntrip_work.should_run = False
            self.ntrip_work.quit()
            self.ntrip_work.wait()

    def ntrip_worker_finished(self):
        self.ntrip_work = None

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
            #  self.label_gps_qual.setText(f"定位质量：{ggameg.gps_qual}")
            # self.label_num_sats.setText(f"卫星数量：{ggameg.num_sats}")
            # self.label_HDOP.setText(f"HDOP：{ggameg.horizontal_dil}")
            self.label_speed.setText(f"速度：{items[6]}km/h")
            self.label_cource.setText(f"航向：{items[7]}°")

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

#endregion

#region Key
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return  # Ignore auto-repeated key press events
        if event.key() == Qt.Key_Left:
            self.pushButton_Steer_left.pressed.emit()

        elif event.key() == Qt.Key_Right:
            self.pushButton_Steer_right.pressed.emit()


    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return  # Ignore auto-repeated key release events
        if event.key() == Qt.Key_Left:
            self.pushButton_Steer_left.released.emit()

        elif event.key() == Qt.Key_Right:
            self.pushButton_Steer_right.released.emit()
#endregion

class Worker_CAN(QThread):
    message_received = pyqtSignal(can.Message)

    def __init__(self,bus):
        super().__init__()
        self.is_running = True  # 控制循环运行的标志位
        self.bus = bus

    def run(self):

        while self.is_running:
            message = self.bus.recv(1)
            if message != None:
                self.message_received.emit(message)
            else :
                msg = can.Message(arbitration_id=0x11111, data=[0xff], is_extended_id=True)
                self.message_received.emit(msg)

    def stop(self):
        self.is_running = False  # 修改标志位以停止循环

class Worker_Position_Record(QThread):
    update_signal = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.is_running = True  # 控制循环运行的标志位

    def run(self):

        while self.is_running:
            print(f"vr_rpm:,vl_rpm:")

    def stop(self):
        self.is_running = False  # 修改标志位以停止循环

class Worker_AutoDriving(QThread):
    update_signal = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.is_running = True  # 控制循环运行的标志位

    def run(self):

        try:
            if platform.system() == 'Linux':
                bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate = 250000)
            if platform.system() == 'Windows':
                bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
        except:
            print("当前系统下不存在CAN设备")

        sim_flag = True

        DB.pointx = [0.0, 20.0]
        DB.pointy = [0.0, 0.0]

        DB.cx, DB.cy, DB.cyaw, DB.ck, DB.s = cubic_spline_planner.calc_spline_course(DB.pointx, DB.pointy, ds=0.1)
        goal = [DB.cx[-1], DB.cy[-1]]
        e, e_th = 0.0, 0.0

        state = Sim.State(x=DB.RT_x, y=DB.RT_y, yaw=DB.RT_course, v=DB.RT_speed)
        state = Sim.State(x=-1, y=-2, yaw=0.0, v=0)

        while self.is_running:
            if sim_flag :
                vr, vl, target_ind, e, e_th = Sim.lqr_steering_control(
                    state, DB.cx, DB.cy, DB.cyaw, DB.ck, e, e_th)
                vr_r, vl_r = Sim.speed_limit(vr, vl)
                # print(f"vr_rpm:{vr},vl_rpm:{vl}")
                print(f"vr_rpm:{vr},vl_rpm:{vl}")
                DB.data+=1
                # 控制输出
                # message = Sim.create_can_2_message(vr_rpm, vl_rpm)
                # bus.send(message)
                # 状态更新
                # state.x=DB.RT_x
                # state.y=DB.RT_y
                # state.yaw=Sim.remap_heading_angle(DB.RT_course)
                # state.v=DB.RT_speed

                # 模拟
                state = Sim.update(state, vr, vl, Sim.dt)

                time.sleep(0.01)
            
            if Sim.check_goal(state, goal, target_ind, len(DB.cx)):
                if sim_flag:
                    print("goal")
                sim_flag = False

                time.sleep(0.5)
                self.update_signal.emit(DB.data)  # 发送信号
                DB.data+=1
        try:
            bus.shutdown()
        except:
            print("当前系统下不存在CAN设备")

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
                        else:
                            e,e_th = 0, 0
                        DB.record_deviation_list.append(e)
                        DB.record_eth_deviation_list.append(e_th)
                        DB.record_reallongitude_list.append(DB.rlongitude)
                        DB.record_reallatitude_list.append(DB.rlatitude)
                        
                        DB.record_z_axis_angle_list.append(DB.z_axis_angle)
                        DB.record_z_axis_pals_list.append(DB.z_axis_pals)
                        DB.record_left_speed_list.append(DB.RT_left)
                        DB.record_right_speed_list.append(DB.RT_right)
                        DB.record_setspont_list.append(DB.setpoint)

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

class Worker_Ntrip(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)

    def __init__(self, gnss_worker):
        super(Worker_Ntrip, self).__init__()
        self.port = "/dev/ttyUSB0"
        self.baudrate = 115200
        self.should_run = True
        self.gnss_worker = gnss_worker

        # self.Ntrip_host = "120.253.239.161"
        # self.Ntrip_port = 8002
        # self.Ntrip_mountpoint = "RTCM33_GRCEJ"
        # self.Ntrip_user = "cavm174"
        # self.Ntrip_password = "8uh087n8"

        self.Ntrip_host = "rtk.ntrip.qxwz.com"
        self.Ntrip_port = 8002
        self.Ntrip_mountpoint = "AUTO"
        self.Ntrip_user = "qxyifw001"
        self.Ntrip_password = "577da0f"

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.reconnect()
            while self.should_run:
                gnss_messages_send = self.gnss_worker.gnss_messages_send
                print(gnss_messages_send)
                if self.RTK_singal_OK == 1:
                    self.s.send(gnss_messages_send.encode())
                    rtcm_data = self.s.recv(102400)
                    if rtcm_data is not None:
                        ser.write(rtcm_data)
                        ser.flush()
        except Exception as err_1:
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            self.finished_signal.emit()
            ser.close()

    def reconnect(self):
        while True:
            self.s = socket.socket()
            self.s.settimeout(10)
            self.s.connect((self.Ntrip_host, self.Ntrip_port))
            self.ntrip_request = f"GET /{self.Ntrip_mountpoint} HTTP/1.0\r\n"
            self.ntrip_request += "User-Agent: NTRIP PythonClient/1.0\r\n"
            self.ntrip_request += f"Authorization: Basic {base64.b64encode((self.Ntrip_user + ':' + self.Ntrip_password).encode()).decode()}\r\n"
            self.ntrip_request += "\r\n"

            self.s.send(self.ntrip_request.encode())
            response = self.s.recv(1024)

            if b"ICY 200 OK" in response:
                self.RTK_singal_OK = 1
                break


def main():
    if platform.system() == 'Linux':
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)   # 启用高DPI（每英寸点数）缩放
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)      # 启用高分辨率图标
    app = QApplication(sys.argv)

    # 启动CAN
    try:
        if platform.system() == 'Linux':
            bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate = 250000)
        if platform.system() == 'Windows':
            bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=250000)
    except:
        print("当前系统下不存在CAN设备")
        bus = can.interface.Bus(interface='virtual',channel='test')

    window = MainWindow(bus)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
