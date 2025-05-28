"""
604
增加Windows平台识别。
Win系统下需插入PCAN设备(USBtoCAN)。
程序统一一个CAN总线bus，不必每个线程创建一次。

从偏差小于阈值修改为输出小于阈值为零。
修改最小输出起点变更，为550。
增加输出值和上一次输出值异号时，强制输出为零。
增加轮速检测和记录。
614
套行路径
617
直线测试，曲线测试
两点直线，三点曲线
增加PID控制器

"""

import os
import sys
import platform
import serial
import serial.tools.list_ports
import time
import can
import threading
import queue

import numpy as np
import cubic_spline_planner
import User_Function as Fun
# import stanley_controller as Stan
import simulation_pyCAN as Simu
from datetime import datetime,timedelta,timezone
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt, QFile, QTextStream
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from UI_617 import Ui_MainWindow  # 导入ui.py 中的 Ui_MaiuinWindow 界面类
from pyproj import CRS, Transformer
from collections import deque
import pandas as pd
import DataBase as DB


class MainWindow(QMainWindow, Ui_MainWindow):  # 继承 QMainWindow 类和 Ui_MainWindow 界面类
    def __init__(self,bus):
        super().__init__()
        self.setupUi(self)  # 继承 Ui_MainWindow 界面类
        # self.apply_stylesheet()
        if platform.system() == 'Linux':
            self.showMaximized()
        self.bus = bus

        self.pushButton_serial_GNSS.clicked.connect(self.click_pushButton_GNSS)
        self.pushButton_AutoDriving.clicked.connect(self.click_pushButton_AutoDriving)
        self.pushButton_AutoSuspend.clicked.connect(self.click_pushButton_AutoSuspend)
        self.pushButton_fit.clicked.connect(self.click_pushButton_Fit_navigation_line)
        self.pushButton_TrackRecord.clicked.connect(self.click_pushButton_TrackRecord)
        self.pushButton_valve_Confirm.clicked.connect(self.click_pushButton_valve_Confirm)
        self.comboBox_ForB.currentIndexChanged.connect(self.on_combobox_changed)

        self.pushButton_CAN.clicked.connect(self.click_pushButton_CAN)
        self.pushButton_header_up.clicked.connect(self.click_pushButton_header_up)
        self.pushButton_header_down.clicked.connect(self.click_pushButton_header_down)
        
        self.pushButton_Reel_UP.pressed.connect(self.pressed_pushButton_Reel_UP)
        self.pushButton_Reel_UP.released.connect(self.released_pushButton_Reel_UP)
        self.pushButton_Reel_Down.pressed.connect(self.pressed_pushButton_Reel_Down)
        self.pushButton_Reel_Down.released.connect(self.released_pushButton_Reel_Down)

        self.pushButton_setPoint.clicked.connect(self.click_pushButton_setPoint)


        # 创建转换器
        crs_wgs = CRS("EPSG:4326")
        crs_gauss_kruger = CRS("EPSG:4547")
        self.transformer = Transformer.from_crs(crs_wgs, crs_gauss_kruger)

        # 创建工作线程
        self.workerautodriving = Worker_AutoDriving(self.bus)
        self.can_thread = Worker_CAN(self.bus)
        
    def apply_stylesheet(self):
        # 加载 QSS 样式表
        file = QFile("style.qss")
        if file.open(QFile.ReadOnly | QFile.Text):
            stream = QTextStream(file)
            self.setStyleSheet(stream.readAll())
            file.close()


#region pushButton CAN operation

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
            value = 1100
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

                ax = [x1, x2, x3]
                ay = [y1, y2, y3]
                DB.cx,DB.cy,DB.cyaw,DB.ck,DB.s =cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
                # print(DB.cx,DB.cy,DB.cyaw)

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

#endregion

#region pushButton AutoDriving

    def on_combobox_changed(self, index):
        if index == 0:
            DB.forward_back_flag = True

            latitude1, longitude1 = map(float, self.lineEdit_pointA.text().split(','))
            latitude2, longitude2 = map(float, self.lineEdit_pointB.text().split(','))
            x1, y1 = self.transformer.transform(latitude1, longitude1)
            x2, y2 = self.transformer.transform(latitude2, longitude2)
            ax = [x1, x2]
            ay = [y1, y2]
            DB.cx,DB.cy,DB.cyaw,DB.ck,DB.s =cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
            print("前进直线拟合")
            
        elif index == 1:
            DB.forward_back_flag = False

            latitude1, longitude1 = map(float, self.lineEdit_pointA.text().split(','))
            latitude2, longitude2 = map(float, self.lineEdit_pointB.text().split(','))
            x1, y1 = self.transformer.transform(latitude1, longitude1)
            x2, y2 = self.transformer.transform(latitude2, longitude2)
            ax = [x2, x1]
            ay = [y2, y1]
            DB.cx,DB.cy,DB.cyaw,DB.ck,DB.s =cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
            print("后退直线拟合")
            

    def click_pushButton_valve_Confirm(self):
        # 参数配置
        try:
            text = self.lineEdit_valve_parameter1.text()
            DB.K_p = float(text)

            text = self.lineEdit_valve_parameter2.text()
            DB.K_i = (float(text))

            text = self.lineEdit_valve_parameter3.text()
            DB.K_d = (float(text))

            text = self.lineEdit_valve_parameter4.text()
            var1, var2 = text.split(',')
            DB.u_min = (int(var1))
            DB.u_max = (int(var2))

        except:
            print('转换出错')

    def click_pushButton_setPoint(self):
        parameter1 = self.lineEdit_setPoint.text()
        if DB.setpoint == 0:
            DB.setpoint = float(parameter1)
        else:
            DB.setpoint = 0
        self.update_setpoint()

    def update_setpoint(self):
        self.label_setpoint_current.setText(f"setpoint = {DB.setpoint}")

    def click_pushButton_AutoDriving(self):

        if self.workerautodriving.isRunning() :
            self.workerautodriving.stop()  # 停止线程
            self.workerautodriving.wait()  # 等待线程真正结束
            self.pushButton_AutoDriving.setText("自动驾驶关")
            self.label_AutoPrompt.setText("驾驶停止")
            self.pushButton_AutoSuspend.setText("自动驾驶暂停")
            message = Fun.create_can_2_message(0,500)
            self.bus.send(message)

            DB.flag_TrackRecord = False
            self.pushButton_TrackRecord.setText('轨迹记录关')
            # self.trackRecord()
        else:
            self.workerautodriving = Worker_AutoDriving(self.bus)  # 重新创建线程对象
            self.workerautodriving.update_signal.connect(self.endPrompt_Display)
            self.workerautodriving.target_ind_signal.connect(self.Prompt_Display)
            self.workerautodriving.control_state_signal.connect(self.ConState_Display)
            self.workerautodriving.Auto_finished_signal.connect(self.Autodriving_finished)
            self.workerautodriving.start()  # 启动线程
            self.pushButton_AutoDriving.setText("自动驾驶开")
            self.label_AutoPrompt.setText("自动驾驶中")
            self.pushButton_AutoSuspend.setText("自动驾驶中")

            DB.flag_TrackRecord = True
            self.pushButton_TrackRecord.setText('轨迹记录开')

    def click_pushButton_AutoSuspend(self):
        if self.workerautodriving.isRunning() and DB.flag_Auto == True:
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
    
    def ConState_Display(self, di, output):
        self.label_AutoState.setText(f"控制状态：{(di):.3f},{output}")
        DB.control_state_time_list.append(datetime.now())
        DB.control_state_di_list.append(di)
        DB.control_state_output_list.append(output)

    def endPrompt_Display(self, value):     
        self.label_AutoPrompt.setText(f"驾驶结束{value}")

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
                            'right speed':DB.record_right_speed_list, 'setpoint':DB.record_setspont_list})
            current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
            parameter1 = self.lineEdit_valve_parameter1.text()
            parameter2 = self.lineEdit_valve_parameter2.text()
            parameter3 = self.lineEdit_valve_parameter3.text()
            file1_name = f"617G_{current_time}_para{parameter1}_{parameter2}_{parameter3}.xlsx"

            current_dir = os.path.dirname(os.path.abspath(__file__))  
            out_dir = os.path.join(current_dir, "out")  
            if not os.path.exists(out_dir):  
                os.makedirs(out_dir)  
            full_file1_path = os.path.join(out_dir, file1_name) 
            df1.to_excel(full_file1_path, index=False, engine='openpyxl')

            df2 = pd.DataFrame({'Time': DB.control_state_time_list, 'control-state': DB.control_state_di_list, 
                                    'output':DB.control_state_output_list})
            file2_name = f"617G_{current_time}_control.xlsx"
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
            DB.record_setspont_list = []

    def trackRecord(self):
        df1 = pd.DataFrame({'Time': DB.record_time1_list, 'longitude': DB.record_longitude_list, 'latitude':DB.record_latitude_list,
                            'left-right':DB.record_left_right,'speed':DB.record_speed_list,
                            'course':DB.record_course_list,'deviation':DB.record_deviation_list,
                            'ethdeviation':DB.record_eth_deviation_list, 'rlongitude': DB.record_reallongitude_list, 
                            'rlatitude':DB.record_reallatitude_list, 'z axis angle':DB.record_z_axis_angle_list, 
                            'z axis pals':DB.record_z_axis_pals_list, 'left speed':DB.record_left_speed_list,
                            'right speed':DB.record_right_speed_list, 'setpoint':DB.record_setspont_list})
        current_time = datetime.now().strftime("%Y%m%d-%H%M%S")
        parameter1 = self.lineEdit_valve_parameter1.text()
        parameter2 = self.lineEdit_valve_parameter2.text()
        parameter3 = self.lineEdit_valve_parameter3.text()
        file1_name = f"617G_{current_time}_P{parameter1}_I{parameter2}_D{parameter3}.xlsx"

        current_dir = os.path.dirname(os.path.abspath(__file__))  
        out_dir = os.path.join(current_dir, "out")  
        if not os.path.exists(out_dir):  
            os.makedirs(out_dir)  
        full_file1_path = os.path.join(out_dir, file1_name) 
        df1.to_excel(full_file1_path, index=False, engine='openpyxl')

        df2 = pd.DataFrame({'Time': DB.control_state_time_list, 'control-state': DB.control_state_di_list, 
                                'output':DB.control_state_output_list})
        file2_name = f"617G_{current_time}_control.xlsx"
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
        DB.record_setspont_list = []
        


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
            # self.serial_work = Worker_serial()
            # self.serial_work.finished_signal.connect(self.serial_worker_finished)
            # self.serial_work.mistake_message_transmit.connect(self.mistake_message_show)
            # self.serial_work.speed_signal.connect(self.speedinformation_show)
            # self.serial_work.start()

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

class Worker_AutoDriving(QThread):
    update_signal = pyqtSignal(int)
    target_ind_signal = pyqtSignal(int, float)
    control_state_signal = pyqtSignal(float, int)
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
        DB.flag_goal = False

        DB.target_ind = 0
        DB.last_ind = 0

        di_prev = 0.0

        goal = [DB.cx[-1], DB.cy[-1]]

        state = Simu.State(x=DB.RT_x, y=DB.RT_y, yaw=Fun.remap_heading_angle(DB.RT_course), v=DB.RT_speed)
        e, e_th, target_ind, last_ind= 0.0, 0.0, 0, 0

        '''
        循环开始
        '''
        while self.is_running:

            DB.target_ind, _ , _ = Fun.calc_nearest_index(state, DB.cx, DB.cy, DB.cyaw)
            x, y = DB.cx[-1], DB.cy[-1]
            distance = np.hypot(state.x - x, state.y - y)
            self.target_ind_signal.emit(DB.target_ind, float(distance))
            

            # 状态更新
            # with DB.data_lock:
            state.x = DB.RT_x
            state.y = DB.RT_y
            state.yaw = Fun.remap_heading_angle(DB.RT_course)
            state.v = DB.RT_speed

            if DB.flag_Auto :
                # if (DB.RT_speed > 0.05):
                if (True):
                    vr, vl, target_ind, e, e_th = Simu.lqr_steering_control(
                                state, DB.cx, DB.cy, DB.cyaw, DB.ck, e, e_th)

                    # di, outputs = self.forward_control(vr,vl,e,e_th,DB.forward_back_flag)
                    di, outputs = self.PI_control(vr,vl,DB.forward_back_flag)

                    # di, outputs = self.testPI_control(DB.setpoint, DB.forward_back_flag)
                    # 左右转换时保证舵机先回零
                    if di * di_prev < 0 :
                        di, outputs = 0, 500
                    di_prev = di

                    message = Fun.create_can_2_message(di,int(outputs))
                    self.bus.send(message)

            self.control_state_signal.emit(di, int(outputs))
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
            outputs = Fun.map_value(abs(di), DB.input_range_2, DB.output_range_2, DB.min_output, DB.max_output_2)
        elif -0.06 <= e <= 0.27:
            outputs = Fun.map_value(abs(di), DB.input_range_3, DB.output_range_3, DB.min_output, DB.max_output_3)
        else:
            outputs = Fun.map_value(abs(di), DB.input_range_4, DB.output_range_4, DB.min_output, DB.max_output_4)
        
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
        self.current_var = delta
        
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt > 1:
            dt = 0.1
        error = self.setpoint - self.current_var
        error = abs(error)
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        control_output = DB.K_p * error + DB.K_i * self.integral + DB.K_d * derivative
        self.prev_time = current_time
        self.prev_error = error
        outputs = int(max(DB.u_min, min(control_output, DB.u_max)))
        
        return di, outputs
    
    def testPI_control(self, setpoint, direction):
        if setpoint == 0:
            di, outputs = 0, 500
            return di, outputs
        else:
            if direction == 1:
                di = setpoint    # 右
                delta = DB.RT_right - DB.RT_left
            else:
                di = -setpoint    # 左
                delta = DB.RT_left - DB.RT_right

            self.setpoint = di
            self.current_var = delta
            
            current_time = time.time()
            dt = current_time - self.prev_time
            if dt > 1:
                dt = 0.1
            error = self.setpoint - self.current_var
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            control_output = DB.K_p * error + DB.K_i * self.integral + DB.K_d * derivative
            self.prev_time = current_time
            self.prev_error = error
            outputs = int(max(DB.u_min, min(control_output, DB.u_max)))
            
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
                        # with DB.data_lock:
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
                        try:
                            DB.z_axis_pals = self.Angular_Velocity_Calculation(DB.z_axis_angle)
                            DB.RT_left, DB.RT_right = self.calculate_track_speeds(DB.RT_speed,DB.z_axis_pals,2)
                        except:
                            print("MIMU初始化...")
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

