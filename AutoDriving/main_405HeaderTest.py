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

import numpy as np
import cubic_spline_planner
import simulation_pyCAN as Sim
# from openpyxl import Workbook
from datetime import datetime
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from UI_HeaderTest import Ui_MainWindow  # 导入ui.py 中的 Ui_MaiuinWindow 界面类
# from serial_manager import SerialManager
from pyproj import CRS, Transformer
import pandas as pd
import DataBase as DB


class MainWindow(QMainWindow, Ui_MainWindow):  # 继承 QMainWindow 类和 Ui_MainWindow 界面类
    def __init__(self, bus):
        super().__init__()
        self.setupUi(self)  # 继承 Ui_MainWindow 界面类
        if platform.system() == 'Linux':
            self.showMaximized()

        self.bus = bus

        self.pushButton_CAN.clicked.connect(self.click_pushButton_CAN)
        self.pushButton_HeaderRaise_PIDset.clicked.connect(self.click_pushButton_HeaderRaise_PIDset)
        self.pushButton_HeaderRaise_antiWindupset.clicked.connect(self.click_pushButton_HeaderRaise_antiWindupset)
        self.pushButton_HeaderLower_PIDset.clicked.connect(self.click_pushButton_HeaderLower_PIDset)
        self.pushButton_HeaderLower_antiWindupset.clicked.connect(self.click_pushButton_HeaderLower_antiWindupset)
        self.pushButton_HeaderPosition.clicked.connect(self.click_pushButton_HeaderPosition)
        self.pushButton_ReelPosition.clicked.connect(self.click_pushButton_ReelPosition)
        self.pushButton_Header_Auto_Manual.clicked.connect(self.click_pushButton_Header_Auto_Manual_Switch)
        self.pushButton_Header_UP.pressed.connect(self.pressed_pushButton_Header_UP)
        self.pushButton_Header_UP.released.connect(self.released_pushButton_Header_UP)
        self.pushButton_Header_Down.pressed.connect(self.pressed_pushButton_Header_Down)
        self.pushButton_Header_Down.released.connect(self.released_pushButton_Header_Down)
        self.pushButton_Reel_UP.pressed.connect(self.pressed_pushButton_Reel_UP)
        self.pushButton_Reel_UP.released.connect(self.released_pushButton_Reel_UP)
        self.pushButton_Reel_Down.pressed.connect(self.pressed_pushButton_Reel_Down)
        self.pushButton_Reel_Down.released.connect(self.released_pushButton_Reel_Down)


        crs_wgs = CRS("EPSG:4326")
        crs_gauss_kruger = CRS("EPSG:4547")
        # EPSG:4547，中国大陆2000国家大地坐标系 / 3度带投影 / 第19带
        # 基于CGCS2000（中国大陆2000国家大地坐标系）的高斯-克吕格（Gauss-Kruger）投影系统，用于第19带，每带覆盖3度经度。 
        # 数据和地图通常会被投影到以东经108度为中心的区域（即第19带），采用CGCS2000作为其地理坐标系统的基准。
        self.transformer = Transformer.from_crs(crs_wgs, crs_gauss_kruger)

        self.can_thread = Worker_CAN(self.bus)


#region pushButton CAN operation

#region pushButton CAN operation Header_Reel Manual operation

    def pressed_pushButton_Header_UP(self):
        # 割台升按钮按下时执行的代码
        try:
            speed_data = self.lineEdit_Header_UP_Param.text()
            if 0 <= int(speed_data) <= 10000: 
                data = [0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[2] = (int(speed_data) >> 8) & 0xFF
                data[3] = int(speed_data) & 0xFF
                message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)

                self.bus.send(message)
            else:
                print("数字不对")
        except ValueError:  # 如果转换失败（即文本不是一个整数）
            print("数字转换出错")

    def released_pushButton_Header_UP(self):
        # 割台升按钮释放时执行的代码
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)

        self.bus.send(message)

    def pressed_pushButton_Header_Down(self):
        # 割台降按钮按下时执行的代码
        try:
            speed_data = self.lineEdit_Header_Down_Param.text()
            if 0 <= int(speed_data) <= 10000: 
                data = [0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[4] = (int(speed_data) >> 8) & 0xFF
                data[5] = int(speed_data) & 0xFF
                message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)

                self.bus.send(message)
            else:
                print("数字不对")
        except ValueError:  # 如果转换失败（即文本不是一个整数）
            print("数字转换出错")
        
    def released_pushButton_Header_Down(self):
        # 割台降按钮释放时执行的代码
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F357F2, data=data, is_extended_id=True)

        self.bus.send(message)

    def pressed_pushButton_Reel_UP(self):
        # 拨禾轮升按钮按下时执行的代码
        try:
            speed_data = self.lineEdit_Reel_UP_Param.text()
            if 0 <= int(speed_data) <= 10000: 
                data = [0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[2] = (int(speed_data) >> 8) & 0xFF
                data[3] = int(speed_data) & 0xFF
                message = can.Message(arbitration_id=0x9F358F2, data=data, is_extended_id=True)

                self.bus.send(message)
            else:
                print("数字不对")
        except ValueError:  # 如果转换失败（即文本不是一个整数）
            print("数字转换出错")
        
    def released_pushButton_Reel_UP(self):
        # 拨禾轮升按钮释放时执行的代码
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F358F2, data=data, is_extended_id=True)

        self.bus.send(message)

    def pressed_pushButton_Reel_Down(self):
        # 拨禾轮降按钮按下时执行的代码
        try:
            speed_data = self.lineEdit_Reel_Down_Param.text()
            if 0 <= int(speed_data) <= 10000: 
                data = [0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[4] = (int(speed_data) >> 8) & 0xFF
                data[5] = int(speed_data) & 0xFF
                message = can.Message(arbitration_id=0x9F358F2, data=data, is_extended_id=True)

                self.bus.send(message)
            else:
                print("数字不对")
        except ValueError:  # 如果转换失败（即文本不是一个整数）
            print("数字转换出错")
        
    def released_pushButton_Reel_Down(self):
        # 拨禾轮降按钮释放时执行的代码
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        message = can.Message(arbitration_id=0x9F358F2, data=data, is_extended_id=True)

        self.bus.send(message)
#endregion

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

        elif(msg.arbitration_id == 11111):
            self.label_ADC.setText(f"CAN超时")

    def closeEvent(self, event):
        if self.can_thread.isRunning():
            self.can_thread.stop()
            self.can_thread.wait()
        self.bus.shutdown()
        super().closeEvent(event)

    def click_pushButton_Header_Auto_Manual_Switch(self):
        if(self.pushButton_Header_Auto_Manual.text()=="手动"):
            try:
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                msg = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                self.pushButton_Header_Auto_Manual.setText("自动")
                # QMessageBox.information(self, 'Message', '不为空')
            except :
                QMessageBox.information(self, 'Message', 'CAN出错')
        else :
            try:
                data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                msg = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                self.pushButton_Header_Auto_Manual.setText("手动")
                # QMessageBox.information(self, 'Message', '不为空')
            except :
                QMessageBox.information(self, 'Message', 'CAN出错')

    def click_pushButton_ReelPosition(self):
        try:
            if(self.lineEdit_ReelPosition.text()):
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[3] = (int(self.lineEdit_ReelPosition.text()) >> 8) & 0xFF
                data[4] = int(self.lineEdit_ReelPosition.text()) & 0xFF
                msg = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                self.pushButton_Header_Auto_Manual.setText("自动")
                # QMessageBox.information(self, 'Message', '不为空')
            else:
                QMessageBox.information(self, 'Message', '不能为空')
        except :
            QMessageBox.information(self, 'Message', 'CAN出错')

    def click_pushButton_HeaderPosition(self):
        try:
            if(self.lineEdit_HeaderPosition.text()):
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = (int(self.lineEdit_HeaderPosition.text()) >> 8) & 0xFF
                data[2] = int(self.lineEdit_HeaderPosition.text()) & 0xFF
                msg = can.Message(arbitration_id=0x9F352F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                self.pushButton_Header_Auto_Manual.setText("自动")
                # QMessageBox.information(self, 'Message', '不为空')
            else:
                QMessageBox.information(self, 'Message', '不能为空')
        except :
            QMessageBox.information(self, 'Message', 'CAN出错')

    def click_pushButton_HeaderLower_antiWindupset(self):
        try:
            if(self.lineEdit_HeaderLower_Pwindup.text() and self.lineEdit_HeaderLower_Iwindup.text() and self.lineEdit_HeaderLower_Dwindup.text()):
                data = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = int(self.lineEdit_HeaderLower_Pwindup.text()) & 0xFF
                data[2] = int(self.lineEdit_HeaderLower_Iwindup.text()) & 0xFF
                data[3] = int(self.lineEdit_HeaderLower_Dwindup.text()) & 0xFF
                msg = can.Message(arbitration_id=0x9F356F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                # QMessageBox.information(self, 'Message', '不为空')
            else:
                QMessageBox.information(self, 'Message', '不能为空')
        except :
            QMessageBox.information(self, 'Message', 'CAN出错')

    def click_pushButton_HeaderLower_PIDset(self):
        try:
            if(self.lineEdit_HeaderLower_P.text() and self.lineEdit_HeaderLower_I.text() and self.lineEdit_HeaderLower_D.text()):
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = (int(self.lineEdit_HeaderLower_P.text()) >> 8) & 0xFF
                data[2] = int(self.lineEdit_HeaderLower_P.text()) & 0xFF
                data[3] = (int(self.lineEdit_HeaderLower_I.text()) >> 8) & 0xFF
                data[4] = int(self.lineEdit_HeaderLower_I.text()) & 0xFF
                data[5] = (int(self.lineEdit_HeaderLower_D.text()) >> 8) & 0xFF
                data[6] = int(self.lineEdit_HeaderLower_D.text()) & 0xFF
                msg = can.Message(arbitration_id=0x9F356F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                # QMessageBox.information(self, 'Message', '不为空')
            else:
                QMessageBox.information(self, 'Message', '不能为空')
        except :
            QMessageBox.information(self, 'Message', 'CAN出错')

    def click_pushButton_HeaderRaise_antiWindupset(self):
        try:
            if(self.lineEdit_HeaderRaise_Pwindup.text() and self.lineEdit_HeaderRaise_Iwindup.text() and self.lineEdit_HeaderRaise_Dwindup.text()):
                data = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = int(self.lineEdit_HeaderRaise_Pwindup.text()) & 0xFF
                data[2] = int(self.lineEdit_HeaderRaise_Iwindup.text()) & 0xFF
                data[3] = int(self.lineEdit_HeaderRaise_Dwindup.text()) & 0xFF
                msg = can.Message(arbitration_id=0x9F355F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                # QMessageBox.information(self, 'Message', '不为空')
            else:
                QMessageBox.information(self, 'Message', '不能为空')
        except :
            QMessageBox.information(self, 'Message', 'CAN出错')

    def click_pushButton_HeaderRaise_PIDset(self):
        try:
            if(self.lineEdit_HeaderRaise_P.text() and self.lineEdit_HeaderRaise_I.text() and self.lineEdit_HeaderRaise_D.text()):
                data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                data[1] = (int(self.lineEdit_HeaderRaise_P.text()) >> 8) & 0xFF
                data[2] = int(self.lineEdit_HeaderRaise_P.text()) & 0xFF
                data[3] = (int(self.lineEdit_HeaderRaise_I.text()) >> 8) & 0xFF
                data[4] = int(self.lineEdit_HeaderRaise_I.text()) & 0xFF
                data[5] = (int(self.lineEdit_HeaderRaise_D.text()) >> 8) & 0xFF
                data[6] = int(self.lineEdit_HeaderRaise_D.text()) & 0xFF
                msg = can.Message(arbitration_id=0x9F355F2, data=data, is_extended_id=True)
                self.bus.send(msg)
                # QMessageBox.information(self, 'Message', '不为空')
            else:
                QMessageBox.information(self, 'Message', '不能为空')
        except :
            QMessageBox.information(self, 'Message', 'CAN出错')

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

def main():
    # QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)   #启用高DPI（每英寸点数）缩放
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

