#!/usr/bin/env python3
import rospy
from std_msgs.msg import *
from pyqt_ros_example.srv import *

import sys
import os
import time
from threading import Thread

from PyQt5.QtWidgets import *
from PyQt5 import uic

from pyModbusTCP.client import ModbusClient



# TODO: Set UI Path
# dir = os.getcwd()
# qUIForm = uic.loadUiType(dir + "/src/pyqt_ros_example/ui/ui_file.ui")[0]
qUIForm = uic.loadUiType("/home/psh/kw_ws/src/pyqt_ros_example/ui/ui_file.ui")[0]

class UI(QMainWindow, qUIForm):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # @@@@@@@@@@@@@@@@@@@@@@
        # @@@@@@ For  ROS @@@@@@
        # @@@@@@@@@@@@@@@@@@@@@@
        rospy.init_node('talker', anonymous=True)
        # Message Publisher
        self._pub = rospy.Publisher('/test', Int64, queue_size= 10)
        
        # service client
        # rospy.wait_for_service('/test_service')
        self._rosClentDrawingType = rospy.ServiceProxy('/drawing_type',drawing_type)
        # TODO: Add Client Class

        # @@@@@@@@@@@@@@@@@@@@@@
        # @@@@@@@ Set UI @@@@@@@
        # @@@@@@@@@@@@@@@@@@@@@@
        self._mLogList = [] # 14개 들어감
        self.pushButtonSelect.clicked.connect(self._setMode)
        self.pushButtonClearLog.clicked.connect(self._clearLog)

    def _setMode(self):
        nMode = -1
        sMode = ""
        if self.radioButtonMode0.isChecked():
            nMode = 0
            sMode = "Rectangle"
        elif self.radioButtonMode1.isChecked():
            nMode = 1
            sMode = "Circle"
        elif self.radioButtonMode2.isChecked():
            nMode = 2
            sMode = "Camera"

        if nMode == -1:
            self._setLog("Select Mode First")
            return
        
        self._setLog("[UI]Set Mode: " + sMode)
        
        # mesage publisher
        self._pub.publish(nMode) 
        # service client
        req_set_mode = drawing_typeRequest()
        req_set_mode.type = nMode
        res = self._rosClentDrawingType(req_set_mode)

        if res :
            if res.set_success:
                self._setLog("[PROCESS]Set mode complete")
            else:
                self._setLog("[WARN]Try again set mode")
        else:
            self._setLog("[ERROR]Try again set mode")
        self._setLog("--------------------")



    # @@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For UI @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@
    def _clearLog(self):
        self._mLogList.clear()
    def _setLog(self, str): # Log 메시지 세팅 - 밑에서부터 올라오게 해놓음
        if len(self._mLogList) >= 14:
            self._mLogList.pop(0)
        self._mLogList.append(str)
    def _printLog(self):
        str = ""
        for i in self._mLogList:
            str+="\n"
            str+=i
        self.qlabelLog.setText(str)
    
    def statusThread(self):
        while True:
            self._printLog()
            time.sleep(0.01)

if __name__ == '__main__':
    try:
        qApp = QApplication(sys.argv)
        qt_ui = UI()

        th1 = Thread(target=qt_ui.statusThread, daemon=True)
        th1.start()

        qt_ui.show()
        if qApp.exec_():
            close = "close"
    except:
        pass


    pass