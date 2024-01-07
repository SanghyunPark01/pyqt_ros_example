#!/usr/bin/env python3
import rospy
from std_msgs.msg import *
from pyqt_ros_example.srv import *
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped


import sys
import os
import time
from threading import Thread
import threading
import signal
import cv2
import numpy as np

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QCoreApplication
from PyQt5.QtGui import *

from pyModbusTCP.client import ModbusClient



# TODO: Set UI Path
# dir = os.getcwd()
# qUIForm = uic.loadUiType(dir + "/src/pyqt_ros_example/ui/ui_file.ui")[0]
# qUIForm = uic.loadUiType("/home/psh/kw_ws/src/pyqt_ros_example/ui/ui_file_with_stream.ui")[0]
qUIForm = uic.loadUiType("/home/psh/kw_ws/src/pyqt_ros_example/ui/ui_file.ui")[0]

class UI(QMainWindow, qUIForm):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        img_logo = cv2.imread("/home/psh/kw_ws/src/pyqt_ros_example/ui/Doosan_Logo.jpg",cv2.IMREAD_COLOR)
        # cv2.cvtColor(img_logo,cv2.COLOR_BGR2RGB)
        pixmap__ = covert_cv_qt0(img_logo)
        self.qlabelLogo.setPixmap(pixmap__)
        # IP
        self._mStatus = False # IP연결 여부
        self.msIPAddress = ""
        self.msIPConnectedStatus = ""
        self.qLineEditIP.textChanged.connect(self._setIPAddress)
        self.qpushbuttonConnectIP.clicked.connect(self._connectIPAddress)

        # @@@@@@@@@@@@@@@@@@@@@@
        # @@@@@@ For  ROS @@@@@@
        # @@@@@@@@@@@@@@@@@@@@@@
        rospy.init_node('talker', anonymous=True)
        self.mtx1 = threading.Lock()
        self.mtx2 = threading.Lock()
        self.mtx3 = threading.Lock()
        # Initialize
        imgEmpty = np.ones((480, 640), dtype=np.uint8) * 255
        pixmap = convert_cv_qt_gray(imgEmpty)
        self.labelLineImg.setPixmap(pixmap)
        # service client
        self._rosClentDrawingType = rospy.ServiceProxy('/drawing_type',drawing_type)
        self._rosClentSetImgFlag = rospy.ServiceProxy('/set_img_flag',set_img_flag)
        self._rosClentStartCalculateFlag = rospy.ServiceProxy('/calcuate_flag',start_calculate_flag)
        self._rosClentStartDrawingFlag = rospy.ServiceProxy('/drawing_start_flag',start_drawing_flag)
        # sub msg
        self._rosSubCapturedImg = rospy.Subscriber('/captured_img',Image, self._callbackCaputuredImg, queue_size=10)
        self._mbCamIsReady = False
        self._rosSubCamStream = rospy.Subscriber('/camera/color/image_raw',Image, self._callbackStream, queue_size=1000)
        self._pubShutdown = rospy.Publisher('/shutdown_flag',Bool,queue_size=10)
        self._subDrawingPath = rospy.Subscriber('/drawing_path',Path, self._callbackDrawingPath, queue_size=10)
        self.mMovePathSize = 0
        self._mListDrawingPath = []
        self._mbPathFlag = False
        # @@@@@@@@@@@@@@@@@@@@@@
        # @@@@@@@ Set UI @@@@@@@
        # @@@@@@@@@@@@@@@@@@@@@@
        self._mLogList = [] # 14개 들어감
        self.pushButtonClearLog.clicked.connect(self._clearLog)
        #
        self._mnMode = -1
        self.pushButtonSelect.clicked.connect(self._setMode)
        self._mbIsCaptured = False
        self.pushButtonCapture.clicked.connect(self._captureTime)
        self._mnSiderBarValue = 0
        self.horizontalSliderLineImage.valueChanged.connect(self._setSliderValue)
        self.pushButtonPathCalcuate.clicked.connect(self._calculate)
        self._startFlag = True
        self.pushButtonStart.clicked.connect(self._startDrawing)
        self.pushButtonResetImage.clicked.connect(self._resetImg)
        self._mbPlayPathPause = True
        self.pushButtonPlayPath.clicked.connect(self._playPath)
        #
        self._mbPauseFlag = False
        self.pushButtonPauseNResume.clicked.connect(self._puase_resumeSystem)
        #
        self.pushButtonShutDown.clicked.connect(self._shutdownSys)

        

    # @@@@@@@@@@@@@@@@@@
    # @@@@@@@ UI @@@@@@@
    # @@@@@@@@@@@@@@@@@@
    def _startDrawing(self):
        DrawingStartFlag = True
        if not self._mStatus:
            self._setLog("[UI]Connect IP first")
            return
        self._startFlag = False
        time.sleep(0.1)
        if self._mModbusClient.read_holding_registers(210,1)[0] is not None and self._mModbusClient.read_holding_registers(210,1)[0]:
            if self._mModbusClient.read_holding_registers(210,1)[0] != 1:
                self._startFlag = True
                self._setLog("[ROBOT]Robot is busy!")
                return

        self._setLog("[UI]Start Drawing")

        # # service client
        # req_startFlag = start_drawing_flagRequest()
        # req_startFlag.start_flag = DrawingStartFlag
        # res = self._rosClentStartDrawingFlag(req_startFlag)

        # if res :
        #     if res.set_success:
        #         self._setLog("[PROCESS]Drawing Start")
        #     else:
        #         self._setLog("[WARN]Can't Start")
        # else:
        #     self._setLog("[ERROR]Can't Start")
        # self._setLog("--------------------")
        self._startFlag = False
        # TODO: Modbus // -1을 65535, -2를 65534
        # 128 데이터 갯수
        self._mModbusClient.write_single_register(128,len(self._mListDrawingPath))
        print(len(self._mListDrawingPath))
        # 129 x // 130 y
        # 131 pc to robot, 132 robot to pc
        num = 1
        for point in self._mListDrawingPath:
            time.sleep(0.01)
            while True:
                if not self._mModbusClient.read_holding_registers(132,1)[0] or self._mModbusClient.read_holding_registers(132,1)[0] is None:
                    continue
                if num == self._mModbusClient.read_holding_registers(132,1)[0]:
                    break
            print(num)
            if point[0] == -1:
                self._mModbusClient.write_single_register(129,65535)
                self._mModbusClient.write_single_register(130,65535)
            elif point[0] == -2:
                self._mModbusClient.write_single_register(129,65534)
                self._mModbusClient.write_single_register(130,65534)
            else :
                self._mModbusClient.write_single_register(129,int(639-point[0]))
                self._mModbusClient.write_single_register(130,int(point[1]))
            time.sleep(0.01)
            self._mModbusClient.write_single_register(131,num)
            num += 1
        self._startFlag = True


    def _playPath(self):
        if self._mbPlayPathPause:
            self._mbPlayPathPause = False
            self.pushButtonPlayPath.setText("II")
        else :
            self._mbPlayPathPause = True
            self.pushButtonPlayPath.setText(">")

    def _resetImg(self):
        cv_image = np.zeros((480,640,3), np.uint8)
        pixmap = covert_cv_qt(cv_image)
        self.labelCaptured.setPixmap(pixmap)
        self._setLog("[PROCESS]Reset Image")
        self._mbIsCaptured = False
        self._mbPathFlag = False
    
    def _puase_resumeSystem(self):
        if self._mbPauseFlag:
            # TODO: 로봇으로 resume 보내기
            self._mbPauseFlag = False
            self.pushButtonPauseNResume.setText("Pause")
            self._setLog("[ROBOT]Resume Robot")
        else :
            # TODO: 로봇으로 pause보내기
            self._mbPauseFlag = True
            self.pushButtonPauseNResume.setText("Resume")
            self._setLog("[ROBOT]Pause Robot")
        return
    
    def _setMode(self):
        self._mnMode = -1
        sMode = ""
        if self.radioButtonMode0.isChecked():
            self._mnMode = 0
            sMode = "Rectangle"
        elif self.radioButtonMode1.isChecked():
            self._mnMode = 1
            sMode = "Circle"
        elif self.radioButtonMode2.isChecked():
            if self._mbCamIsReady:
                self._mnMode = 2
                sMode = "Camera"
            else :
                self._setLog("[PROCESS]Cam is not ready")
                return
        elif self.radioButtonMode3.isChecked():
            self._mnMode = 3
            sMode = "Mouse"

        if self._mnMode == -1:
            self._setLog("[UI]Select Mode First")
            return
        
        self._setLog("[UI]Set Mode: " + sMode)
        
        # ROS
        req_set_mode = drawing_typeRequest()
        req_set_mode.type = self._mnMode
        res = self._rosClentDrawingType(req_set_mode)

        if res :
            if res.set_success:
                self._setLog("[PROCESS]Set mode complete")
            else:
                self._setLog("[WARN]Try again set mode")
        else:
            self._setLog("[ERROR]Try again set mode")
        self._mbPathFlag = False
        # self._setLog("--------------------")

    def _captureTime(self):
        if self._mnMode == -1:
            self._setLog("[UI]Select Mode First")
            return
        captureTime = rospy.Time.now()
        
        if captureTime == 0:
            self._setLog("invalid capture time")
            return
        # self._setLog("Capture Success")

        # ROS
        self.mtx3.acquire()
        req_captime = set_img_flagRequest()
        req_captime.set_image_flag_time = captureTime
        res = self._rosClentSetImgFlag(req_captime)
        time.sleep(0.01)
        self.mtx3.release()
        if res :
            if res.set_success:
                self._setLog("[PROCESS]Image Capture complete")
                self._mbIsCaptured = True
            else:
                self._setLog("[WARN]Try again Image Capture")
        else:
            self._setLog("[ERROR]Try again Image Capture")
        # self._setLog("--------------------")
        sMode = ""
        if self._mnMode == 0:
            sMode = "Rectangle"
        elif self._mnMode == 1:
            sMode = "Circle"
        elif self._mnMode == 2:
            sMode = "Camera"
        elif self._mnMode == 3:
            sMode = "Mouse"
        self.label_5.setText(sMode)
        self._mbPathFlag = False
        
    def _calculate(self):
        calculateStartFlag = True
        
        if not self._mbIsCaptured:
            self._setLog("[UI]Capture image first!")
            return
        self._setLog("[UI]Start Calculate")
        # ROS
        self.mtx3.acquire()
        req_calculFlag = start_calculate_flagRequest()
        req_calculFlag.start_flag = calculateStartFlag
        res = self._rosClentStartCalculateFlag(req_calculFlag)
        time.sleep(0.01)
        self.mtx3.release()
        if res :
            if res.set_success:
                s = "good"
            else:
                self._setLog("[WARN]Try again Calculate")
        else:
            self._setLog("[ERROR]Try again Calculate")
        # self._setLog("--------------------")
        self._mbIsCaptured = False
        
    


    def _setSliderValue(self, v):
        self.mtx1.acquire()
        self._mnSiderBarValue = v
        imgEmpty = np.ones((480, 640), dtype=np.uint8) * 255
        prevpoint_x = -1
        prevpoint_y = -1
        firstFlag = True
        if self._mbPathFlag and len(self._mListDrawingPath) != 0:
            max_num = self.mMovePathSize*(v/1000)
            num = 0
            for path in self._mListDrawingPath:
                # imgEmpty[int(path[1]),int(path[0])] = 0 # row col
                if path[0] < 0 or path[1] < 0:
                    prevpoint_x = -1
                    prevpoint_y = -1
                    firstFlag = True
                    continue
                if num >= max_num:
                    break
                if firstFlag:
                    prevpoint_x = path[0]
                    prevpoint_y = path[1]
                    firstFlag = False
                    continue
                # cv2.circle(imgEmpty,(int(path[0]),int(path[1])),2,0,-1)
                cv2.line(imgEmpty,(int(prevpoint_x),int(prevpoint_y)),(int(path[0]),int(path[1])),0,2)
                prevpoint_x = path[0]
                prevpoint_y = path[1]
                num += 1

        pixmap = convert_cv_qt_gray(imgEmpty)
        self.labelLineImg.setPixmap(pixmap)
        self.mtx1.release()

    # ==============================
    #       Set & Connect IP
    # ==============================
    def _setIPAddress(self):
        self.msIPAddress = self.qLineEditIP.text()
    def _connectIPAddress(self):
        gt = "0123456789."
        gt2 = "0123456789"
        dotNum = 0
        dotIdx = []
        # 잘못된 IP 주소 - 숫자와 점이 아닌 문자
        idx = 0
        for tmp in self.msIPAddress: 
            if gt.find(tmp) == -1:
                self.msIPConnectedStatus = "Wrong IP address... See Log"
                self._setLog("[Error] " + self.msIPAddress + " is wrong address. Check IP Address")
                return
            if tmp == ".":
                dotIdx.append(idx)
                dotNum += 1
            idx += 1
        # print(dotIdx)
        # 잘못된 IP 주소 - 점의 갯수 부족
        if dotNum != 3: 
            self.msIPConnectedStatus = "Wrong IP address... See Log"
            self._setLog("[Error] " + self.msIPAddress + " is wrong address. Check IP Address")
            return
        # 잘못된 IP 주소 - 비정상적 점의 배치
        for tmpIdx in dotIdx:
            if tmpIdx == 0 or tmpIdx == len(self.msIPAddress)-1 \
                or gt2.find(self.msIPAddress[tmpIdx-1]) == -1 \
                or gt2.find(self.msIPAddress[tmpIdx+1]) == -1:
                self.msIPConnectedStatus = "Wrong IP address... See Log"
                self._setLog("[Error] " + self.msIPAddress + " is wrong address. Check IP Address")
                return
        self._mModbusClient = ModbusClient(host = self.msIPAddress, port=502, auto_open=True, debug=False, timeout=0.5)
        if(self._mModbusClient.read_coils(0, 10)):
            self.msIPConnectedStatus = "Connect Success!"
            self._mStatus = True
            self._setLog("[Success] " + self.msIPAddress + " Connecting Success!")
            self._mModbusClient.write_single_register(128,0)
        else:
            self.msIPConnectedStatus = "Connecting failed... See Log"
            self._mStatus = False
            self._setLog("[Error] " + self.msIPAddress + " Connecting Failed. Check IP Address")
        self.qlabelConnectStatus.setText(self.msIPConnectedStatus)
        
    # @@@@@@@@@@@@@@@@@@@@@@
    # @@@@@@@ For UI @@@@@@@
    # @@@@@@@@@@@@@@@@@@@@@@
    def _shutdownSys(self):
        self._pubShutdown.publish(True)
        QCoreApplication.instance().quit()
        # rospy.on_shutdown(print())
    def _callbackStream(self, msg): # 에러
        if not self._mbCamIsReady:
            self._mbCamIsReady = True
            self._setLog("[PROCESS]Cam is ready!")
        # bridge = CvBridge()
        # cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.mtx2.acquire()
        # pixmap_ = covert_cv_qt1(cv_image)
        # self.labelCamStream.setPixmap(pixmap_)
        # self.mtx2.release()

    def _callbackCaputuredImg(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.mtx2.acquire()
        pixmap = covert_cv_qt(cv_image)
        self.labelCaptured.setPixmap(pixmap)
        self.mtx2.release()

    def _callbackDrawingPath(self, msg):
        self._mListDrawingPath.clear()
        ListPath = []
        # (x,y) => (col,row)
        for poseStamp in msg.poses:
            # print(poseStamp.pose.position.x, poseStamp.pose.position.y)
            pose = [poseStamp.pose.position.x, poseStamp.pose.position.y]
            ListPath.append(pose)
        self._mListDrawingPath = ListPath
        self.mMovePathSize = len(self._mListDrawingPath)
        for path in self._mListDrawingPath:
            if path[0] < 0 or path[1] < 0:
                self.mMovePathSize -= 1
        self._mbPathFlag = True
        self._setLog("[PROCESS]Calculate complete")

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
    def _printJoint(self):
        if not self._mStatus:
            return
        if not self._startFlag:
            return
        # lock.acquire()
        lJointPosition = self._mModbusClient.read_holding_registers(270,6)
        # lock.release()
        if not lJointPosition and lJointPosition is None:
            return
        for i in range(len(lJointPosition)):
            if lJointPosition[i] > 30000:
                lJointPosition[i] = (lJointPosition[i] - 65535) / 10
            else:
                lJointPosition[i] /= 10
        self.qJoint1.setText(str(lJointPosition[0]))
        self.qJoint2.setText(str(lJointPosition[1]))
        self.qJoint3.setText(str(lJointPosition[2]))
        self.qJoint4.setText(str(lJointPosition[3]))
        self.qJoint5.setText(str(lJointPosition[4]))
        self.qJoint6.setText(str(lJointPosition[5]))
    
    def statusThread(self):
        while True:
            self._printLog()
            self._printJoint()
            time.sleep(0.01)
    def camThread(self):
        while True:
            self.mtx1.acquire()
            if not self._mbPlayPathPause:
                if self._mnSiderBarValue < 1000:
                    self._mnSiderBarValue += 5
                self.horizontalSliderLineImage.setValue(self._mnSiderBarValue)
            if self._mnSiderBarValue == 1000:
                # self._mnSiderBarValue = 0
                self.horizontalSliderLineImage.setValue(self._mnSiderBarValue)
                self._mbPlayPathPause = True
                self.pushButtonPlayPath.setText(">")
            self.mtx1.release()
            time.sleep(0.01)

def covert_cv_qt0(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w,c = cvtd_img.shape
    qimg = QImage(cvtd_img.data, w, h, w*c, QImage.Format_RGB888)
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01
def covert_cv_qt(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    qimg = QImage(cvtd_img.data, cvtd_img.shape[1], cvtd_img.shape[0], QImage.Format_RGB888)
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01
def covert_cv_qt1(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w,c = cvtd_img.shape
    qimg = QImage(cvtd_img.data, w, h, w*c, QImage.Format_RGB888)
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01
def convert_cv_qt_gray(cv_img):
    qimg = QImage(cv_img.data, cv_img.shape[1], cv_img.shape[0], QImage.Format_Grayscale8)
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01

if __name__ == '__main__':
    
    try:
        qApp = QApplication(sys.argv)
        qt_ui = UI()

        th1 = Thread(target=qt_ui.statusThread, daemon=True)
        th1.start()
        th2 = Thread(target=qt_ui.camThread, daemon=True)
        th2.start()

        qt_ui.show()
        
        if qApp.exec_():
            close = "close"
    except:
        pass


    pass