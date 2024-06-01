import sys
import time
import math
import cv2
from common import DAQ, PID, Robot
from util import openFlirCamera
from PyQt5.QtCore import pyqtSignal, QTimer, Qt
from PyQt5.Qt import QApplication, QWidget, QThread
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtWidgets, uic


class TextBrowserThread(QThread):
    # 实时显示追加线程（要继承QThread， 继承threading.Thread不行）
    signal = pyqtSignal()  # 信号

    def __init__(self):
        super().__init__()

    def run(self):
        while True:
            self.signal.emit()  # 发射信号(实参类型要和定义信号的参数类型一致)
            time.sleep(1)


class VideoThread(QThread):
    # 实时显示追加线程（要继承QThread， 继承threading.Thread不行）
    signal = pyqtSignal()  # 信号

    def __init__(self, robot, frame_width=1920, frame_height=1080):
        super().__init__()
        # self.cap = cv2.VideoCapture(0)
        self.cap = openFlirCamera()
        self.robot = robot
        self.frame = None
        self.frame_width, self.frame_height = frame_width, frame_height

    def run(self):
        # self.cap = cv2.VideoCapture(0)
        self.cap = openFlirCamera()
        try:
            while self.cap.isOpened():
                ret, self.frame = self.cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break

                self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
                # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
                self.frame = cv2.resize(self.frame, (self.frame_width, self.frame_height))
                # print("hhhh")
                self.robot.process_frame(self.frame)
                self.robot.show_robot_frame(self.frame)
                # Retrieve and print the current robot position
                # robot_position = self.robot.get_robot_position()
                # print(f"Robot Position: {robot_position[0]}  {robot_position[1]}")

                # obstacles.find_all_obstacles(frame)
                # # Retrieve and print the current robot position
                # obstacles.show_all_obstacles(frame)

                # Get robot show_frame for visualization
                # cv2.imshow("img", self.frame)
                # self.frame = cv2.resize(self.frame, (491, 341))
                self.signal.emit()
                # print("ffff")
                time.sleep(0.01)

                # 等待按键，如果按下 'q' 键，就退出循环
                # if cv2.waitKey(10) & 0xFF == ord('q'):
                #     break
        except Exception as e:
            print(f"Exception in VideoThread: {e}")
        self.cap.release()
        print("video Release!!!!!")


class MouseKeyTracker(QtCore.QObject):
    positionChanged = QtCore.pyqtSignal(QtCore.QPoint)
    leftButtonClicked = QtCore.pyqtSignal(QtCore.QPoint)
    upKeyPressed = QtCore.pyqtSignal()
    downKeyPressed = QtCore.pyqtSignal()
    leftKeyPressed = QtCore.pyqtSignal()
    rightKeyPressed = QtCore.pyqtSignal()

    def __init__(self, widget):
        super().__init__(widget)
        self._widget = widget
        self.widget.setMouseTracking(True)
        self.widget.setFocusPolicy(Qt.StrongFocus)
        self.widget.installEventFilter(self)

    @property
    def widget(self):
        return self._widget

    def eventFilter(self, o, e):
        if o is self.widget:
            if e.type() == QtCore.QEvent.MouseMove:
                self.positionChanged.emit(e.pos())
            elif e.type() == QtCore.QEvent.MouseButtonPress and e.button() == QtCore.Qt.LeftButton:
                self.leftButtonClicked.emit(e.pos())
            elif e.type() == QtCore.QEvent.KeyPress:
                key = e.key()
                if key == Qt.Key_Up:
                    self.upKeyPressed.emit()
                elif key == Qt.Key_Down:
                    self.downKeyPressed.emit()
                elif key == Qt.Key_Left:
                    self.leftKeyPressed.emit()
                elif key == Qt.Key_Right:
                    self.rightKeyPressed.emit()

        return super().eventFilter(o, e)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("../QtUI/mainqt.ui")

        # f, beta, B edit初始化
        self.f_edit = self.ui.F_spinBox
        self.f_edit.setValue(10)
        self.beta_edit = self.ui.beta_spinBox
        self.beta_edit.setValue(0)
        self.B_edit = self.ui.B_spinBox
        self.B_edit.setValue(0)
        self.beta_dial_edit = self.ui.beta_dial
        self.beta_dial_edit.setValue(0)
        # 打印调试信息的textBrower
        self.textBrowser = self.ui.textBrowser
        self.ImgShowLabel = self.ui.ImgShow
        # 显示鼠标坐标的标签
        self.label_position = QtWidgets.QLabel(
            self.ImgShowLabel, alignment=QtCore.Qt.AlignCenter
        )
        self.label_position.setStyleSheet('background-color: white; border: 1px solid black')
        # 启动向目标前进的实时更新Flag
        self.update_flag = False

        # 机器人实际坐标 x,y 目标坐标mx my
        self.robot_x = self.ui.x_label
        self.robot_y = self.ui.y_label
        self.robot_mx = 0
        self.robot_my = 0
        self.robot_mx_edit = self.ui.x_spinBox
        self.robot_my_edit = self.ui.y_spinBox
        self.daq = DAQ(['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2'])
        # cap = cv2.VideoCapture(0)
        cap = openFlirCamera()
        time.sleep(3)
        self.frame_width, self.frame_height = 1920, 1080
        self.robot = Robot(cap, frame_width=self.frame_width, frame_height=self.frame_height)

        # 给Start Stop个按钮绑定槽函数
        self.ui.StartButton.clicked.connect(self.startDaq)  # 绑定槽函数
        self.ui.StopButton.clicked.connect(self.stopDaq)  # 绑定槽函数
        self.ui.Update_Button.clicked.connect(lambda: self.set_update_flag(True))
        self.ui.Unupdate_Button.clicked.connect(lambda: self.set_update_flag(False))
        self.ui.StartPIDButton.clicked.connect(self.startPID)
        self.ui.StopPIDButton.clicked.connect(self.stopPID)

        self.f_edit.valueChanged.connect(self.f_changed)
        self.beta_edit.valueChanged.connect(self.beta_changed)
        self.B_edit.valueChanged.connect(self.B_changed)
        self.beta_dial_edit.valueChanged.connect(self.beta_dial_changed)
        self.robot_mx_edit.valueChanged.connect(self.robot_mx_changed)
        self.robot_my_edit.valueChanged.connect(self.robot_my_changed)

        self.tracker = MouseKeyTracker(self.ImgShowLabel)
        self.tracker.positionChanged.connect(self.mousePositionChanged)
        self.tracker.leftButtonClicked.connect(self.mousePositionPressed)
        self.tracker.upKeyPressed.connect(lambda: self.beta_edit.setValue(180))
        self.tracker.downKeyPressed.connect(lambda: self.beta_edit.setValue(0))
        self.tracker.leftKeyPressed.connect(lambda: self.beta_edit.setValue(90))
        self.tracker.rightKeyPressed.connect(lambda: self.beta_edit.setValue(270))

        self.videoThread = VideoThread(self.robot)
        self.videoThread.signal.connect(self.refreshShow)
        self.videoThread.start()

        self.printThread = TextBrowserThread()
        self.printThread.signal.connect(self.slot_text_browser)
        self.printThread.start()

        # PID 初始化
        self.pid = PID(frame=None, x0=self.robot.get_robot_position()[0], y0=self.robot.get_robot_position()[1])
        # 创建一个定时器
        self.pidTimer = QTimer(self.ui)
        # 设置定时器的间隔时间（毫秒为单位）
        interval = 1000  # 1秒
        self.pidTimer.setInterval(interval)
        # 连接定时器的timeout信号到要执行的函数
        self.pidTimer.timeout.connect(self.update_pid)

        self.pid_t = 0
        self.position_list = None
        # # 创建一个定时器
        # self.daqTimer = QTimer(self.ui)
        # # 设置定时器的间隔时间（毫秒为单位）
        # interval1 = 1  # 1秒
        # self.daqTimer.setInterval(interval1)
        # # 连接定时器的timeout信号到要执行的函数
        # self.daqTimer.timeout.connect(self.update_daq)
        # # 启动定时器
        # self.daqTimer.start()

    def set_update_flag(self, value):
        self.update_flag = value

    def startDaq(self):
        self.daq.startDaq()
        self.daq.start()

    def stopDaq(self):
        self.daq.stopDaq()

    def startPID(self):
        print("START PID-----")
        self.pid.x0 = self.robot.get_robot_position()[0]
        self.pid.y0 = self.robot.get_robot_position()[1]
        self.pid_t = 0
        self.position_list = self.pid.plot_list()
        self.update_pid()
        # 启动定时器
        self.pidTimer.start()
        self.daq.startDaq()
        self.daq.start()

    def stopPID(self):
        print("STOP PID-------")
        self.daq.stopDaq()
        self.pidTimer.stop()

    def f_changed(self):
        num = self.f_edit.value()
        print("f:", num)
        self.daq.set_f(num)
        self.pid.set_f(num)

    def beta_changed(self):
        num = self.beta_edit.value()
        print("beta:", num)
        self.daq.set_beta(num)
        self.beta_dial_edit.blockSignals(True)
        self.beta_dial_edit.setValue(num)
        self.beta_dial_edit.blockSignals(False)

    def B_changed(self):
        num = self.B_edit.value()
        print("B:", num)
        self.daq.set_B(num)

    def beta_dial_changed(self):
        num = self.beta_dial_edit.value()
        print("beta_dial:", num)
        self.daq.set_beta(num)
        # 阻塞信号发送 setValue()方法会自动发送valueChanged信号问题
        self.beta_edit.blockSignals(True)
        self.beta_edit.setValue(num)
        self.beta_edit.blockSignals(False)

    def robot_mx_changed(self):
        self.robot_mx = self.robot_mx_edit.value()

    def robot_my_changed(self):
        self.robot_my = self.robot_my_edit.value()

    def update_second_beta(self):
        if self.update_flag:
            # print("flag -------")
            try:
                dx = self.robot_mx - self.robot.get_robot_position()[0]
                dy = self.robot_my - self.robot.get_robot_position()[1]
                # print(f"{dx} ------{dy}")
                beta = 0
                if abs(dx) < 10 and abs(dy) < 10:
                    print("Robot Finished！！！！！！！！ ")
                    self.daq.stopDaq()
                if dx > 0:
                    alpha = math.atan(dy / dx)
                    beta = int(270 - math.degrees(alpha))
                elif dx < 0:
                    alpha = math.atan(dy / dx)
                    beta = int(90 - math.degrees(alpha))
                else:
                    if dy < 0:
                        beta = 0
                    else:
                        bata = 180
                print("beta update:", beta)
                self.beta_dial_edit.blockSignals(True)
                self.beta_edit.blockSignals(True)
                self.beta_edit.setValue(beta)
                self.beta_dial_edit.setValue(beta)
                self.daq.set_beta(beta)
                self.beta_dial_edit.blockSignals(False)
                self.beta_edit.blockSignals(False)
            except Exception as e:
                print(f"update: {e}")

    def slot_text_browser(self):
        # text_browser槽函数
        # self.daq.set_beta(self.daq.get_beta() + 10)
        # self.textBrowser.clear()
        text = f'频率f：<font color="red">{self.daq.get_f()}</font> 角度beta: <font color="red">{self.daq.get_beta()}</font> 磁场强度B: <font color="red">{self.daq.get_B()}</font>'
        text_a = f'ao0: <font color="red">{self.daq.write_a0:.2f}</font> ao1: <font color="red">{self.daq.write_a1:.2f}</font> ao2: <font color="red">{self.daq.write_a2:.2f}</font>'
        text_robot = f'robot_mx: <font color="red">{self.robot_mx}</font> robot_my: <font color="red">{self.robot_my}</font>'
        self.textBrowser.append(text + '        ' + text_a + '        ' + text_robot)

        self.update_second_beta()


    def refreshShow(self):
        # 将 OpenCV 图像转换为 Qt 图像
        # print(self.videoThread.frame.shape)
        try:
            if self.position_list:
                for position in self.position_list:
                    cv2.circle(self.videoThread.frame, (position[0], 1080 - position[1]), 3, (0, 255, 0), -1)
        except Exception as e:
            print(f"plot: {e}")
        height, width, channel = self.videoThread.frame.shape
        # print(height + ">>" + width + ">>" + channel )
        bytes_per_line = 3 * width
        q_image = QImage(self.videoThread.frame.data, width, height, bytes_per_line, QImage.Format_BGR888)  # 设置颜色格式
        #
        # # # 将 Qt 图像设置为 QLabel 的背景
        self.ImgShowLabel.setPixmap(QPixmap.fromImage(q_image))

        self.robot_x.setText(str(self.robot.get_robot_position()[0]))
        self.robot_y.setText(str(self.robot.get_robot_position()[1]))

    @QtCore.pyqtSlot(QtCore.QPoint)
    def mousePositionChanged(self, pos):
        delta = QtCore.QPoint(30, -15)
        self.label_position.show()
        self.label_position.move(pos + delta)
        self.label_position.setText("(%d, %d)" % (pos.x(), 1080 - pos.y()))
        self.label_position.adjustSize()

    @QtCore.pyqtSlot(QtCore.QPoint)
    def mousePositionPressed(self, pos):
        self.robot_mx = pos.x()
        self.robot_my = 1080 - pos.y()
        self.robot_mx_edit.blockSignals(True)
        self.robot_my_edit.blockSignals(True)
        self.robot_mx_edit.setValue(self.robot_mx)
        self.robot_my_edit.setValue(self.robot_my)
        self.robot_mx_edit.blockSignals(False)
        self.robot_my_edit.blockSignals(False)

    def update_pid(self):
        # self.pid.pidPosition(self.robot.get_robot_position(), self.pid_t)
        try:
            beta, B = self.pid.pidPosition(self.robot.get_robot_position(), self.pid_t)
            self.beta_dial_edit.blockSignals(True)
            self.beta_edit.blockSignals(True)
            self.beta_edit.setValue(beta)
            self.beta_dial_edit.setValue(beta)
            self.daq.set_beta(beta)
            self.beta_dial_edit.blockSignals(False)
            self.beta_edit.blockSignals(False)
            self.B_edit.setValue(B)
            self.pid_t += 1
        except Exception as e:
            print(f"update_pid: {e}")

    def update_daq(self):
        self.daq.run_old()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    Window = MainWindow()
    Window.ui.show()
    app.exec()
