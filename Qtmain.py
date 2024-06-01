import sys
import time
import cv2
from common import DAQ, PID, Robot
from PyQt5.QtCore import pyqtSignal, QTimer, Qt
from PyQt5.Qt import QApplication, QWidget, QThread
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import QtCore, QtWidgets, uic
from util import openFlirCamera, get_contours


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

    def __init__(self, robot, frame_width=1920, frame_height=1080, img_frame=None, contours=None):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        # self.cap = openFlirCamera()
        self.robot = robot
        self.frame = None
        self.img_frame = img_frame
        self.contours = contours

        self.frame_width, self.frame_height = frame_width, frame_height

        # 是否展示机器人的位置
        self.show_tracking_robot = True
        self.show_tracking_path = False

    def run(self):
        # self.cap = cv2.VideoCapture(0)
        # self.cap = openFlirCamera()
        try:
            while self.cap.isOpened():

                ret, self.frame = self.cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break

                # self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
                self.frame = cv2.resize(self.frame, (self.frame_width, self.frame_height))
                if self.img_frame is not None:
                    self.frame = cv2.addWeighted(self.frame, 0.1, self.img_frame, 0.9, 0)
                if self.contours is not None:
                    cv2.drawContours(self.frame, self.contours, -1, (0, 0, 0), cv2.FILLED)

                # 追踪机器人
                self.robot.process_frame(self.frame)
                # 打印机器人
                if self.show_tracking_robot:
                    self.robot.show_robot_frame(self.frame, self.show_tracking_path)
                self.signal.emit()
                time.sleep(0.01)
        except Exception as e:
            print(f"Exception in VideoThread: {e}")
            self.cap.release()
            print("video Release!!!!!")


class MouseKeyTracker(QtCore.QObject):
    positionChanged = QtCore.pyqtSignal(QtCore.QPoint)
    upKeyPressed = QtCore.pyqtSignal()
    downKeyPressed = QtCore.pyqtSignal()
    leftKeyPressed = QtCore.pyqtSignal()
    rightKeyPressed = QtCore.pyqtSignal()
    spaceKeyPressed = QtCore.pyqtSignal()

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
                elif key == Qt.Key_Space:
                    self.spaceKeyPressed.emit()

        return super().eventFilter(o, e)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("./QtUI/mainqt.ui")

        # f, beta, B edit初始化
        self.f_edit = self.ui.F_spinBox
        self.f_edit.setValue(13)
        self.beta_edit = self.ui.beta_spinBox
        self.beta_edit.setValue(0)
        self.B_edit = self.ui.B_spinBox
        self.B_edit.setValue(0)
        self.alpha_edit = self.ui.alpha_spinBox
        self.alpha_edit.setValue(90)
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
        # 启动向鼠标点击的目标点前进的实时更新Flag
        self.update_flag = False

        # 机器人实际坐标 x,y
        self.robot_x = self.ui.x_label
        self.robot_y = self.ui.y_label
        self.daq = DAQ(['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2'])
        cap = cv2.VideoCapture(0)
        # cap = openFlirCamera()
        time.sleep(1)
        # frame = cv2.imread("./image/333.png")
        self.frame_width, self.frame_height = 1080, 1080
        # frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        # contours = get_contours(frame)
        # self.robot = Robot(cap, frame_width=self.frame_width, frame_height=self.frame_height, img_frame=None, contours=contours)
        self.robot = Robot(cap, frame_width=self.frame_width, frame_height=self.frame_height, img_frame=None, contours=None)

        # PID 初始化
        ret, frame = cap.read()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BayerBG2BGR)  # for RGB camera demosaicing
        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        cap.release()
        try:
            self.pid = PID(frame=frame, x0=self.robot.get_robot_position()[0], y0=self.robot.get_robot_position()[1], contours=None)
        except Exception as e:
            print(f"PID------: {e}")

        # 给Start Stop个按钮绑定槽函数
        self.ui.StartButton.clicked.connect(self.startDaq)  # 绑定槽函数
        self.ui.StopButton.clicked.connect(self.stopDaq)  # 绑定槽函数
        self.ui.StartPIDButton.clicked.connect(self.startPID)
        self.ui.StopPIDButton.clicked.connect(self.stopPID)
        self.ui.show_planned_path_Button.clicked.connect(self.update_show_planned_path)
        self.ui.show_tracking_box_Button.clicked.connect(self.update_show_tracking_box)
        self.ui.show_tracking_path_Button.clicked.connect(self.update_show_tracking_path)

        self.f_edit.valueChanged.connect(self.f_changed)
        self.beta_edit.valueChanged.connect(self.beta_changed)
        self.B_edit.valueChanged.connect(self.B_changed)
        self.beta_dial_edit.valueChanged.connect(self.beta_dial_changed)
        self.alpha_edit.valueChanged.connect(self.alpha_changed)

        self.tracker = MouseKeyTracker(self.ImgShowLabel)
        self.tracker.positionChanged.connect(self.mousePositionChanged)
        self.tracker.upKeyPressed.connect(lambda: self.beta_edit.setValue(180))
        self.tracker.downKeyPressed.connect(lambda: self.beta_edit.setValue(0))
        self.tracker.leftKeyPressed.connect(lambda: self.beta_edit.setValue(90))
        self.tracker.rightKeyPressed.connect(lambda: self.beta_edit.setValue(270))
        self.tracker.spaceKeyPressed.connect(self.spaceKeyPressed)

        self.position_list = None
        self.videoThread = VideoThread(self.robot, frame_width=self.frame_width, frame_height=self.frame_height, img_frame=None, contours=None)
        self.videoThread.signal.connect(self.refreshShow)
        self.videoThread.start()

        self.printThread = TextBrowserThread()
        self.printThread.signal.connect(self.slot_text_browser)
        self.printThread.start()

        # PID 初始化
        # 创建一个定时器
        self.pidTimer = QTimer(self.ui)
        # 设置定时器的间隔时间（毫秒为单位）
        interval = 1000  # 1秒
        self.pidTimer.setInterval(interval)
        # 连接定时器的timeout信号到要执行的函数
        self.pidTimer.timeout.connect(self.update_pid)

        self.pid_t = 0
        self.show_planned_path = False
        self.position_list = self.pid.plot_list()

    def update_show_tracking_box(self):
        self.videoThread.show_tracking_robot = not self.videoThread.show_tracking_robot

    def update_show_tracking_path(self):
        self.videoThread.show_tracking_path = not self.videoThread.show_tracking_path

    def update_show_planned_path(self):
        self.show_planned_path = not self.show_planned_path

    def startDaq(self):
        self.daq.startDaq()
        self.daq.start()

    def stopDaq(self):
        self.daq.stopDaq()

    def startPID(self):
        print("START PID-----")
        try:
            self.pid.x0 = self.robot.get_robot_position()[0]
            self.pid.y0 = self.robot.get_robot_position()[1]
            self.pid_t = 0
            self.position_list = self.pid.plot_list()
        except Exception as e:
            print(f"PID: {e}")
        print("----")
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
        # self.pid.set_f(num)

    def beta_changed(self):
        num = self.beta_edit.value()
        print("beta:", num)
        self.daq.set_beta(num)
        self.beta_dial_edit.blockSignals(True)
        self.beta_dial_edit.setValue(num)
        self.beta_dial_edit.blockSignals(False)

    def alpha_changed(self):
        num = self.alpha_edit.value()
        print("alpha:", num)
        self.daq.set_alpha(num)

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

    def slot_text_browser(self):
        # text_browser槽函数
        # self.daq.set_beta(self.daq.get_beta() + 10)
        # self.textBrowser.clear()
        text = f'频率f：<font color="red">{self.daq.get_f()}</font> 角度beta: <font color="red">{self.daq.get_beta()}</font> 磁场强度B: <font color="red">{self.daq.get_B()}</font>'
        text_a = f'ao0: <font color="red">{self.daq.write_a0:.2f}</font> ao1: <font color="red">{self.daq.write_a1:.2f}</font> ao2: <font color="red">{self.daq.write_a2:.2f}</font>'
        self.textBrowser.append(text + '        ' + text_a)
        # self.update_second_beta()

    def refreshShow(self):
        # 将 OpenCV 图像转换为 Qt 图像
        # print(self.videoThread.frame.shape)
        try:
            if self.position_list and self.show_planned_path:
                for position in self.position_list:
                    cv2.circle(self.videoThread.frame, (position[0], 1080 - position[1]), 5, (0, 255, 0), -1)
        except Exception as e:
            print(f"plot: {e}")
        try:
            height, width, channel = self.videoThread.frame.shape
            # print(height + ">>" + width + ">>" + channel)
            bytes_per_line = 3 * width
            q_image = QImage(self.videoThread.frame.data, width, height, bytes_per_line, QImage.Format_BGR888)  # 设置颜色格式
            # # # 将 Qt 图像设置为 QLabel 的背景
            self.ImgShowLabel.setPixmap(QPixmap.fromImage(q_image))
        except Exception as e:
            print(f"refreshShow------: {e}")
        self.robot_x.setText(str(self.robot.get_robot_position()[0]))
        self.robot_y.setText(str(self.robot.get_robot_position()[1]))

    @QtCore.pyqtSlot(QtCore.QPoint)
    def mousePositionChanged(self, pos):
        delta = QtCore.QPoint(30, -15)
        self.label_position.show()
        self.label_position.move(pos + delta)
        self.label_position.setText("(%d, %d)" % (pos.x(), 1080 - pos.y()))
        self.label_position.adjustSize()

    @QtCore.pyqtSlot()
    def spaceKeyPressed(self):
        print("space key 被按下！重新选择你要追踪的框")
        # 关闭 pid 和 daq
        print("daq stop, pid stop")
        self.pidTimer.stop()
        self.daq.stopDaq()
        self.robot.init_tracker_bbox()
        # 启动 pid 和 daq
        self.pidTimer.start()
        self.daq.startDaq()
        self.daq.start()

    def update_pid(self):
        # self.pid.pidPosition(self.robot.get_robot_position(), self.pid_t)
        try:
            B, f, alpha, beta = self.pid.pidPosition(self.robot.get_robot_position())
            self.beta_dial_edit.blockSignals(True)
            self.beta_edit.blockSignals(True)
            self.beta_edit.setValue(beta)
            self.beta_dial_edit.setValue(beta)
            self.daq.set_beta(beta)
            self.beta_dial_edit.blockSignals(False)
            self.beta_edit.blockSignals(False)
            self.B_edit.setValue(B)
            self.f_edit.setValue(f)
            self.alpha_edit.setValue(alpha)
            self.pid_t += 1
            print("pid update ：", self.pid_t)
        except Exception as e:
            print(f"update_pid: {e}")

    def update_daq(self):
        self.daq.run_old()


if __name__ == "__main__":
    # QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    Window = MainWindow()
    Window.ui.show()
    app.exec()
