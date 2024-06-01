import sys
import time
from common import DAQ
from PyQt5 import uic
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from PyQt5.Qt import QApplication, QWidget, QThread
import os


class MyThread(QThread):
    # 实时显示追加线程（要继承QThread， 继承threading.Thread不行）
    signal = pyqtSignal(str)  # 信号

    def __init__(self, daq):
        super().__init__()
        self.daq = daq

    def run(self):
        for i in range(1000):
            text = f'频率f：<font color="red">{self.daq.get_f()}</font> beta: <font color="red">{self.daq.get_beta()}</font> ' \
                   f'alpha: <font color="red">{self.daq.get_alpha()}</font> B: <font color="red">{self.daq.get_B()}</font>'
            self.signal.emit(text)  # 发射信号(实参类型要和定义信号的参数类型一致)
            time.sleep(1)
            # # 4.清空文本
            # # self.textbrower.clear()
            # text = "频率f：%d \n beta: %d" % self.daq.get_f(), self.daq.get_beta()
            # self.textbrower.append("hhhh")
            # time.sleep(1)


class MyWin(QWidget):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("../QtUI/daq.ui")

        # 从ui文件中加载控件
        # f, beta, B edit初始化
        self.f_edit = self.ui.F_spinBox
        self.beta_edit = self.ui.beta_spinBox
        self.B_edit = self.ui.B_spinBox
        self.alpha_edit = self.ui.alpha_spinBox
        self.beta_dial_edit = self.ui.beta_dial

        self.start = self.ui.StartButton
        self.stop = self.ui.StopButton
        self.textBrowser = self.ui.daq_textBrowser

        # 给2个按钮绑定槽函数
        self.start.clicked.connect(self.startDaq)  # 绑定槽函数
        self.stop.clicked.connect(self.stopDaq)  # 绑定槽函数

        self.f_edit.valueChanged.connect(self.f_changed)
        self.beta_edit.valueChanged.connect(self.beta_changed)
        self.B_edit.valueChanged.connect(self.B_changed)
        self.beta_dial_edit.valueChanged.connect(self.beta_dial_changed)
        self.alpha_edit.valueChanged.connect(self.alpha_changed)

        self.daq = DAQ(['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2'])

        # self.textBrowser.setText("fffffff")
        self.printThread = MyThread(self.daq)
        self.printThread.signal.connect(self.slot_text_browser)
        self.printThread.start()

        self.f_edit.setValue(20)
        self.beta_edit.setValue(90)
        self.B_edit.setValue(5)
        self.alpha_edit.setValue(90)
        self.beta_dial_edit.setValue(0)

    # def click_2(self):
    #     self.my_thread = MyThread()  # 创建线程
    #     self.my_thread.start()  # 开始线程

    def startDaq(self):
        self.daq.startDaq()
        self.daq.set_B(self.B_edit.value())
        self.daq.start()

    def stopDaq(self):
        self.daq.stopDaq()

    def f_changed(self):
        num = self.f_edit.value()
        print("f:", num)
        self.daq.set_f(num)

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

    def slot_text_browser(self, text):
        # text_browser槽函数
        # self.daq.set_beta(self.daq.get_beta() + 10)
        self.textBrowser.append(text)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myshow = MyWin()
    myshow.ui.show()
    app.exec()
