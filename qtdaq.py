import sys
import time
from DAQ import DAQ
from PyQt5 import uic
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from PyQt5.Qt import QApplication, QWidget, QThread


class MyThread(QThread):
    # 实时显示追加线程（要继承QThread， 继承threading.Thread不行）
    signal = pyqtSignal(str)  # 信号

    def __init__(self, daq):
        super().__init__()
        self.daq = daq

    def run(self):
        for i in range(1000):
            text = f'频率f：{self.daq.get_f()} beta: {self.daq.get_beta()}'
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
        self.ui = uic.loadUi("./daq.ui")

        # 从ui文件中加载控件
        self.f_edit = self.ui.F_Edit
        self.beta_edit = self.ui.beta_Edit
        self.start = self.ui.StartButton
        self.stop = self.ui.StopButton
        self.textBrowser = self.ui.daq_textBrowser

        # 给2个按钮绑定槽函数
        self.start.clicked.connect(self.startDaq)  # 绑定槽函数
        self.stop.clicked.connect(self.stopDaq)  # 绑定槽函数
        self.f_edit.editingFinished.connect(self.f_changed)
        self.beta_edit.editingFinished.connect(self.beta_changed)

        self.daq = DAQ(['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2'])

        # self.textBrowser.setText("fffffff")

        self.printThread = MyThread(self.daq)
        self.printThread.signal.connect(self.slot_text_browser)
        self.printThread.start()

    # def click_2(self):
    #     self.my_thread = MyThread()  # 创建线程
    #     self.my_thread.start()  # 开始线程

    def startDaq(self):
        self.daq.startDaq()
        self.daq.start()

    def stopDaq(self):
        self.daq.stopDaq()

    def f_changed(self):
        num = int(self.f_edit.text())
        print(num)
        self.daq.set_f(num)

    def beta_changed(self):
        num = int(self.beta_edit.text())
        print(num)
        self.daq.set_beta(num)

    def slot_text_browser(self, text):
        # text_browser槽函数
        # self.daq.set_beta(self.daq.get_beta() + 10)
        self.textBrowser.append(text)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myshow = MyWin()
    myshow.ui.show()
    app.exec()
