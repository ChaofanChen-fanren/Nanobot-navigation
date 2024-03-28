import time
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from math import e
from scipy.interpolate import make_interp_spline as spline


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.kp = P
        self.ki = I
        self.kd = D
        self.uPrevious = 0
        self.uCurent = 0
        self.setValue = 0
        self.lastErr = 0
        self.preLastErr = 0
        self.errSum = 0
        self.errSumLimit = 10

    # 位置式PID
    def pidPosition(self, curValue):
        err = self.setValue - curValue
        dErr = err - self.lastErr
        self.preLastErr = self.lastErr
        self.lastErr = err
        self.errSum += err
        outPID = self.kp * err + (self.ki * self.errSum) + (self.kd * dErr)
        return outPID

    # 增量式PID
    def pidIncrease(self, curValue):
        self.uCurent = self.pidPosition(curValue)
        outPID = self.uCurent - self.uPrevious
        self.uPrevious = self.uCurent
        return outPID


class BeControlled:
    def __init__(self):
        self.lastControlIn = 0
        self.preLastControlIn = 0
        self.lastControlOut = 0
        self.preLastControlOut = 0

    # 被控对象的相关计算
    def beControlledDeal(self, outPID):
        # output = 2*self.lastControlOut - 1*self.preLastControlOut + \
        #     0.00005*self.lastControlIn + 0.00005*self.preLastControlIn

        # output为被控对象的输出，此处是被控对象的传递函数离散化后，写成差分方程后的形式，被控对象的方程此处直接采用了设计好的参数，并与PID控制器的输出进行计算。
        # 如果需要设计自己的被控对象，将传递函数进行z变换后，交叉相乘，再进行z反变换即可，可参考《计算机控制系统》等书籍。
        # 因为是单位反馈，所以被控对象的输出等于传递函数的输入。
        output = 0.00019346 * self.preLastControlIn + 0.00019671e-04 * self.lastControlIn + \
                 1.9512 * self.lastControlOut - 0.9512 * self.preLastControlOut
        self.preLastControlIn = self.lastControlIn
        self.lastControlIn = outPID
        self.preLastControlOut = self.lastControlOut
        self.lastControlOut = output
        return output


def testPid(P=0.2, I=0.0, D=0.0, Len=1000):
    pid = PID(P, I, D)
    beControlled = BeControlled()
    pid.setValue = 1  # set end
    curValue = 0
    curValueList = []
    timeList = []
    setValueList = []
    PIDoutList = []
    curValueList.append(0)
    timeList.append(0)
    setValueList.append(pid.setValue)
    PIDoutList.append(0)

    for i in range(1, Len):
        # 采用位置式PID去掉注释即可
        # outPID = pid.pidPosition(curValue)
        outPID = pid.pidIncrease(curValue)
        PIDoutList.append(outPID)
        curValue = beControlled.beControlledDeal(outPID)
        curValueList.append(curValue)
        setValueList.append(pid.setValue)
        timeList.append(i)

    # 绘图
    timeSm = np.array(timeList)
    timeSmooth = np.linspace(timeSm.min(), timeSm.max(), 300)  # 将x轴300等分
    curValueSmooth = spline(timeList, curValueList)(timeSmooth)  # 插值.使原y轴数据平滑
    pidoutSmooth = spline(timeList, PIDoutList)(timeSmooth)  # 使PID控制器输出平滑
    plt.figure(0)
    # plt.xticks([-1, 3, 5])
    # plt.yticks([-1, 100, 10])
    plt.plot(timeSmooth, curValueSmooth)  # 画被控对象输出
    plt.plot(timeSmooth, pidoutSmooth)  # 画PID控制器输出
    plt.plot(timeList, setValueList)  # 画直线
    plt.xlim((0, Len))
    plt.ylim((min(curValueList) - 0.5, max(curValueList) + 0.5))
    plt.xlabel('time (s)')
    plt.ylabel('set value')
    plt.title('PID')
    plt.ylim((1 - 0.5, 1 + 0.5))
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    testPid(P=0, I=0.5, D=0, Len=5000)
