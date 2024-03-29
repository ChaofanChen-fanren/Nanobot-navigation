import math


class PID:
    def __init__(self,
                 x0, y0,
                 p1=0.1,  i1=0.001,  d1=0.05,
                 p2=0.1,  i2=0.001,  d2=0.05,
                 f=10):
        # 设置PID参数
        self.k_p1 = p1
        self.k_p2 = p2
        self.k_i1 = i1
        self.k_i2 = i2
        self.k_d1 = d1
        self.k_d2 = d2

        # 机器人的初始坐标
        self.x0 = x0
        self.y0 = y0

        # 设置频率
        self.f = f

        self.uPrevious = 0
        self.uCurent = 0
        self.setValue_x = 0
        self.setValue_y = 0
        self.lastErr_x = 0
        self.lastErr_y = 0
        self.errSum_x = 0
        self.errSum_y = 0
        self.errSumLimit_x = 1000
        self.errSumLimit_y = 1000

    def GetCalcuValue(self, t):
        x = self.x0 + t
        y = self.y0 + 200*math.sin(t/50)
        return x, y

    # 限幅函数
    def limitIntegralTerm(self, term, limit):
        if term > limit:
            return limit
        elif term < -limit:
            return -limit
        else:
            return term

    # 位置式PID
    def pidPosition(self, robot_position, t):
        print(f"X:{robot_position[0]} , Y: {robot_position[1]} , T:{t:.2f}\n")
        self.setValue_x, self.setValue_y = self.GetCalcuValue(t)
        # P
        err_x = self.setValue_x - robot_position[0]
        err_y = self.setValue_y - robot_position[1]
        # I
        self.errSum_x += err_x
        self.errSum_y += err_y
        # D
        dErr_x = err_x - self.lastErr_x
        dErr_y = err_y - self.lastErr_y

        self.lastErr_x = err_x
        self.lastErr_y = err_y

        self.errSum_x = self.limitIntegralTerm(self.errSum_x, self.errSumLimit_x)
        self.errSum_y = self.limitIntegralTerm(self.errSum_y, self.errSumLimit_y)

        PID_X = self.k_p1 * err_x + (self.k_i1 * self.errSum_x) + (self.k_d1 * dErr_x)
        PID_Y = self.k_p2 * err_y + (self.k_i2 * self.errSum_y) + (self.k_d2 * dErr_y)

        beta = self.cal_beta(PID_X, PID_Y)
        B = self.cal_B(PID_X, PID_Y, t)

        return beta, B

    def plot_list(self):
        position_list = list()
        for t in range(200):
            x, y = self.GetCalcuValue(t)
            position_list.append((int(x), int(y)))
        return position_list


    def cal_beta(self, dx, dy):
        if dx != 0:
            alpha = math.atan(dy / dx)
            beta = int(270 - math.degrees(alpha)) if dx > 0 else int(90 - math.degrees(alpha))
        else:
            beta = 0 if dy < 0 else 180
        return beta

    def cal_B(self, dx, dy, t):
        B = math.sqrt(dx * dx + dy * dy) / math.cos(2 * math.pi * self.f * t)
        return B

    def set_f(self, value):
        self.f = value

    # 增量式PID
    def pidIncrease(self, curValue):
        self.uCurent = self.pidPosition(curValue)
        outPID = self.uCurent - self.uPrevious
        self.uPrevious = self.uCurent
        return outPID