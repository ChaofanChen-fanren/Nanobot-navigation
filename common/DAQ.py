import nidaqmx
from nidaqmx.stream_writers import AnalogMultiChannelWriter
import math
import numpy as np
import time
from PyQt5.Qt import QThread
# from nidaqmx.constants import AcquisitionType, RegenerationMode
# import threading


class DAQ(QThread):
    def __init__(self, physical_channels_list=['Dev1/ao0', 'Dev1/ao1', 'Dev1/ao2']):
        super().__init__()
        # 设置ao0、ao1、ao2通道
        self.ao0 = physical_channels_list[0]
        self.ao1 = physical_channels_list[1]
        self.ao2 = physical_channels_list[2]

        # beta角
        self.beta = 0
        # alpha角
        self.alpha = 90
        # 磁场强度B
        self.B = 0
        # 偏差值 m
        self.m = -math.pi/2
        # 频率f
        self.f = 10

        self.write_a0 = 0
        self.write_a1 = 0
        self.write_a2 = 0

        self.flag = True

        self.t = 0

    def cal_write(self):
        # 角度转化为弧度制
        beta_rad = math.radians(self.beta)
        alpha_rad = math.radians(self.alpha)
        # (B/26)*sin(2*pi*f)*(-1)*cos(alpha_rad)
        left = (self.B / 26) * math.sin(2 * math.pi * self.f * self.t) * (-1) * math.cos(alpha_rad)
        # (B/26)*sin(2*pi*f + m)
        right = (self.B / 26) * math.sin(2 * math.pi * self.f * self.t + self.m)
        self.write_a0 = (left * math.cos(beta_rad) + right * math.sin(beta_rad)) * 43 / 10.81
        self.write_a1 = (left * math.sin(beta_rad) - right * math.cos(beta_rad)) * 27.9 / 8.26
        # B/26*sin(2*pi*f)*sin(alpha_rad)*30*12
        self.write_a2 = (self.B / 26) * math.sin(2 * math.pi * self.f * self.t) * math.sin(alpha_rad) * 29.4 / 19.42

    def run_old(self) -> None:
        self.t = 0
        try:
            self.t += 1e-3
            # print(f"run>>>> {self.t}")
        except Exception as e:
            print(f"发生了其他错误: {e}")

    def run(self):
        # for t in range(10):
        #     self.cal_write(t)
        #     print(self.beta)
        #     # print([self.write_a0, self.write_a1, self.write_a2])
        #     time.sleep(1)
        try:
            # 创建DAQmx任务
            with nidaqmx.Task() as task:
                # 为每个物理通道创建虚拟通道
                task.ao_channels.add_ao_voltage_chan(self.ao0)
                task.ao_channels.add_ao_voltage_chan(self.ao1)
                task.ao_channels.add_ao_voltage_chan(self.ao2)
                # task.timing.cfg_samp_clk_timing(rate=20000, sample_mode=AcquisitionType.CONTINUOUS)
                # 开始任务
                task.start()

                self.t = 0
                # 循环运行，直到某个条件被满足（需要你根据VI的逻辑填写）
                while self.flag:
                    # Generate the sine wave as the data to write
                    self.cal_write()
                    data_to_write = np.array([self.write_a0, self.write_a1, self.write_a2])

                    # Write the sine wave data to the analog output channels
                    writer = AnalogMultiChannelWriter(task.out_stream)
                    writer.write_one_sample(data_to_write)

                    # Add a delay to control the rate of data generation
                    time.sleep(1e-3)  # Adjust the delay as needed
                    self.t += 1e-3
        except nidaqmx.DaqError as e:
            print(f"发生了DAQmx错误: {e}")
        except Exception as e:
            print(f"发生了其他错误: {e}")
        print("daq ----停止-----")

    # Getter和Setter方法 for beta
    def get_beta(self):
        return self.beta

    def set_beta(self, value):
        self.beta = value

    # Getter和Setter方法 for alpha
    def get_alpha(self):
        return self.alpha

    def set_alpha(self, value):
        self.alpha = value

    # Getter和Setter方法 for B
    def get_B(self):
        return self.B

    def set_B(self, value):
        self.B = value

    # Getter和Setter方法 for m
    def get_m(self):
        return self.m

    def set_m(self, value):
        self.m = value

    # Getter和Setter方法 for f
    def get_f(self):
        return self.f

    def set_f(self, value):
        self.f = value

    def get_t(self):
        return self.t

    def stopDaq(self):
        self.B = 0
        self.flag = False

    def startDaq(self):
        self.B = 20
        self.flag = True

