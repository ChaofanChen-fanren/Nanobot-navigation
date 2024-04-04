import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
from random import randrange
import math

def fx(t, alpha, beta, f, B):
    m = -math.pi / 2
    beta_rad = math.radians(beta)
    alpha_rad = math.radians(alpha)
    # (B/26)*sin(2*pi*f)*(-1)*cos(alpha_rad)
    left = B * math.sin(2 * math.pi * f * t) * (-1) * math.cos(alpha_rad)
    # (B/26)*sin(2*pi*f + m)
    right = B * math.sin(2 * math.pi * f * t + m)
    a0 = (left * math.cos(beta_rad) + right * math.sin(beta_rad)) * 43 / 10.81 / 10
    a1 = (left * math.sin(beta_rad) - right * math.cos(beta_rad)) * 27.9 / 8.26 / 10
    # B/26*sin(2*pi*f)*sin(alpha_rad)*30*12
    a2 = B * math.sin(2 * math.pi * f * t) * math.sin(alpha_rad) * 29.4 / 19.42 / 10
    return a0, a1, a2


def get_a0a1a2(t_list, alpha, beta, f, B):
    # 角度转化为弧度制
    a0_list, a1_list, a2_list = [], [], []
    for it in t_list:
        a0, a1, a2 = fx(it, alpha=alpha, beta=beta, f=f, B=B)
        a0_list.append(a0)
        a1_list.append(a1)
        a2_list.append(a2)

    return a0_list, a1_list, a2_list


fig, ax = plt.subplots()
t = np.linspace(0, 2, 2000)

alpha, beta, f, B = 90, 90, 10, 25
y0, y1, y2 = get_a0a1a2(t, alpha=alpha, beta=beta, f=f, B=B)
(p0,) = ax.plot(t, y0, label="a0")
(p1,) = ax.plot(t, y1, label="a1")
(p2,) = ax.plot(t, y2, label="a2")
#显示图像的label
plt.grid(True)
plt.legend()
# (p,) = ax.plot(data_1)
#
plt.subplots_adjust(left=0.1, bottom=0.25)
axfreq = plt.axes([0.1, 0.15, 0.8, 0.05])
slider_B = Slider(
    ax=axfreq,
    label="B",
    valmin=0,
    valmax=100,
    valinit=10,
)
axfreq = plt.axes([0.1, 0.1, 0.8, 0.05])
slider_f = Slider(
    ax=axfreq,
    label="f",
    valmin=0,
    valmax=30,
    valinit=0,
)
axfreq = plt.axes([0.1, 0, 0.8, 0.05])
slider_beta = Slider(
    ax=axfreq,
    label="beta",
    valmin=0,
    valmax=360,
    valinit=0,
)

def update_B(val):
    global B
    B = val
    y0, y1, y2 = get_a0a1a2(t, alpha=alpha, beta=beta, f=f, B=B)
    p0.set_ydata(y0)
    p1.set_ydata(y1)
    p2.set_ydata(y2)

def update_beta(val):
    global beta
    beta = val
    y0, y1, y2 = get_a0a1a2(t, alpha=alpha, beta=beta, f=f, B=B)
    p0.set_ydata(y0)
    p1.set_ydata(y1)
    p2.set_ydata(y2)

def update_f(val):
    global f
    f = val
    y0, y1, y2 = get_a0a1a2(t, alpha=alpha, beta=beta, f=f, B=B)
    p0.set_ydata(y0)
    p1.set_ydata(y1)
    p2.set_ydata(y2)


slider_B.on_changed(update_B)
slider_beta.on_changed(update_beta)
slider_f.on_changed(update_f)

plt.show()