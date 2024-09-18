#!/usr/bin/env python3
import math
import numpy as np

from kalman_filter import KalmanFilter
from imm import Imm
import data
from plot import *


def save_to_file(filename, data, headers=None):
    if headers is not None:
        with open(filename, 'w') as f:
            f.write(','.join(headers) + '\n')
            np.savetxt(f, data, delimiter=',')
    else:
        np.savetxt(filename, data, delimiter=',')

def z_data(dt= 0.1):
    cnt = 100
    z_std = data.cv_z(0., 10., 0., 10., dt, cnt)
    z_std += data.ct_z(z_std[-1][0, 0], z_std[-1][1, 0],
                       z_std[-1][2, 0], z_std[-1][3, 0], math.pi / 180 * 25, dt, cnt)
    z_std += data.ca_z(z_std[-1][0, 0], z_std[-1][1, 0], 6.,
                       z_std[-1][2, 0], z_std[-1][3, 0], 8., dt, cnt)

    return z_std

def intersection_data(dt=0.1):
    cnt = 100  # 每个阶段的时间步数
    z_std = []

    # 1. 匀速直行段（以巡航速度行驶）
    # 初始位置 (x0, y0) = (0, 0)
    # 初始速度 (vx0, vy0) = (10 m/s, 0 m/s)
    z_std += data.cv_z(0., 10., 0., 0., dt, cnt)

    # 2. 减速接近交叉口
    # 减速度 ax = -1.5 m/s², ay = 0 m/s²
    last_state = z_std[-1]
    vx_cruise = last_state[1, 0]
    vy_cruise = last_state[3, 0]
    z_std += data.ca_z(last_state[0, 0], vx_cruise, -1.5,
                       last_state[2, 0], vy_cruise, 0., dt, cnt)

    # 3. 在交叉口转弯（协调转弯）
    # 以较低的速度进行转弯，转弯角速度 ω = 20°/s
    last_state = z_std[-1]
    z_std += data.ct_z(last_state[0, 0], last_state[1, 0],
                       last_state[2, 0], last_state[3, 0],
                       math.radians(20), dt, cnt)

    # 4. 转弯后加速离开
    # 加速度 ax = 1.5 m/s², ay = 0 m/s²（假设在新的方向上）
    last_state = z_std[-1]
    z_std += data.ca_z(last_state[0, 0], last_state[1, 0], 1.5,
                       last_state[2, 0], last_state[3, 0], 0., dt, cnt)

    return z_std



def test_intersection_data():
    z_std = intersection_data()
    z_noise = data.add_noise(z_std, np.array([
        [5.],
        [2],
        [5.],
        [2]
    ]))

    z_std = np.array(z_std)
    z_noise = np.array(z_noise)
    headers = ['x', 'vx', 'y', 'vy']
    save_to_file('../z_std_cvat_intersection.csv', z_std.reshape(z_noise.shape[0], -1), headers)
    save_to_file('../z_noise_cvat_intersection.csv', z_noise.reshape(z_noise.shape[0], -1), headers)

def test_cvat_data():
    z_std = z_data()
    z_noise = data.add_noise(z_std, np.array([
        [5.],
        [2],
        [5.],
        [2]
    ]))

    z_std = np.array(z_std)
    z_noise = np.array(z_noise)
    headers = ['x', 'vx', 'y', 'vy']
    save_to_file('../z_std_cvat.csv', z_std.reshape(z_noise.shape[0], -1), headers)
    save_to_file('../z_noise_cvat.csv', z_noise.reshape(z_noise.shape[0], -1), headers)


# test_cvat_data()
test_intersection_data()