#!/usr/bin/env python3
import math
import numpy as np
import pandas as pd

from kalman_filter import KalmanFilter
from imm import Imm
import data
from plot import *


dt = 0.1


def kf_cv_pos_only():
    A = np.array([
        [1., dt, 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 1., dt],
        [0., 0., 0., 1.]
    ])
    H = np.array([
        [1., 0., 0., 0.],
        [0., 0., 1., 0.]
    ])
    return KalmanFilter(A, H)


def kf_ca_pos_only():
    A = np.array([
        [1., dt, 0.5 * dt ** 2, 0., 0., 0.],
        [0., 1., dt, 0., 0., 0.],
        [0., 0., 1., 0., 0., 0.],
        [0., 0., 0., 1., dt, 0.5 * dt ** 2],
        [0., 0., 0., 0., 1., dt],
        [0., 0., 1., 0., 0., 1.]
    ])
    H = np.array([
        [1., 0., 0., 0., 0., 0.],
        [0., 0., 0., 1., 0., 0.]
    ])
    return KalmanFilter(A, H)


def kf_ct_pos_only():
    dtheta = math.pi / 180 * 15
    theta = dtheta * dt
    A = np.array([
        [1., math.sin(theta) / dtheta, 0., -(1 - math.cos(theta)) / dtheta, 0.],
        [0., math.cos(theta), 0., -math.sin(theta), 0.],
        [0., (1 - math.cos(theta)) / dtheta, 1., math.sin(theta) / dtheta, 0.],
        [0., math.sin(theta), 0., math.cos(theta), 0.],
        [0., 0., 0., 0., 1.],
    ])
    H = np.array([
        [1., 0., 0., 0., 0.],
        [0., 0., 1., 0., 0.]
    ])
    return KalmanFilter(A, H)


def imm_cvat():
    P_trans = np.array([
        [0.98, 0.01, 0.01],
        [0.01, 0.98, 0.01],
        [0.01, 0.01, 0.98]
    ])
    U_prob = np.array([0.8, 0.1, 0.1]).reshape((-1, 1))

    models = [kf_cv_pos_only(), kf_ca_pos_only(), kf_ct_pos_only()]
    r = np.array([
        [5.],
        # [2.],
        [5.],
        # [2.5]
    ])
    for model in models:
        model.R *= r

    T12 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, 0, 0, 0]
    ])

    T23 = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0]
    ])

    model_trans = [
        [np.eye(models[0].A.shape[0]), T12.T, np.dot(T12.T, T23.T)],
        [T12, np.eye(models[1].A.shape[0]), T23.T],
        [np.dot(T23, T12), T23, np.eye(models[2].A.shape[0])]
    ]

    return Imm(models, model_trans, P_trans, U_prob)


def test_cvat():
    z_std = [np.array(z).reshape(4, 1) for z in pd.read_csv('../z_std_cvat_intersection.csv').values]
    z_noise = [np.array(z).reshape(4, 1) for z in pd.read_csv('../z_noise_cvat_intersection.csv').values]

    imm = imm_cvat()
    z0 = z_noise[0]
    imm.models[0].X = np.array([
        [z0[0, 0]],
        [0.],
        [z0[2, 0]],
        [0.]
    ])
    imm.models[1].X = np.array([
        [z0[0, 0]],
        [0.],
        [0.],
        [z0[2, 0]],
        [0.],
        [0.]
    ])
    imm.models[2].X = np.array([
        [z0[0, 0]],
        [0.],
        [z0[2, 0]],
        [0.],
        [0.]
    ])

    prob = []
    z_filt = []
    for z in z_noise:
        z_pos = z[[0, 2],:]
        prob.append(np.copy(imm.filt(z_pos)))
        # merge
        x = np.zeros(imm.models[0].X.shape)
        for i in range(len(imm.models)):
            x += np.dot(imm.model_trans[0][i], imm.models[i].X) * prob[-1][i]
        z_filt.append(x)

    prob_array = np.hstack(prob)
    z_filt_array = np.hstack(z_filt)
    z_filt_df = pd.DataFrame(z_filt_array.T, columns=['x', 'vx', 'y', 'vy'])
    prob_df = pd.DataFrame(prob_array.T, columns=['cv', 'ca', 'ct'])
    z_filt_df.to_csv('../z_filt_cvat_py.csv', index=False)
    prob_df.to_csv('../prob_cvat_py.csv', index=False)

    plot_position(
        [z[0, 0] for z in z_std],
        [z[2, 0] for z in z_std],
        [z[0, 0] for z in z_noise],
        [z[2, 0] for z in z_noise],
        [z[0, 0] for z in z_filt],
        [z[2, 0] for z in z_filt]
    )
    plot_speed(
        [z[1, 0] for z in z_std],
        [z[3, 0] for z in z_std],
        [z[1, 0] for z in z_noise],
        [z[3, 0] for z in z_noise],
        [z[1, 0] for z in z_filt],
        [z[3, 0] for z in z_filt]
    )
    plot_prob(
        [p[0, 0] for p in prob],
        [p[1, 0] for p in prob],
        [p[2, 0] for p in prob],
    )
    plot_show()


test_cvat()
