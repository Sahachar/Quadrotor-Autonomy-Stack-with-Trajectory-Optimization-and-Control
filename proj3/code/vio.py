#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    new_p = np.zeros((3, 1))
    new_v = np.zeros((3, 1))
    new_q = Rotation.identity()

    R = Rotation.as_matrix(q)
    new_p = p + (v*dt) + (0.5*dt*dt*((R@(a_m - a_b))+g))
    new_v = v + (dt * ((R @ (a_m - a_b)) + g))
    vec = (w_m - w_b)*dt
    R_ = Rotation.as_matrix(Rotation.from_rotvec(vec.flatten()))
    new_q = Rotation.from_matrix(R @ R_)

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    R = Rotation.as_matrix(q)
    vec = (w_m - w_b) * dt
    R_ = Rotation.as_matrix(Rotation.from_rotvec(vec.flatten()))
    a = (a_m-a_b)

    a_hat = np.array([[0, -a[-1][0], a[1][0]],
                      [a[-1][0], 0, -a[0][0]],
                      [-a[1][0], a[0][0], 0]])
    O3 = np.zeros((3, 3))
    I3 = np.identity(3)
    V_i = ((accelerometer_noise_density*dt)**2)*I3
    Th_i = ((gyroscope_noise_density*dt)**2)*I3
    A_i = ((accelerometer_random_walk**2)*dt)*I3
    Ph_i = ((gyroscope_random_walk**2)*dt)*I3

    Q_i = np.vstack((np.hstack((V_i, O3, O3, O3)),
                     np.hstack((O3, Th_i, O3, O3)),
                     np.hstack((O3, O3, A_i, O3)),
                     np.hstack((O3, O3, O3, Ph_i))))

    F_i = np.vstack((np.hstack((O3, O3, O3, O3)),
                     np.hstack((I3, O3, O3, O3)),
                     np.hstack((O3, I3, O3, O3)),
                     np.hstack((O3, O3, I3, O3)),
                     np.hstack((O3, O3, O3, I3)),
                     np.hstack((O3, O3, O3, O3))))
    # print(-dt*(R@a_hat))

    F_x = np.vstack((np.hstack((I3, dt*I3, O3, O3, O3, O3)),
                     np.hstack((O3, I3, -dt*(R@a_hat), -dt*R, O3, dt*I3)),
                     np.hstack((O3, O3, R_.T, O3, -dt*I3, O3)),
                     np.hstack((O3, O3, O3, I3, O3, O3)),
                     np.hstack((O3, O3, O3, O3, I3, O3)),
                     np.hstack((O3, O3, O3, O3, O3, I3))))
    # print("********************F_x**************", "\n", F_x)
    error_state_covariance_update = (F_x @ (error_state_covariance @ (F_x.T))) + (F_i @ (Q_i @ (F_i.T)))

    # return an 18x18 covariance matrix
    return error_state_covariance_update


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # Compute the innovation next state, next error_state covariance
    innovation = np.zeros((2, 1))
    R = Rotation.as_matrix(q)
    Pc = (R.T) @ (Pw-p)
    Xc, Yc, Zc = Pc[0], Pc[1], Pc[2]
    Pc_n = (Pc/Zc)
    zt = Pc_n[0:2]
    # print(zt)
    innovation = uv - zt
    if np.linalg.norm(innovation) > error_threshold:
        return (p, v, q, a_b, w_b, g), error_state_covariance, innovation

    dzt_dPc = (1/Zc)*np.array([[1, 0, -zt[0][0]],
                        [0, 1, -zt[1][0]]])
    dPc_ddth = np.array([[0, -Zc[0], Yc[0]],
                        [Zc[0], 0, -Xc[0]],
                        [-Yc[0], Xc[0], 0]])
    dPc_ddp = -R.T
    dzt_ddth = dzt_dPc @ dPc_ddth
    dzt_ddp = dzt_dPc @ dPc_ddp

    O23 = np.zeros((2, 3))
    H = np.hstack((dzt_ddp, O23, dzt_ddth, O23, O23, O23))
    # print(dzt_dPc)
    # print(Q.shape)

    cov = error_state_covariance
    # print(H)
    # print((H @ (cov @ (H.T))))
    # print("********************H**************", "\n", dzt_dPc, "\n", dPc_ddth)
    K = (cov @ H.T) @ np.linalg.inv((H @ (cov @ (H.T))) + Q)


    dx = K @ innovation
    R_dx = Rotation.as_matrix(Rotation.from_rotvec(dx[6:9].flatten()))
    p += dx[:3]
    v += dx[3:6]
    q = Rotation.from_matrix(Rotation.as_matrix(q) @ R_dx)
    a_b += dx[9:12]
    w_b += dx[12:15]
    g += dx[15:18]
    # print(innovation)
    error_state_covariance = ((np.identity((K @ H).shape[0]) - K @ H)
                              @ cov @ ((np.identity((K @ H).shape[0]) - K @ H).T)) + K @ (Q @ (K.T))


    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation
