import numpy as np
import math
from squaternion import *

GRAVITY_G = 9.8066


def rfTransform(data, cali=True):
    """
    :param data:
    :return: [[ax,ay,az], [], ...]
    """
    acce = []
    gyro = []
    q, ori = attitude(data, cali)
    dataLen = min(len(data[0]), len(data[3]), len(data[6]), len(data[9]))
    for i in range(dataLen):
        tmpAcce = np.array([[data[0][i]], [data[1][i]], [data[2][i]]])
        tmpGyro = np.array([[data[3][i]], [data[4][i]], [data[5][i]]])
        acce.append(frameTransform(q[i], tmpAcce))
        gyro.append(frameTransform(q[i], tmpGyro))
    return acce, gyro, ori


def attitude(data, cali=True):
    # yields an equal length input
    resOrien = []

    res_qt = []
    northWinG = []  # G:gravity, north direction from mag
    northWinC = []  # C:accumulation,  north direction from gyro

    graWinG = []  # gravity direction from gravity
    graWinC = []  # gravity direction from gyro

    data = matchData(data)
    dataLen = min(len(data[0]), len(data[3]), len(data[6]), len(data[9]))

    delta_t = 0  # time interval for opportunistic calibration
    # prev_timestamp = 0  # used for getting time interval
    h = 1 / 50  # sample interval
    qt = np.array([[0, 0, 0, 1]])  # initial quaternion for differential equation

    Eg = 0  # error of gravity
    Ec = 0  # error of accumulation

    for i in range(dataLen):
        tmpA = [data[0][i], data[1][i], data[2][i]]
        tmpG = [data[6][i], data[7][i], data[8][i]]
        tmpW = [data[3][i], data[4][i], data[5][i]]
        tmpM = [data[9][i], data[10][i], data[11][i]]

        yawG, pitchG, rollG = get_orientation(tmpM, tmpG)
        northWinG.append(yawG)
        graWinG.append(pitchG)

        if i == 0:
            qt = list(Quaternion.from_euler(rollG, pitchG, yawG))
            qt = np.array([[qt[1], qt[2], qt[3], qt[0]]])

            northWinC.append(yawG)
            graWinC.append(pitchG)
        else:
            qt, r_t = get_rotation_euler_axis_angle(qt, tmpW, h)
            tmpOri = get_orientation_from_rotation(r_t)
            resOrien.append(tmpOri)
            northWinC.append(tmpOri[0])
            graWinC.append(tmpOri[1])
            w_val = np.linalg.norm(tmpW, 2)
            a_val = np.linalg.norm(tmpA, 2)
            if w_val >= np.pi * 2 / 3 or a_val >= 2 * GRAVITY_G:
                Ec = np.Inf
            else:
                Ec += 0.0003 * w_val * h + 0.001 * a_val * h
            delta_t += h

        while len(northWinG) > 100:
            northWinG.pop(0)

        while len(northWinC) > 100:
            northWinC.pop(0)

        while len(graWinG) > 100:
            graWinG.pop(0)

        while len(graWinC) > 100:
            graWinC.pop(0)

        # opportunistic calibration
        if delta_t >= 2:
            # calculate time series similarity
            p_c = 1 / (2 ** np.var([northWinG[i] - northWinC[i] for i in range(len(northWinC))]))
            p_g = 1 / (2 ** np.var([graWinG[i] - graWinC[i] for i in range(len(graWinC))]))
            if cali:
                if p_c > 0.2 and p_g > 0.2:
                    E_1 = -32.14 * p_c + 19.93
                    E_2 = -12.86 * p_g + 11.57
                    Eg = max(E_1, E_2)
                    yawG, pitchG, rollG = get_orientation(tmpM, tmpG)
                    qg = list(Quaternion.from_euler(rollG, pitchG, yawG))
                    qg = np.array([[qg[1], qg[2], qg[3], qg[0]]])

                    if Eg < Ec:
                        qt = qg
                        Ec = 0
                        delta_t = 0
        res_qt.append(qTrans(qt))
    return res_qt, resOrien


def get_orientation(mag, gravity):
    Ax = gravity[0]
    Ay = gravity[1]
    Az = gravity[2]
    normsqA = Ax ** 2 + Ay ** 2 + Az ** 2
    g_threshold = 0.01 * GRAVITY_G ** 2
    if normsqA < g_threshold:
        return mag
    Ex = mag[0]
    Ey = mag[1]
    Ez = mag[2]
    #
    Hx = Ey * Az - Ez * Ay
    Hy = Ez * Ax - Ex * Az
    Hz = Ex * Ay - Ey * Ax
    normH = np.sqrt(Hx ** 2 + Hy ** 2 + Hz ** 2)
    if normH < 0.1:
        return mag
    invH = 1.0 / normH
    Hx *= invH
    Hy *= invH
    Hz *= invH
    invA = 1.0 / np.sqrt(Ax ** 2 + Ay ** 2 + Az ** 2)
    Ax *= invA
    Ay *= invA
    Az *= invA
    Mx = Ay * Hz - Az * Hy
    My = Az * Hx - Ax * Hz
    Mz = Ax * Hy - Ay * Hx
    R0 = Hx
    R1 = Hy
    R2 = Hz
    R3 = Mx
    R4 = My
    R5 = Mz
    R6 = Ax
    R7 = Ay
    R8 = Az

    orientation = [0, 0, 0]
    orientation[0] = np.arctan2(R1, R4)
    orientation[1] = np.arcsin(-R7)
    orientation[2] = np.arctan2(-R6, R8)
    return orientation


def get_orientation_from_rotation(rotation):
    orientation = [0, 0, 0]
    orientation[0] = np.arctan2(rotation.item((0, 1)), rotation.item((1, 1)))
    orientation[1] = np.arcsin(-rotation.item((2, 1)))
    orientation[2] = np.arctan2(-rotation.item((2, 0)), rotation.item((2, 2)))
    return orientation


def get_rotation_compass_gravity(gravity, theta):
    """
    :param gravity: gravity measurements from the phone
    :param theta: compass reading
    :return: R_c
    """
    g_norm = np.linalg.norm(gravity, 2)
    a = gravity[0] / g_norm
    b = gravity[1] / g_norm
    c = gravity[2] / g_norm
    K = ((a * np.cos(theta) - a * b) * (a * np.cos(theta) - b * np.sin(theta))) ** 2 / (
            a ** 2 * (a * np.cos(theta) - b * np.sin(theta)) ** 2) \
        + ((b ** 2 - c * np.cos(theta)) * (a * np.cos(theta) - a * b)) ** 2 \
        + ((a * np.cos(theta) - a * b) * (a * np.cos(theta) - b * np.sin(theta))) ** 2
    R = np.matrix(
        [[(b ** 2 - c * np.cos(theta)) / (a * np.cos(theta) - b * np.sin(theta)) * np.sqrt(K), np.sin(theta), a],
         [a * c / (a * np.cos(theta) - a * b) * np.sqrt(K), np.cos(theta), b],
         [np.sqrt(K), b, c]])
    return R


def get_rotation_euler_axis_angle(q_t, w_t, h):
    Q_t = get_matrix_Q_t(q_t)
    dq_dt = 0.5 * np.dot(Q_t, w_t)
    # fourth-order Runge-Kutte approximation
    k1 = dq_dt
    k2 = 0.5 * np.dot(get_matrix_Q_t(q_t + 0.5 * k1 * h), w_t)
    k3 = 0.5 * np.dot(get_matrix_Q_t(q_t + 0.5 * k2 * h), w_t)
    k4 = 0.5 * np.dot(get_matrix_Q_t(q_t + k3 * h), w_t)
    q_t_plus_1 = q_t + float(h) / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    R_t_plus_1 = get_matrix_R_t(q_t_plus_1)
    return q_t_plus_1, R_t_plus_1


def get_matrix_Q_t(q_t):
    return np.matrix([[q_t.item((0, 3)), -q_t.item((0, 2)), q_t.item((0, 1))],
                      [q_t.item((0, 2)), q_t.item((0, 3)), -q_t.item((0, 0))],
                      [-q_t.item((0, 1)), q_t.item((0, 0)), q_t.item((0, 3))],
                      [-q_t.item((0, 0)), -q_t.item((0, 1)), -q_t.item((0, 2))]])


def get_matrix_R_t(q_t):
    return np.matrix([[q_t.item((0, 0)) ** 2 - q_t.item((0, 1)) ** 2 - q_t.item((0, 2)) ** 2 + q_t.item((0, 3)) ** 2,
                       2 * (q_t.item((0, 0)) * q_t.item((0, 1)) - q_t.item((0, 3)) * q_t.item((0, 2))),
                       2 * (q_t.item((0, 0)) * q_t.item((0, 2)) + q_t.item((0, 3)) * q_t.item((0, 1)))],
                      [2 * (q_t.item((0, 0)) * q_t.item((0, 1)) + q_t.item((0, 3)) * q_t.item((0, 2))),
                       -q_t.item((0, 0)) ** 2 + q_t.item((0, 1)) ** 2 - q_t.item((0, 2)) ** 2 + q_t.item((0, 3)) ** 2,
                       2 * (q_t.item((0, 1)) * q_t.item((0, 2)) - q_t.item((0, 3)) * q_t.item((0, 0)))],
                      [2 * (q_t.item((0, 2)) * q_t.item((0, 0)) - q_t.item((0, 3)) * q_t.item((0, 1))),
                       2 * (q_t.item((0, 2)) * q_t.item((0, 1)) + q_t.item((0, 3)) * q_t.item((0, 0))),
                       2 * (q_t.item((0, 0)) ** 2 - q_t.item((0, 1)) ** 2 + q_t.item((0, 2)) ** 2 + q_t.item(
                           (0, 3)) ** 2)]])


# qvq*
def frameTransform(q, v):
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]
    r = np.array([[1 - 2 * c ** 2 - 2 * d ** 2, 2 * b * c - 2 * a * d, 2 * a * c + 2 * b * d],
                  [2 * b * c + 2 * a * d, 1 - 2 * b ** 2 - 2 * d ** 2, 2 * c * d - 2 * a * b],
                  [2 * b * d - 2 * a * c, 2 * a * b + 2 * c * d, 1 - 2 * b ** 2 - 2 * c ** 2]])
    res = np.dot(r, v)
    res = np.squeeze(np.transpose(res))
    return res


# rotation matrix to qt
def r2q(R):
    T11 = R[0, 0]
    T12 = R[0, 1]
    T13 = R[0, 2]

    T21 = R[1, 0]
    T22 = R[1, 1]
    T23 = R[1, 2]

    T31 = R[2, 0]
    T32 = R[2, 1]
    T33 = R[2, 2]

    q2 = np.sqrt((T12 + T21) * (T23 + T32) / (T13 + T31)) / 2
    q3 = (T13 + T31) / (T12 + T21) * q2
    q1 = (T12 + T21) / 4 / q2
    q4 = np.sqrt(T11 - q1 ** 2 + q2 ** 2 + q3 ** 2)

    return np.array([[q1, q2, q3, q4]])  # in the form of [s, u1, u2, u3]


def smooth(qs, qm, Eg, Ec):
    k = Eg / (Eg + Ec)
    resQ = qs * (1 - k) + k * qm
    resE = (1 - k) * Eg
    return resQ, resE


def qTrans(qt):
    res = [qt[0, 3], qt[0, 0], qt[0, 1], qt[0, 2]]

    return res


def matchData(input):
    mnx = []
    mny = []
    mnz = []
    
    gnx = []
    gny = []
    gnz = []

    ax = input[0]

    mx = input[9]
    my = input[10]
    mz = input[11]
    
    gx = input[3]
    gy = input[4]
    gz = input[5]

    r = len(mx) / len(ax)
    rg = len(gx) / len(ax)
    for i in range(len(ax)):
        t = math.floor(i * r)
        mnx.append(mx[t])
        mny.append(my[t])
        mnz.append(mz[t])
        
        tg = math.floor(i*rg)
        gnx.append(gx[tg])
        gny.append(gy[tg])
        gnz.append(gz[tg])

    input[9] = mnx
    input[10] = mny
    input[11] = mnz
    
    input[3] = gnx
    input[4] = gny
    input[5] = gnz

    return input
