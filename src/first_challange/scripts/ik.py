import math
import numpy as np
from math import sin, cos

d = [0.3330, 0.0000, 0.3160, 0.0000, 0.3840, 0.0000, 0.1070]
theta = [180, -180, 0, 90, -180, 90, 0]
a = [0.0000, 0.0000, 0.0825, 0.0825, 0.0000, 0.0880, 0.0000]
alpha = [90, 90, 90, 90, 90, 90, 0]


def calc_a(num):
    theta_val = theta[num]
    a_val = a[num]

    if num == 0:
        mat = np.array([
            [cos(theta_val), 0, sin(theta_val), 0],
            [sin(theta_val), 0, -cos(theta_val), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
    elif num == 1:
        mat = np.array([
            [cos(theta_val), 0, -sin(theta_val), 0],
            [sin(theta_val), 0, cos(theta_val), 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
    elif num == 2:
        mat = np.array([
            [cos(theta_val), 0, -sin(theta_val), 0],
            [sin(theta_val), 0, cos(theta_val), 0],
            [0, -1, 0, d[2]],
            [0, 0, 0, 1]
        ])
    elif num == 3:
        mat = np.array([
            [cos(theta_val), 0, sin(theta_val), 0],
            [sin(theta_val), 0, -cos(theta_val), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
    elif num == 4:
        mat = np.array([
            [cos(theta_val), 0, sin(theta_val), 0],
            [sin(theta_val), 0, -cos(theta_val), 0],
            [0, 1, 0, d[4]],
            [0, 0, 0, 1]
        ])
    elif num == 5:
        mat = np.array([
            [cos(theta_val), 0, -sin(theta_val), 0],
            [sin(theta_val), 0, cos(theta_val), 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])
    elif num == 6:
        mat = np.array([
            [cos(theta_val), -sin(theta_val), 0, 0],
            [sin(theta_val), cos(theta_val), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    return mat

point = [0, 0, 0.413, 0, 0, 0]
#point = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]


def ik(learning_rate):
    tolerance = 2.5   
    error = 100
                                    
    counter = 0

    while error > tolerance:
        vec1 = [0, 0, 1]

        a_10 = calc_a(0)
        a_21 = calc_a(1)
        a_32 = calc_a(2)
        a_43 = calc_a(3)
        a_54 = calc_a(4)
        a_65 = calc_a(5)
        a_76 = calc_a(6)

        t_01 = a_10
        t_02 = np.dot(a_10, a_21)
        t_03 = np.dot(t_02, a_32)
        t_04 = np.dot(t_03, a_43)
        t_05 = np.dot(t_04, a_54)
        t_06 = np.dot(t_05, a_65)
        t_07 = np.dot(t_06, a_76)

        r_01 = t_01[:-1, :-1]
        r_02 = t_02[:-1, :-1]
        r_03 = t_03[:-1, :-1]
        r_04 = t_04[:-1, :-1]
        r_05 = t_05[:-1, :-1]
        r_06 = t_06[:-1, :-1]
        # r_07 = t_07[:3, :3]

        z_0 = [[0, 0, 1]]
        z_1 = np.expand_dims(r_01[:, 2], axis=0)
        z_2 = np.expand_dims(r_02[:, 2], axis=0)
        z_3 = np.expand_dims(r_03[:, 2], axis=0)
        z_4 = np.expand_dims(r_04[:, 2], axis=0)
        z_5 = np.expand_dims(r_05[:, 2], axis=0)
        z_6 = np.expand_dims(r_06[:, 2], axis=0)

        p0 = [[0, 0, 0]]
        p1 = np.transpose(t_01[:-1, -1:])
        p2 = np.transpose(t_02[:-1, -1:])
        p3 = np.transpose(t_03[:-1, -1:])
        p4 = np.transpose(t_04[:-1, -1:])
        p5 = np.transpose(t_05[:-1, -1:])
        p6 = np.transpose(t_06[:-1, -1:])
        pe = np.transpose(t_07[:-1, -1:])

        jacobian = np.transpose(np.concatenate(
            (
                np.concatenate((np.cross(z_0, pe - p0), z_0), axis=1),
                np.concatenate((np.cross(z_1, pe - p1), z_1), axis=1),
                np.concatenate((np.cross(z_2, pe - p2), z_2), axis=1),
                np.concatenate((np.cross(z_3, pe - p3), z_3), axis=1),
                np.concatenate((np.cross(z_4, pe - p4), z_4), axis=1),
                np.concatenate((np.cross(z_5, pe - p5), z_5), axis=1),
                np.concatenate((np.cross(z_6, pe - p6), z_6), axis=1)
            )
        ))

        point_phi = math.atan2(t_07[2][2], t_07[2][1])
        point_psi = math.atan2(t_07[0][0], t_07[1][0])
        point_theta = math.atan2(math.sqrt((t_07[0][0] ** 2) + (t_07[1][0] ** 2)), -t_07[2][0])
        x = t_07[0][3]
        y = t_07[1][3]
        z = t_07[2][3]

        f_q = np.transpose([x, y, z, point_phi, point_psi, point_theta])

        j_theta = [[cos(point_psi) * sin(point_theta), -sin(point_psi), 0],
                   [sin(point_psi) * sin(point_theta), cos(point_psi), 0],
                   [cos(point_theta), 0, 1]]

        ones_zeros = np.concatenate((np.identity(3), np.zeros((3, 3), dtype=float)), axis=1)
        zeros_theta = np.concatenate((np.zeros((3, 3), dtype=float), j_theta), axis=1)
        jacobian_x = np.concatenate((ones_zeros, zeros_theta), axis=0)

        jacobian_a = np.dot(np.linalg.pinv(jacobian_x), jacobian)

        jacobian_a_plus = np.matmul(np.transpose(jacobian_a),
                                    np.linalg.pinv(np.matmul(jacobian_a, np.transpose(jacobian_a))))


        diff = f_q - point
        error = np.linalg.norm(f_q - point)
        print(error)

        # print("diff: ")
        # print(diff)
        # print("error: ")
        # print(error)

        global theta
        theta = theta -(learning_rate * np.dot(jacobian_a_plus, diff))

        # print('theta: ' + str(theta))

        vel = -(learning_rate * np.dot(jacobian_a_plus, diff))

        # print('counter: ' + str(counter))
        #if (counter % 100 == 0):
        #    learning_rate = learning_rate * 0.1

    return vel
