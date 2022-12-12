#! /usr/bin/env python3
import math
import numpy as np

def kuka_IK(point, R, joint_positions):
    cos = math.cos
    sin = math.sin
    pi = math.pi

    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    # initialize robot links
    k = 0.3330
    l = 0.3160
    m = 0.3840
    n = 0.1070

    # manipulate tolerance
    tolerance = 0.001

    # set initial error to a big value
    error_end = 100

    #dh-parameter
    d = [0.3330, 0.0000, 0.3160, 0.0000, 0.3840, 0.0000, 0.1070]
    theta = [q[0]+pi, q[1]-pi, q[2], q[3]+pi, q[4]-pi, q[5], q[6]]
    a = [0.0000, 0.0000, 0.0825, 0.0825, 0.0000, 0.0880, 0.0000]
    alpha = [pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, 0]

    q = theta

    # repeat until error <= tolerance
    while error_end > tolerance:
        # calculate transformation
        t10 = [
            [cos(q[0]), 0, sin(q[0]), 0],
            [sin(q[0]), 0, -cos(q[0]), 0],
            [0, 1, 0, d[0]],
            [0, 0, 0, 1]
        ]
        t21 = [
            [cos(q[1]), 0, -sin(q[1]), 0],
            [sin(q[1]), 0, cos(q[1]), 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ]
        t32 = [
            [cos(q[2]), 0, sin(q[2]), a[2]*cos(q[2])],
            [sin(q[2]), 0, -cos(q[2]), a[2]*sin(q[2])],
            [0, 1, 0, d[2]],
            [0, 0, 0, 1]
        ]
        t43 = [
            [cos(q[3]), 0, -sin(q[3]), a[3]*cos(q[3])],
            [sin(q[3]), 0, cos(q[3]), a[3]*sin(q[3])],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ]
        t54 = [
            [cos(q[4]), 0, sin(q[4]), 0],
            [sin(q[4]), 0, -cos(q[4]), 0],
            [0, 1, 0, d[4]],
            [0, 0, 0, 1]
        ]
        t65 = [
            [cos(q[5]), 0, sin(q[5]), a[5]*cos(q[5])],
            [sin(q[5]), 0, -cos(q[5]), a[5]*sin(q[5])],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ]
        t76 = [
            [cos(q[6]), -sin(q[6]), 0, 0],
            [sin(q[6]), cos(q[6]), 0, 0],
            [0, 0, 1, d[6]],
            [0, 0, 0, 1]
        ]

        t20 = np.dot(t10, t21)
        t30 = np.dot(t20, t32)
        t40 = np.dot(t30, t43)
        t50 = np.dot(t40, t54)
        t60 = np.dot(t50, t65)
        t70 = np.dot(t60, t76)

        # forward kinamatics
        pos7 = [0, 0, 0, 1]
        pos0 = np.dot(t70, pos7)

        # calculate end difference
        diff_x = pos0[0] - x
        diff_y = pos0[1] - y
        diff_z = pos0[2] - z

        diff_pos = [diff_x, diff_y, diff_z]

        ori1 = np.cross(np.dot(R, [1, 0, 0]), np.dot(t70, [1, 0, 0, 0])[:3])

        ori2 = np.cross(np.dot(R, [0, 1, 0]), np.dot(t70, [0, 1, 0, 0])[:3])

        ori3 = np.cross(np.dot(R, [0, 0, 1]), np.dot(t70, [0, 0, 1, 0])[:3])

        diff_ori = 1/2*(ori1 + ori2 + ori3)

        diff_end = np.concatenate((diff_pos, diff_ori))

        # calculate jacobian
        vec1 = [0, 0, 1, 0]
        vec2 = [0, 0, 0, 1]

        z0 = [0, 0, 1]
        z1 = np.dot(t10, vec1)[:3]
        z2 = np.dot(t20, vec1)[:3]
        z3 = np.dot(t30, vec1)[:3]
        z4 = np.dot(t40, vec1)[:3]
        z5 = np.dot(t50, vec1)[:3]
        z6 = np.dot(t60, vec1)[:3]

        p0 = [0, 0, 0]
        p1 = np.dot(t10, vec2)[:3]
        p2 = np.dot(t20, vec2)[:3]
        p3 = np.dot(t30, vec2)[:3]
        p4 = np.dot(t40, vec2)[:3]
        p5 = np.dot(t50, vec2)[:3]
        p6 = np.dot(t60, vec2)[:3]
        pe = np.dot(t70, vec2)[:3]

        jacobian = np.transpose(
            [
                np.concatenate((np.cross(z0, pe-p0), z0)),
                np.concatenate((np.cross(z1, pe-p1), z1)),
                np.concatenate((np.cross(z2, pe-p2), z2)),
                np.concatenate((np.cross(z3, pe-p3), z3)),
                np.concatenate((np.cross(z4, pe-p4), z4)),
                np.concatenate((np.cross(z5, pe-p5), z5)),
                np.concatenate((np.cross(z6, pe-p6), z6)),
            ]
        )

        # calculate inverse jacobian
        jacobian_inv = np.linalg.pinv(jacobian)

        # calculate config difference
        diff_config = np.dot(jacobian_inv, diff_end)

        # calculate end error
        error_end = np.linalg.norm(diff_end)
        
        # update config
        q = q - diff_config

        #print(q)

    return q

# joint1 = -3.123283e-05
# joint2 = -9.512901e-05
# joint3 = -7.152557e-07
# joint4 = -1.5708812475
# joint5 = -0.0002501010
# joint6 = 1.56948256492
# joint7 = -3.004074e-05

# point = [-0.07, 0, 0.413]

# joints = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]

# kuka_IK(point, 1, joints)