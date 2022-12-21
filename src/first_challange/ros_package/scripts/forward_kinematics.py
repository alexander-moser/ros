import numpy as np
import sympy as sp

#using sympy instead of numpy, so sin(pi) will actually be zero, and not just a very small value
cos = sp.cos
sin = sp.sin
atan2 = sp.atan2


def calculate_end_effector_position(total_t_matrix):

    x = total_t_matrix[:-1][0,-1]
    y = total_t_matrix[:-1][1,-1]
    z = total_t_matrix[:-1][2,-1]
    phi, psi, theta = calculate_euler_angles(total_t_matrix)

    result = [x,y,z,phi,psi,theta]

    return result

def calculate_euler_angles(total_t_matrix):

    phi = atan2(total_t_matrix[2][1], total_t_matrix[2][2])
    psi = atan2(total_t_matrix[1][0], total_t_matrix[0][0])
    theta  = atan2(-(total_t_matrix[2][0]), sp.sqrt((total_t_matrix[0][0] ** 2) + (total_t_matrix[1][0] ** 2)).evalf())

    return phi,psi,theta

def calculate_incrementing_t_matrices(a, alpha, d, theta):
    number_of_joints = len(theta)

    T = calculate_all_t_matrizes(a, alpha, d, theta)

    result = np.array([[]])

    #matrix T01 is simply the first T matrix
    current_matrix = T[0]
    #mutiplying the next T matrix on top and saving it
    for i in range(1, number_of_joints):
        current_matrix = current_matrix @ T[i]
        result = np.append(result, current_matrix)

    result = result.reshape(-1,4,4)

    return result

def calculate_total_t_matrix(a, alpha, d, theta):

    number_of_joints = len(theta)

    T = calculate_all_t_matrizes(a, alpha, d, theta)

    result = T[0]
    for i in range(1, number_of_joints):
        result = result @ T[i]

    return result

def calculate_all_t_matrizes(a, alpha, d, theta):

    number_of_joints = len(theta)

    Ts = []

    for i in range(number_of_joints):
        t_matrix = calculate_t_matrix(a[i], alpha[i], d[i], theta[i])
        Ts.append(t_matrix)

    return Ts

def calculate_t_matrix(a, alpha, d, theta):

    A = np.array([
        [(cos(theta)).evalf(), (-sin(theta)).evalf(), 0, 0],
        [(sin(theta)).evalf(), (cos(theta)).evalf(), 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

    B = np.array([
        [1, 0, 0, a],
        [0, (cos(alpha)).evalf(), (-sin(alpha)).evalf(), 0],
        [0, (sin(alpha)).evalf(), (cos(alpha)).evalf(), 0],
        [0, 0, 0, 1]
    ])

    return A @ B



