import numpy as np
import sympy as sp
import forward_kinematics

#using sympy instead of numpy, so sin(pi) will actually be zero, and not just a very small value
cos = sp.cos
sin = sp.sin
atan2 = sp.atan2


def calculate_analytic_jacobian_pseudo_invers(a, alpha, d, theta):

    analytic_jacobian = calculate_analytic_jacobian(a, alpha, d, theta)
    
    result = np.transpose(analytic_jacobian) @ np.linalg.pinv(analytic_jacobian @ np.transpose(analytic_jacobian))

    return result

def calculate_analytic_jacobian(a, alpha, d, theta):

    total_t_matrix = forward_kinematics.calculate_total_t_matrix(a, alpha, d, theta)
    jacobian_x = calculate_jacobian_x(total_t_matrix)
    geometric_jacobian = calculate_geometric_jacobian(a, alpha, d, theta)

    result = np.linalg.pinv(np.float64(jacobian_x)) @ np.float64(geometric_jacobian)

    return result

def calculate_geometric_jacobian(a, alpha, d, theta):

    Ts = forward_kinematics.calculate_incrementing_t_matrices(a, alpha, d, theta)
    total_T_matrix = forward_kinematics.calculate_total_t_matrix(a, alpha, d, theta)
    
    Zs = np.array([[0,0,1]])

    for matrix in Ts:
        Zs = np.append(Zs, matrix[:-1][:, 2])
    Zs = Zs.reshape(-1,3)

    Pe = total_T_matrix[:-1][:, 3]

    Ps = np.array([[0,0,0]])

    for matrix in Ts:
        Ps = np.append(Ps, matrix[:-1][:, 3])
    Ps = Ps.reshape(-1,3)

    upperrow = np.array([])
    lowerrow = np.array([])

    for i in range (len(Zs)):
        w = np.cross(Zs[i], (Pe-Ps[i]))
        upperrow = np.append(upperrow, w)
        lowerrow = np.append(lowerrow, Zs[i])

    upperrow = upperrow.reshape(-1, 3)
    lowerrow = lowerrow.reshape(-1, 3)

    result = np.hstack((upperrow, lowerrow))

    result = np.transpose(result)

    return result

def calculate_jacobian_x(total_t_matrix):

    jacobian_theta = calculate_jacobian_theta(total_t_matrix)

    ones_zeros = np.concatenate((np.identity(3), np.zeros((3, 3), dtype=float)), axis=1)
    zeros_theta = np.concatenate((np.zeros((3, 3), dtype=float), jacobian_theta), axis=1)
    jacobian_x = np.concatenate((ones_zeros, zeros_theta), axis=0)

    return jacobian_x

def calculate_jacobian_theta(total_t_matrix):
    
    phi,psi,theta = forward_kinematics.calculate_euler_angles(total_t_matrix)

    jacobian_theta = np.array([
        [cos(psi).evalf()*sin(theta).evalf(), -sin(psi).evalf(), 0],
        [sin(psi).evalf()*sin(theta).evalf(), cos(psi).evalf(), 0],
        [cos(theta).evalf(), 0, 1]
    ])

    return jacobian_theta