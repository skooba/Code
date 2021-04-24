import math

import numpy as np
from cvxopt import solvers, matrix


def jacobian(a1, a2, a3, angle1, angle2, angle3):  # a1, a2, a3 are the arm lengths
    toRadians = 2 * math.pi / 360
    q1 = angle1 - 90*toRadians  # transform joint angle representations to
    q2 = angle2 - q1 - 90*toRadians
    q3 = angle3 - q2 - 90*toRadians
    J11 = -a2 * math.sin((q1 + q2)) - a1 * math.sin(q1) - a3 * math.sin(
        (q1 + q2 + q3))
    J12 = -a2 * math.sin((q1 + q2)) - a3 * math.sin((q1 + q2 + q3))
    J13 = -a3 * math.sin((q1 + q2 + q3))
    J21 = a2 * math.cos((q1 + q2)) + a1 * math.cos(q1) + a3 * math.cos(
        (q1 + q2 + q3))
    J22 = a2 * math.cos((q1 + q2)) + a3 * math.cos((q1 + q2 + q3))
    J23 = a3 * math.cos((q1 + q2 + q3))
    rotMat = np.asarray([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  #0 rotation to allign base and tool coordinate frames
    jacobian3D = np.matmul(np.array([[J11, J12, J13], [J21, J22, J23], [1, 1, 1]]),
                           rotMat)  # If we had 3d motion this is what our Jacobian would look like
    jacobian2D = jacobian3D[:2, :]  # here is the redundant Jacobian Matrix
    scaleX = -a1 * math.sin(angle1) - 2 * a2 * math.sin(angle1+angle2) - 3 * a3 * math.sin(angle1 + angle2 + angle3)
    scaleY = a1 * math.cos(angle1) + 2 * a2 * math.cos(angle1+angle2) + 3 * a3 * math.cos(angle1 + angle2 + angle3)
    return jacobian2D, scaleX, scaleY





def cartVelocities(x_centroid, y_centroid, x_center, y_center):
    xUnscaled = x_centroid - x_center
    yUnscaled = y_centroid - y_center
    xScaled = xUnscaled / abs(abs(xUnscaled) + abs(yUnscaled))
    yScaled = yUnscaled / abs(abs(xUnscaled) + abs(yUnscaled))
    return xScaled, yScaled


def solveQP(safety, min1, min2, min3, max1, max2, max3, jacobi, x, y, angle1, angle2, angle3):
    toRadians = 2 * math.pi / 360
    minAngle1 = min1 + safety
    minAngle2 = min2 + safety
    minAngle3 = min3 + safety
    maxAngle1 = max1 - safety
    maxAngle2 = max2 - safety
    maxAngle3 = max3 - safety
    P = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # Weighting matrix
    q = matrix([0.0, 0.0, 0.0])  # linear term we are not using
    G = matrix([[-1.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0, 0.0, 0.0, 1.0]])  # first three are lower bounds, next three are upper bounds
    h = matrix([angle1 - minAngle1, angle2 - minAngle2, angle3 - minAngle3,
                maxAngle1 - angle1, maxAngle2 - angle2, maxAngle3 - angle3])  # can't move joints more than this
    A = matrix(jacobi)
    b = matrix([x, y])  # directional vector in cartesian coordinates it is necessary to travel
    sol = solvers.qp(P, q, G, h, A, b)
    return sol['x']
