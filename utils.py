from numpy import array, cross, eye, dot
from scipy.linalg import expm3,norm
from math import sin, cos

def rotate(axis, vector, angle):
    if axis == 'x':
        m = array([
            [1, 0, 0],
            [0, cos(angle), -sin(angle)],
            [0, sin(angle), cos(angle)]
        ])
    elif axis == 'y':
        m = array([
            [cos(angle), 0, sin(angle)],
            [0, 1, 0],
            [-sin(angle), 0, cos(angle)]
        ])
    elif axis == 'z':
        m = array([
            [cos(angle), -sin(angle), 0],
            [sin(angle), cos(angle), 0],
            [0, 0, 1]
        ])
    else:
        raise Exception("axis needs to be x, y or z")

    return dot(m, vector)
