"""
Author:     Johannes Hauck
Source:
Content:    File includes the functions of the algorithm for optimized control point estimation.
"""


import numpy as np


# computes the control points
# input:    - u (float): u value at current measurement point
#           - b0 (nd.array): first Bézier point
#           - b3 (nd.array): second Bézier point
#           - fu (nd.array): translation camera
#           - fuDerv (nd.array): fist derivation camera
# output:   - [b1, b2] (nd.array): control points
def calcControlPointsEqSys(u, b0, b3, fu, fuDerv):
    # right sight equation system
    rs = np.array((fu - (np.power((1-u), 3)*b0) - np.power(u, 3) * b3, fuDerv + (3 * np.power((1 - u), 2) * b0) - (3 * np.power(u, 2) * b3)))
    # left side equation system
    ls = np.array([[3*u*np.power((1-u), 2), 3*np.power(u, 2)*(1-u)], [3*(np.power((1-u), 2)-2*u*(1-u)), 3*(2*u*(1-u)-np.power(u, 2))]])
    # solve and return
    return np.linalg.solve(ls, rs)


# computes fist derivation with velocity and time and calls control point calculation
# input:    - u (float): u value at current measurement point
#           - b0 (nd.array): first Bézier point
#           - b3 (nd.array): second Bézier point
#           - fu (nd.array): translation camera
#           - tan (nd.array): norm. tangent at fu
#           - v (float): absolute velocity at fu
#           - tAct (float): current time at fu
#           - tStart (float): start time
# output:   - [b1, b2] (nd.array): control points
def calcControlPointsUVTan(u, b0, b3, fu, tan, v, tAct, tStart):
    # calculation velocity vector = first derivation
    vVek = tan * (v * ((tAct - tStart) / u))
    # control point calculation
    return calcControlPointsEqSys(u, b0, b3, fu, vVek)


# calculate all estimations for the control points
# input:    - b0 (nd.array): first Bézier point
#           - b3 (nd.array): second Bézier point
#           - poses (array[nd.array]): all 4x4 camera poses
#           - v (array[float]): absolute velocity at fu
#           - t (array[float]): measurement time vector
# output:   - [[b1i, b2i], ...] (array by nd.arrays): all estimations for both control points
def calcControlPointsMeasurement(b0, b3, poses, v, t):
    # calculate u values by time
    u = np.divide(t, t[-1] - t[0])
    b23EstMeasurement = np.zeros((u.__len__(), 6))

    # control point estimation
    for i in range(1, u.__len__()-1, 1):
        # calculation
        b23Est = calcControlPointsUVTan(u[i], b0, b3, poses[i][0:3, 3], -poses[i][0:3, 2], v[i], t[i], t[0])
        # save both control points
        b23EstMeasurement[i, 0:3] = b23Est[0, :]
        b23EstMeasurement[i, 3:] = b23Est[1, :]

    return b23EstMeasurement


# computes the optimized control points out of all estimations and the median
# input:    - b23Est (nd.array): all estimated control points
# output:   - b23EstOpt (nd.array): optimized control points
#           - madOpt (nd.array): median absolute deviation
def b23EstOptMedian(b23Est):
    b23EstOpt = np.zeros((2, 3))
    b23EstOpt[0, :] = (np.median(b23Est[1:-1, 0]), np.median(b23Est[1:-1, 1]), np.median(b23Est[1:-1, 2]))
    b23EstOpt[1, :] = (np.median(b23Est[1:-1, 3]), np.median(b23Est[1:-1, 4]), np.median(b23Est[1:-1, 5]))
    madOpt = np.zeros((2, 3))
    madOpt[0, :] = (mad(b23EstOpt[0, 0], b23Est[1:-1, 0]), mad(b23EstOpt[0, 1], b23Est[1:-1, 1]), mad(b23EstOpt[0, 2], b23Est[1:-1, 2]))
    madOpt[1, :] = (mad(b23EstOpt[1, 0], b23Est[1:-1, 3]), mad(b23EstOpt[1, 1], b23Est[1:-1, 4]), mad(b23EstOpt[1, 2], b23Est[1:-1, 5]))
    return b23EstOpt, madOpt


# calculates median absolute deviation
# input:    - median (float): median of all estimations
#           - values (nd.array): all estimations
# output:   - (float): median absolute deviation
def mad(median, values):
    maam = 0
    for i in values:
        maam += np.absolute(i-median)
    return maam/values.__len__()


# calculates the optimal control points by estimating all points control points by the measurement data
# input:    - u (float): u value at current measurement point
#           - b0 (nd.array): first Bézier point
#           - b3 (nd.array): second Bézier point
#           - poses (array[nd.array]): all 4x4 camera poses
#           - v (array[float]): absolute velocity at fu
#           - t (array[float]): measurement time vector
# output:   - b23EstOpt (nd.array): optimized control points
#           - madOpt (nd.array): median absolute deviation
def estOptControlPoints(b0, b3, poses, v, zeit):
    # estimate all control points
    b23Est = calcControlPointsMeasurement(b0, b3, poses, v, zeit)
    # optimization and calculation median absolute deviation
    return b23EstOptMedian(b23Est)
