"""
Author:     Johannes Hauck
Source:
Content:    File includes calculation of values on cubic Bézier curve.
"""


import numpy as np


# calculates one point on a cubic Bézier curve
# input:    - uVal (float): parameter of value u
#           - b (list) = [b0, b1, ..., bn] control and Bézier points as list
# output:   - point on the curve (list)
def getBezierValue(uVal, b):
    return b[0]*((1-uVal)**3) + b[1]*3*uVal*((1-uVal)**2) + b[2]*3*(uVal**2)*(1-uVal) + b[3]*(uVal**3)


# calculates N discrete values on a cubic Bézier curve
# input:    - b (list) = [b0, b1, ..., bn] control and Bézier points as list
#           - N (int): number of points on the curve
# output:   - points on the curve (list)
def getBezierValues(b, N):
    uValues = np.linspace(0, 1, N)
    bezValues = []
    for i in uValues:
        bezValues.append(getBezierValue(i, b))
    return bezValues
