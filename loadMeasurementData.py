"""
Author:     Johannes Hauck
Source:
Content:    File includes functions to load a measurement file.
"""


from tkinter import Tk
from tkinter.filedialog import askopenfilename
from readMeasurementData import findAll, getPose, getTimestamp, getVValue, getUValue, getControlBezPoints
import numpy as np


# load measurement data from .txt file
# input:    - path (string): path of the measurement files
# output:   - measurement data array
def loadMeasurement(path):

    if path[-4:] == '.txt':
        filename = path
    else:
        # select file
        Tk().withdraw()
        filename = askopenfilename(title='select measurement', initialdir=path)

    # save filename
    fileN = filename[findAll(filename, '/')[-1] + 1:len(filename)]

    # load file
    file = open(filename, "r")
    mdString = file.readlines()
    file.close()

    # check measurement file type
    uBool = False
    vBool = False
    # without time
    if mdString[0][0] == '(':
        timeBool = False
    # with time
    else:
        timeBool = True
        commas = findAll(mdString[0], ',')
        # check with u
        if commas.__len__() > 1 and commas[1] < findAll(mdString[0], '(')[0]:
            uBool = True
            # check with velocity
            if commas[2] < findAll(mdString[0], '(')[0]:
                vBool = True
            else:
                vBool = False
        else:
            uBool = False

    # measurement data transformation
    posen = []
    time = []
    uValues = []
    vValues = []

    for i in range(0, mdString.__len__() - 4, 1):
        # transform poses
        pose = getPose(mdString[i])
        posen.append(np.array(pose))

        # load time
        if timeBool:
            timestamp = getTimestamp(mdString[i])
            time.append(timestamp)
        # load u
        if uBool:
            uValue = getUValue(mdString[i])
            uValues.append(uValue)
        # load velocity
        if vBool:
            vValue = getVValue(mdString[i])
            vValues.append(vValue)

    # lod control and Bézier points
    controlBezPoints = []
    for i in range(mdString.__len__() - 4, mdString.__len__(), 1):
        cp = getControlBezPoints(mdString[i])
        controlBezPoints.append(np.array(cp))

    # output
    if timeBool and uBool and vBool:
        return [posen, controlBezPoints, fileN, time, uValues, vValues]
    elif timeBool and uBool:
        return [posen, controlBezPoints, fileN, time, uValues]
    elif timeBool:
        return [posen, controlBezPoints, fileN, time]
    else:
        return [posen, controlBezPoints, fileN]


# extract standard deviation of noise in filename
# input:    - noiseStr (string): Name of standard deviation
#           - file (string): filename
#           - und (array): positions of '_' in filename
# output:   - (float): standard deviation
def getNoise(noiseStr, file, und):
    u = file.find(noiseStr)
    if u > -1:
        return float(file[u + 8:und[und.index(u) + 2]])
    else:
        return 0


# return measurement configuration
# input:    - file (string): filename
#           - abSkalTrue (bool): True if measurement is scaled
# output:   - (array): number of measurement points and standard deviation
def filenameConfig(file, abSkalTrue):
    und = findAll(file, '_')

    uN = file.find('_P_')
    N = float(file[uN + 3:und[und.index(uN) + 2]])

    T = getNoise('_NoiseT_', file, und)
    R = getNoise('_NoiseR_', file, und)
    S = getNoise('_NoiseS_', file, und)

    # scaled measurement: find scaling factor
    if abSkalTrue:
        abSkal = getNoise('_abSkal_', file, und)
        return [N, T, R, S, abSkal]
    else:
        return [N, T, R, S]


# calculate camera coordinate points for plot
# input:    - t (array) camera translation
#           - rot: (ndarray) camera rotation matrix
#           - normFct: factor for scaling the axis length
# output:   - plot vector [[x axis],[y axis],[z axis]], [x axis] = [[x values],[y values],[z values]]
def cameraCoord(t, rot, normFct):
    x = np.add(t, normFct * rot[:, 0].T)
    y = np.add(t, normFct * rot[:, 1].T)
    z = np.add(t, normFct * rot[:, 2].T)
    return [[t[0], x[0]], [t[1], x[1]], [t[2], x[2]]], [[t[0], y[0]], [t[1], y[1]], [t[2], y[2]]], [[t[0], z[0]], [t[1], z[1]], [t[2], z[2]]]

