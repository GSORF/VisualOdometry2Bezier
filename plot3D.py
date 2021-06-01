"""
Author:     Johannes Hauck
Source:
Content:    With this file its possible to plot measurement data in an 3D coordinate system.
            The algorithm for optimized control point estimation is callable, the estimated control points and the
            reconstructed trajectory can be visualized.
            Configurations can be done at files end.
            Run this file for visualisation!
"""


from loadMeasurementData import loadMeasurement, filenameConfig, cameraCoord
from beziercurve import getBezierValues
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from controlPointOptimization import estOptControlPoints
from readMeasurementData import findAll

# Functions from @Mateen Ulhaq and @karlo
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


# function generates 3D plot
# input:    - md (array): measurement data array
#           - b23EstOpt (ndarray): np.array with optimized estimated control points
#           - plot bool (boolean array): shows what should be plotted: True: contained in plot, False: not contained
#               [0:translations , 1:control polygon, 2:control points, 3:Bézier points, 4: ground truth,
#                5: camera coordinate systems, 6: optimized control points, 7: optimized curve curve]
def plot3D(md, b23EstOpt, plotBool):
    # ground truth calculation
    if plotBool[4]:
        gt = getBezierValues(md[1], 500)

    # calculate camera coordinate frames
    if plotBool[5]:
        kamKord = []
        for i in range(0, md[3].__len__(), 1):
            kamKord.append(cameraCoord(md[0][i][0:3, -1], md[0][i][0:3, 0:3], 0.2))

    # calculated optimized Bézier curve
    if plotBool[7]:
        est = getBezierValues([md[1][0], b23EstOpt[0][0, :], b23EstOpt[0][1, :], md[1][3]], 500)

    # Plot ############################################################################################################
    fig1 = plt.figure()

    ax1 = fig1.add_subplot(111, projection='3d')

    # plot control polygon
    if plotBool[1]:
        ax1.plot([i[0] for i in md[1]], [i[1] for i in md[1]], [i[2] for i in md[1]], '--k', linewidth=2, label='control polygon')

    # plot Bézier points
    if plotBool[3]:
        ax1.plot([i[0] for i in [md[1][0], md[1][3]]], [i[1] for i in [md[1][0], md[1][3]]], [i[2] for i in [md[1][0], md[1][3]]], 'ob', mew=2, zorder=md[0].__len__() * 3, label='Bézier points')

    # plot control points
    if plotBool[2]:
        ax1.plot([i[0] for i in md[1][1:3]], [i[1] for i in md[1][1:3]], [i[2] for i in md[1][1:3]], 'or', mew=2, zorder=md[0].__len__() * 3, label = 'control points')

    # plot ground truth
    if plotBool[4]:
        ax1.plot([i[0] for i in gt], [i[1] for i in gt], [i[2] for i in gt], '-m', linewidth=2, label='ground truth')

    # plot camera coordinate systems
    if plotBool[5]:
        plt.plot(kamKord[0][0][0], kamKord[0][0][1], kamKord[0][0][2], '-r', linewidth=2, label='camera x-axis')
        plt.plot(kamKord[0][1][0], kamKord[0][1][1], kamKord[0][1][2], '-g', linewidth=2, label='camera y-axis', zorder=md[0].__len__() * 3)
        plt.plot(kamKord[0][2][0], kamKord[0][2][1], kamKord[0][2][2], '-b', linewidth=2, label='camera z-axis')
        for i in range(1, md[3].__len__(), 1):
            plt.plot(kamKord[i][0][0], kamKord[i][0][1], kamKord[i][0][2], '-r', linewidth=2)
            plt.plot(kamKord[i][1][0], kamKord[i][1][1], kamKord[i][1][2], '-g', linewidth=2, zorder=md[0].__len__() * 3)
            plt.plot(kamKord[i][2][0], kamKord[i][2][1], kamKord[i][2][2], '-b', linewidth=2)

    # plot camera translations
    if plotBool[0]:
        plt.plot([i[0][3] for i in md[0]], [i[1][3] for i in md[0]], [i[2][3] for i in md[0]], 'xg', mew=2,
                 label='translations', markersize=6 if plotBool[5] else 8)

    # plot optimized control points
    if plotBool[6]:
        ax1.plot(b23EstOpt[0][0:2, 0], b23EstOpt[0][0:2, 1], b23EstOpt[0][0:2, 2], 'x', color='darkred', mew=2, zorder=md[0].__len__() * 3 + 1, label='optimized $\mathbf{b}_1$, $\mathbf{b}_2$', markersize=8)

    # plot optimized curve
    if plotBool[7]:
        est = getBezierValues([md[1][0], b23EstOpt[0][0, :], b23EstOpt[0][1, :], md[1][3]], 500)
        ax1.plot([i[0] for i in est], [i[1] for i in est], [i[2] for i in est],  '-', color='darkred', linewidth=2, label='optimized curve')


    # read measurement configuration
    c = filenameConfig(md[2], False)
    unter = unt = findAll(md[2], '_')
    titel = 'Bézier curve ' + md[2][:unt[0]] + (' with estimated control points ' if plotBool[6] else ' ') + 'at $N='+str(c[0])+'$' + (', $\sigma_T = '+str(c[1])+'\ m$' if c[1] != 0 else '') + (', $\sigma_R = '+str(c[2])+'\ °$' if c[2] != 0 else '')+ (', $\sigma_S = '+str(c[3])+'\ m/s$' if c[3] != 0 else '')

    ax1.grid()

    handles, labels = ax1.get_legend_handles_labels()
    lg = ax1.legend(handles, labels, bbox_to_anchor=(1, 0.95))
    plt.suptitle(titel, fontsize=20)
    plt.title('measurement file: ' + md[2], fontsize=12)
    ax1.set_xlabel('x [m]', fontsize=20)
    ax1.set_ylabel('y [m]', fontsize=20)
    ax1.set_zlabel('z [m]', fontsize=20)
    ax1.set_box_aspect((1, 1, 1))
    set_axes_equal(ax1)
    ax1.view_init(azim=230, elev=30) # default view

    # show plot
    plt.show()


if __name__ == "__main__":
    # user configuration
    # set True what should be plotted
    #[0:translations , 1:control polygon, 2:control points, 3:Bézier points, 4: ground truth,
    # 5: camera coordinate systems, 6: optimized control points, 7: optimized curve]
    plotBool = [True, True, True, True, True, True, True, True]

    # load measurement by selecting it during runtime in the folder
    # Alternatively or if errors occur: set path directly to the file, e.g.: "measurementData/parable3D_WNOTRS_3D_P_10_NoiseT_0.1_NoiseR_5_NoiseS_1_001.txt"
    path = "measurementData/"
    md = loadMeasurement(path)

    # calculate control points = algorithm for optimized control point estimation
    b23EstOpt = estOptControlPoints(md[0][0][0:3, 3], md[0][-1][0:3, 3], md[0], md[5], md[3])

    # Plot
    plot3D(md, b23EstOpt, plotBool)
