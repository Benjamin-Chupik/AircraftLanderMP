"""
By: Benjamin Chupik

Plots the generated path from the main.cpp run


Data file structure:
    Gemetric File
    [x, y, z, roll, pitch, yaw , x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, t]

    Control File
    [x, y, z, roll, pitch, yaw , x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, d_e, d_a, d_r, thrust, t, conrol duration]

"""

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from numpy import sqrt

isSave = True

dt = 0.1  # From the C++ stuff


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def main():
    # import path
    geoPath = np.genfromtxt("./build/Debug/OutputPath_geo.data", delimiter=" ")
    contPath = np.genfromtxt(
        "./build/Debug/OutputPath_cont.data", delimiter=" ")
    x = geoPath[:, 0]
    y = geoPath[:, 1]
    z = -geoPath[:, 2]  # NED

    yaw = geoPath[:, 5]
    pitch = geoPath[:, 4]
    roll = geoPath[:, 3]

    x_dot = geoPath[:, 6]
    y_dot = geoPath[:, 7]
    z_dot = geoPath[:, 8]

    roll_dot = geoPath[:, 9]
    pitch_dot = geoPath[:, 10]
    yaw_dot = geoPath[:, 11]

    t = geoPath[:, 12]

    de = contPath[:, 13]
    da = contPath[:, 14]
    dr = contPath[:, 15]
    thrust = contPath[:, 16]

    #t = np.arange(0, 0.1*np.shape(x)[0], 0.1)

    t_cont = np.cumsum(contPath[:, 17])

    nData = np.shape(x)[0]

    # -----------------------------------------------------------------------------------
    # Plot Position and velocity map
    # -----------------------------------------------------------------------------------
    # get color map for velocity
    geo_velocity = sqrt(x_dot**2+y_dot**2+z_dot**2)

    # Plotting
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    fig.tight_layout()

    ax.plot(x, y, z, color='red')
    sc = ax.scatter(x, y, z, cmap='hot', c=geo_velocity)
    ax.scatter(x[0], y[0], z[0], c='blue')
    ax.scatter(x[-1], y[-1], z[-1], c='green')
    ax.set_xlabel("x Position")
    ax.set_ylabel("y Position")
    ax.set_zlabel("z Position")
    set_axes_equal(ax)
    fig.colorbar(sc, label='Velocity')
    if isSave:
        fig.canvas.manager.full_screen_toggle()  # toggle fullscreen mode
        plt.savefig("3DPlot.png")

    # Position Plots

    fig, axs = plt.subplots(3)
    fig.tight_layout()
    axs[0].plot(t, x)
    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("x Position")
    axs[0].set_title("x Position")

    axs[1].plot(t, y)
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("y Position")
    axs[1].set_title("y Position")

    axs[2].plot(t, z)
    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("z Position")
    axs[2].set_title("z Position")
    if isSave:
        fig.canvas.manager.full_screen_toggle()  # toggle fullscreen mode
        plt.savefig("PositionPlots.png")

    # -----------------------------------------------------------------------------------
    # Plot Angles
    # -----------------------------------------------------------------------------------
    fig, axs = plt.subplots(3)
    fig.tight_layout()
    axs[0].plot(t, np.rad2deg(roll))
    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("Roll Angle [deg]")
    axs[0].set_title("Roll Angle")

    axs[1].plot(t, np.rad2deg(pitch))
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Pitch Angle [deg]")
    axs[1].set_title("Pitch Angle")

    axs[2].plot(t, np.rad2deg(yaw))
    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("Yaw Angle[deg]")
    axs[2].set_title("Yaw Angle")
    if isSave:
        fig.canvas.manager.full_screen_toggle()  # toggle fullscreen mode
        plt.savefig("AttitudePlots.png")

    # -----------------------------------------------------------------------------------
    # Plot controls
    # -----------------------------------------------------------------------------------
    fig, axs = plt.subplots(4)
    fig.tight_layout()
    axs[0].plot(t_cont, de)
    axs[0].set_xlabel("Time [s]")
    axs[0].set_ylabel("Elevator Control")
    axs[0].set_title("Elevator Control")

    axs[1].plot(t_cont, da)
    axs[1].set_xlabel("Time [s]")
    axs[1].set_ylabel("Aileron Control")
    axs[1].set_title("Aileron Control")

    axs[2].plot(t_cont, dr)
    axs[2].set_xlabel("Time [s]")
    axs[2].set_ylabel("Rudder Control")
    axs[2].set_title("Rudder Control")

    axs[3].plot(t_cont, thrust)
    axs[3].set_xlabel("Time [s]")
    axs[3].set_ylabel("Thrust Control")
    axs[3].set_title("Thrust Control")
    if isSave:
        fig.canvas.manager.full_screen_toggle()  # toggle fullscreen mode
        plt.savefig("ControlInputsPlots.png")


# -----------------------------------------------------------------------------------------------------
# Have main at the bottom call so functions are all declared
# -----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    plt.style.use('ggplot')

    main()

    plt.show()
