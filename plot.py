"""
By: Benjamin Chupik

Plots the generated path from the main.cpp run


Data file structure:
    Gemetric File
    [x_pos, y_pos, z_pos, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot]

    Control File
    [x_pos, y_pos, z_pos, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot, d_e, d_a, d_r, thrust, conrol duration]

"""

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from numpy import sqrt


def main():
    # import path
    geoPath = np.genfromtxt("./build/Debug/OutputPath_geo.data", delimiter=" ")
    contPath = np.genfromtxt(
        "./build/Debug/OutputPath_cont.data", delimiter=" ")
    x = geoPath[:, 0]
    y = geoPath[:, 1]
    z = -geoPath[:, 2]  # NED

    roll = geoPath[:, 3]
    pitch = geoPath[:, 4]
    yaw = geoPath[:, 5]

    x_dot = geoPath[:, 6]
    y_dot = geoPath[:, 7]
    z_dot = geoPath[:, 8]

    roll_dot = geoPath[:, 9]
    pitch_dot = geoPath[:, 10]
    yaw_dot = geoPath[:, 11]

    # get color map for velocity
    geo_velocity = sqrt(x_dot**2+y_dot**2+z_dot**2)

    # Plotting
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    fig.tight_layout()

    ax.plot(x, y, z)
    sc = ax.scatter(x, y, z, cmap='hot', c=geo_velocity)
    ax.set_xlabel("x Position")
    ax.set_ylabel("y Position")
    ax.set_zlabel("z Position")

    fig.colorbar(sc, label='Velocity')


# -----------------------------------------------------------------------------------------------------
# Have main at the bottom call so functions are all declared
# -----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()

    plt.show()
