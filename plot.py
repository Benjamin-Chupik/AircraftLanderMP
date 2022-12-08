"""
By: Benjamin Chupik

Plots the generated path from the main.cpp run


Data file structure:
    [x_pos, y_pos, z_pos, roll, pitch, yaw, x_dot, y_dot, z_dot, roll_dot, pitch_dot, yaw_dot]

"""

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from numpy import sqrt


def main():
    # import path
    path = np.genfromtxt("./build/Debug/OutputPath.data", delimiter=" ")
    x = path[:, 0]
    y = path[:, 1]
    z = path[:, 2]

    roll = path[:, 3]
    pitch = path[:, 4]
    yaw = path[:, 5]

    x_dot = path[:, 6]
    y_dot = path[:, 7]
    z_dot = path[:, 8]

    roll_dot = path[:, 9]
    pitch_dot = path[:, 10]
    yaw_dot = path[:, 11]

    # Plotting
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    fig.tight_layout()

    # get color map for velocity
    col = sqrt(x_dot**2+y_dot**2+z_dot**2)

    ax.plot3D(x, y, z, c=col)
    ax.set_xlabel("x Position")
    ax.set_ylabel("y Position")
    ax.set_zlabel("z Position")

    print(path)

    # PLotting


# -----------------------------------------------------------------------------------------------------
# Have main at the bottom call so functions are all declared
# -----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()

    plt.show()
