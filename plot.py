"""
By: Benjamin Chupik

Plots the generated path from the main.cpp run


Data file structure:
    [x_pos, y_pos, z_pos, x_quat, y_quat, z_quat, w_quat]

"""

import numpy as np
import matplotlib.pyplot as plt


def main():
    # import path
    path = np.genfromtxt("./build/Debug/OutputPath.data", delimiter=" ")

    # Plotting
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    fig.tight_layout()

    ax.plot3D(path[:, 0], path[:, 1], path[:, 2])
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
