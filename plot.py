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
    fig, axs = plt.subplots(4, 2, sharex=True)
    fig.tight_layout()

    print(path)

    # PLotting


# -----------------------------------------------------------------------------------------------------
# Have main at the bottom call so functions are all declared
# -----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()

    plt.show()
