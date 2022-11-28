"""
By: Benjamin Chupik & Guido Insinger

Created: 11/28/2022

Goal:
Main run file for Stat... Final Project part 1 (Deterministic System Analysis)

"""

# -----------------------------------------------------------------------------------------------------
# Imports
# -----------------------------------------------------------------------------------------------------

import numpy as np
from numpy import pi, sin, cos, tan  # want to be able to just use pi
from scipy.linalg import expm

# -----------------------------------------------------------------------------------------------------
# Numpy Print Styling
# -----------------------------------------------------------------------------------------------------
np.set_printoptions(precision=3)  # Limit decimal places
np.set_printoptions(suppress=True)  # True is not scientific notation

# -----------------------------------------------------------------------------------------------------
# Nominal System Parameters
# -----------------------------------------------------------------------------------------------------
# DT Time
dT = 0.1  # [sec]

# UGV (unmaned ground vehicle) Parameters
L = 0.5  # [m]

phi_g_min = -5*pi/12  # [rads]
phi_g_max = 5*pi/12  # [rads]s

v_g_max = 3  # [m/s]

# UGV Initial Conditions
xi_g_0 = 10  # [m?]
eta_g_0 = 0  # [m?]
theta_g_0 = pi/2  # [rad]
v_g_0 = 2  # [m/s]
phi_g_0 = -pi/18  # [rad]


# UAV (unmaned areal vehicle) Parameters
omega_a_min = -pi/6  # [rad/s]
omega_a_max = pi/6  # [rad/s]

v_a_min = 10  # [m/s]
v_a_min = 20  # [m/s]

# UAV Initial Conditions
xi_a_0 = -60  # [m?]
eta_a_0 = 0  # [m?]
theta_a_0 = -pi/2  # [rad]
v_a_0 = 12  # [m/s]
w_a_0 = pi/25  # [rad/s]


# -----------------------------------------------------------------------------------------------------
# 1: Jacobian Matricies
# -----------------------------------------------------------------------------------------------------
# Dont need any code for this problem

# -----------------------------------------------------------------------------------------------------
# 2: CT Linearization about norm
# -----------------------------------------------------------------------------------------------------
A = np.array([[0, 0, -v_g_0*sin(theta_g_0), 0, 0, 0],
              [0, 0, v_g_0*cos(theta_g_0), 0, 0, 0],
              [0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, -v_a_0*sin(theta_a_0)],
              [0, 0, 0, 0, 0, v_a_0*cos(theta_a_0)],
              [0, 0, 0, 0, 0, 0], ])

B = np.array([[cos(theta_g_0), 0, 0, 0],
              [sin(theta_g_0), 0, 0, 0],
              [tan(phi_g_0)/L, v_g_0/cos(phi_g_0)**2, 0, 0],
              [0, 0, cos(theta_a_0), 0],
              [0, 0, sin(theta_a_0), 0],
              [0, 0, 0, 1], ])

print('A:')
print(A)
print('B:')
print(B)

# -----------------------------------------------------------------------------------------------------
# 2: DT Creation
# -----------------------------------------------------------------------------------------------------
FGMatrix = expm(A*dT)  # get the combo matrix [[F, G],[0,I]]
print("FGMatrix:\n", FGMatrix)  # Uncoment to look at the F matrix

# Get F and G matrix (DT) from the combo matrix
F = FGMatrix[0:4, 0:4]
G = FGMatrix[0:4, 4:]

# Print out F and G matrix
print("F:\n", F)
print("G:\n", G)

# -----------------------------------------------------------------------------------------------------
# 3:
# -----------------------------------------------------------------------------------------------------
