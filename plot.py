"""
By: Benjamin Chupik

Plots the generated path from the main.cpp run

"""

import pandas as pd
import numpy as np

# import path
path = np.genfromtxt("./build/Debug/OutputPath.data", delimiter=" ")

# Delete last colum becuse of extra added space

print(path)
