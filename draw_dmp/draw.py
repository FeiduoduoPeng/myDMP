'''
@Author: Boris.Peng
@Date: 2020-06-19 15:31:19
@LastEditors: Boris.Peng
@LastEditTime: 2020-06-19 16:41:07
'''
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d

import numpy as np
import re


demo = np.loadtxt("19.txt")
res = np.loadtxt("res.txt")

fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot3D(
    [row[4] for row in demo],
    [row[5] for row in demo],
    [row[6] for row in demo],
    'red')

ax.plot3D(
    [row[4] for row in res],
    [row[5] for row in res],
    [row[6] for row in res],
    'green')
plt.show()