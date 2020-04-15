#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 14 10:44:24 2020

@author: wyw
"""

import numpy as np
import matplotlib.pyplot as plt

points = np.loadtxt("points.txt")

x1 = points[:,0]
y1 = points[:,1]

x2 = points[:,3]
y2 = points[:,4]

x = np.linspace(x1,x2, 10)
y = np.linspace(y1,y2, 10)


 
def CubicSpline(x,y):
    from scipy.interpolate import CubicSpline    
    cs = CubicSpline(x, y)  
    #1-d array containing values of the independent variable. Values must be real, finite and in strictly increasing order.
    xs = np.arange(-0.5, 9.6, 0.1)
    plt.plot(xs, cs(xs), label="S")   
    plt.show()


def Bspline(x,y,i):
    from scipy.interpolate import splprep, splev
    tck, u = splprep([x, y], s=0, per=True)
    unew = np.arange(0,1,0.001)
    newx, newy = splev(unew, tck)

    
    ax.plot(x, y, 'ro')
    ax.plot(newx, newy, 'r-')

    with open('multiwp'+str(i)+'.csv', 'w') as file: 
        for i in range(len(newx)):
            file.write('%f, %f\n' %(newx[i], newy[i]))
    
fig, ax = plt.subplots()

for i in range(10):
    Bspline(x[i,:],y[i,:],i)
plt.show()
fig.savefig('paths.png')