#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test.py guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd edition, John Wiley & Sons, Chichester, UK. 
URL: https://www.fossen.biz/wiley
    
Author:     Thor I. Fossen
"""
import os
import time
import webbrowser
import matplotlib.pyplot as plt
from src.python_vehicle_simulator.vehicles import *
from src.python_vehicle_simulator.lib import *

# Simulation parameters: 
sampleTime = 0.02  # 采样时间 [秒/s]
N = 5000  # 采样数量2000报错

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50  # number of 3D data points
FPS = 10  # frames per second (animated GIF)
filename = '3D_animation.gif'  # data file9 for animated GIF
browser = 'safari'  # browser for visualization of animated GIF

printSimInfo()

no = input("Please enter a vehicle no.: ")

match no:  # The match statement requires Python >= 3.10
    case '1':
        vehicle = remus100('depthHeadingAutopilot', 30, 50, 1525, 0, 0)
    case '2':
        # model_sim run
        vehicle = remus100('mpcDepthControl', 30, 0, 1525, 0, 0)
    case '3':
        # model_sim not run
        vehicle = remus100('mpcDepthControl', 20, 0, 1525, 0, 0)
    case '9':
        vehicle = remus100_raw('depthHeadingAutopilot', 30, 50, 1525, 0.5, 170)
    case _:
        print('Error: Not a valid simulator option'), sys.exit()

# printVehicleinfo(vehicle, sampleTime, N)


###############################################################################
# Main simulation loop 
###############################################################################
def main():
    print("程序开始运行。。。")
    start_time = time.time()
    [simTime, simData] = simulate(N, sampleTime, vehicle)

    end_time = time.time()

    np.savetxt('../../data/data_x.csv', np.hstack((simTime, simData)), delimiter=',')

    plotVehicleStates(simTime, simData, 1)
    plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 3)

    print("耗时: {:.2f}秒".format(end_time - start_time))
    """ Ucomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))

    plt.show()
    plt.close()


main()
