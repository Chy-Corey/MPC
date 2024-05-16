#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd edition, John Wiley & Sons, Chichester, UK. 
URL: https://www.fossen.biz/wiley
    
Author:     Thor I. Fossen
"""
import os
import webbrowser
import matplotlib.pyplot as plt
from src.python_vehicle_simulator.vehicles import *
from src.python_vehicle_simulator.lib import *

# Simulation parameters: 
sampleTime = 0.02  # 采样时间 [秒/s]
N = 50000  # 采样数量

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50  # number of 3D data points
FPS = 10  # frames per second (animated GIF)
filename = '3D_animation.gif'  # data file9 for animated GIF
browser = 'safari'  # browser for visualization of animated GIF

###############################################################################
# Vehicle constructors
###############################################################################
printSimInfo()

"""
DSRV('depthAutopilot',z_d)                                       
frigate('headingAutopilot',U,psi_d)
otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)                  
ROVzefakkel('headingAutopilot',U,psi_d)                          
semisub('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                      
shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)  
supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)      
tanker('headingAutopilot',psi_d,V_c,beta_c,depth)    
remus100('depthHeadingAutopilot',z_d,psi_d,V_c,beta_c)             

Call constructors without arguments to test step inputs, e.g. DSRV(), otter(), etc. 
"""

no = input("Please enter a vehicle no.: ")

match no:  # The match statement requires Python >= 3.10
    case '1':
        vehicle = remus100('depthHeadingAutopilot', 30, 50, 1525, 0, 0)
    case '9':
        vehicle = remus100_raw('depthHeadingAutopilot', 30, 50, 1525, 0.5, 170)
    case _:
        print('Error: Not a valid simulator option'), sys.exit()

printVehicleinfo(vehicle, sampleTime, N)


###############################################################################
# Main simulation loop 
###############################################################################
def main():
    [simTime, simData, model_SimData] = simulate(N, sampleTime, vehicle)

    plotVehicleStates(simTime, simData, 1)
    plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 3)
    plotModelStates(simTime, simData, model_SimData, 4)

    """ Ucomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    # webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))

    plt.show()
    plt.close()


main()
