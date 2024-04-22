#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main simulation loop called by main.py.

Author:     Thor I. Fossen
"""

import numpy as np
from .gnc import attitudeEuler


###############################################################################
# Function printSimInfo(vehicle)
###############################################################################
def printSimInfo():
    """
    Constructors used to define the vehicle objects as (see main.py for details):
        DSRV('depthAutopilot',z_d)                                       
        frigate('headingAutopilot',U,psi_d)
        otter('headingAutopilot',psi_d,V_c,beta_c,tau_X)                  
        ROVzefakkel('headingAutopilot',U,psi_d)                          
        semisub('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)                       
        shipClarke83('headingAutopilot',psi_d,L,B,T,Cb,V_c,beta_c,tau_X)  
        supply('DPcontrol',x_d,y_d,psi_d,V_c,beta_c)      
        tanker('headingAutopilot',psi_d,V_c,beta_c,depth)    
        remus100('depthHeadingAutopilot',z_d,psi_d,V_c,beta_c)
    """

    print('---------------------------------------------------------------------------------------')
    print('The Python Vehicle Simulator')
    print('---------------------------------------------------------------------------------------')
    print('1 - Deep submergence rescue vehicle (DSRV): controlled by a stern plane, L = 5.0 m')
    print('2 - Frigate: rudder-controlled ship described by a nonlinear Nomoto model, L = 100.0 m')
    print('3 - Otter unmanned surface vehicle (USV): controlled by two propellers, L = 2.0 m')
    print('4 - ROV Zefakkel: rudder-controlled ship described by a nonlinear Nomoto model, L = 54.0 m')
    print('5 - Semisubmersible: controlled by tunnel thrusters and main propellers, L = 84.5 m')
    print('6 - Ship: linear maneuvering model specified by L, B and T using the Clarke (1983) formulas')
    print('7 - Offshore supply vessel: controlled by tunnel thrusters and main propellers, L = 76.2 m')
    print('8 - Tanker: rudder-controlled ship model including shallow water effects, L = 304.8 m')
    print('9 - Remus 100: AUV controlled by stern planes, a tail rudder and a propeller, L = 1.6 m')
    print('---------------------------------------------------------------------------------------')


###############################################################################    
# Function printVehicleinfo(vehicle)
###############################################################################
def printVehicleinfo(vehicle, sample_time, N):
    print('---------------------------------------------------------------------------------------')
    print('%s' % vehicle.name)
    print('Length: %s m' % vehicle.L)
    print('%s' % vehicle.controlDescription)
    print('Sampling frequency: %s Hz' % round(1 / sample_time))
    print('Simulation time: %s seconds' % round(N * sample_time))
    print('---------------------------------------------------------------------------------------')


###############################################################################
# Function simulate(N, sample_time, vehicle)
###############################################################################
def simulate(n, sample_time, vehicle):
    DOF = 6  # degrees of freedom 六个自由度：x，y，z，phi(p, roll)，theta(q, pitch)，psi(r, yaw)
    t = 0  # initial simulation time

    # Initial state vectors
    eta = np.array([0, 0, 0, 0, 0, 0], float)  # position/attitude, user editable；x，y，z，phi，theta，psi
    nu = vehicle.nu  # velocity, defined by vehicle class 速度，eta 的导数
    u_actual = vehicle.u_actual  # actual inputs, defined by vehicle class，控制输入，阶数由 vehicle 决定

    # Initialization of table used to store the simulation data
    # 初始化仿真数据维数，自由度 × 2 表示状态和状态的导数，控制输入 × 2 表示控制输入和执行器输出
    simData = np.empty([0, 2 * DOF + 2 * vehicle.dimU], float)

    # Simulator for-loop
    for i in range(0, n + 1):

        t = i * sample_time  # simulation time

        # Vehicle specific control systems
        if vehicle.controlMode == 'depthAutopilot':
            u_control = vehicle.depthAutopilot(eta, nu, sample_time)
        elif vehicle.controlMode == 'headingAutopilot':
            u_control = vehicle.headingAutopilot(eta, nu, sample_time)
        elif vehicle.controlMode == 'depthHeadingAutopilot':
            u_control = vehicle.depthHeadingAutopilot(eta, nu, sample_time)
        elif vehicle.controlMode == 'DPcontrol':
            u_control = vehicle.DPcontrol(eta, nu, sample_time)
        elif vehicle.controlMode == 'stepInput':
            u_control = vehicle.stepInput(t)

            # Store simulation data in simData
        signals = np.append(np.append(np.append(eta, nu), u_control), u_actual)
        simData = np.vstack([simData, signals])

        # Propagate vehicle and attitude dynamics
        [nu, u_actual] = vehicle.dynamics(eta, nu, u_actual, u_control, sample_time)
        eta = attitudeEuler(eta, nu, sample_time)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t + sample_time, step=sample_time)[:, None]

    return simTime, simData