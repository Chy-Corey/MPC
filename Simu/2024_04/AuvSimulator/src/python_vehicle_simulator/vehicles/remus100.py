#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
remus100.py:

   Class for the Remus 100 cylinder-shaped autonomous underwater vehicle (AUV),
   which is controlled using a tail rudder, stern planes and a propeller. The
   length of the AUV is 1.6 m, the cylinder diameter is 19 cm and the
   mass of the vehicle is 31.9 kg. The maximum speed of 2.5 m/s is obtained
   when the propeller runs at 1525 rpm in zero currents.

   remus100()
       Step input, stern plane, rudder and propeller revolution

    remus100('depthHeadingAutopilot',z_d,psi_d,n_d,V_c,beta_c)
        z_d:    desired depth (m), positive downwards
        psi_d:  desired yaw angle (deg)
        n_d:    desired propeller revolution (rpm)
        V_c:    current speed (m/s)
        beta_c: current direction (deg)

Methods:

    [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime ) returns
        nu[k+1] and u_actual[k+1] using Euler's method. The control input is:

            u_control = [ delta_r   rudder angle (rad)
                         delta_s    stern plane angle (rad)
                         n          propeller revolution (rpm) ]

    u = depthHeadingAutopilot(eta,nu,sampleTime)
        Simultaneously control of depth and heading using controllers of
        PID and SMC ype. Propeller rpm is given as a step command.

    u = stepInput(t) generates tail rudder, stern planes and RPM step inputs.

References:

    B. Allen, W. S. Vorus and T. Prestero, "Propulsion system performance
         enhancements on REMUS AUVs," OCEANS 2000 MTS/IEEE Conference and
         Exhibition. Conference Proceedings, 2000, pp. 1869-1873 vol.3,
         doi: 10.1109/OCEANS.2000.882209.
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion
         Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley

Author:     Thor I. Fossen
"""
import numpy as np
import math
import sys
from src.python_vehicle_simulator.lib.control import integralSMC
from src.python_vehicle_simulator.lib.gnc import crossFlowDrag, forceLiftDrag, Hmtrx, m2c, gvect, ssa
import casadi as ca
import time


# Class Vehicle
class remus100:
    """
    remus100()
        Rudder angle, stern plane and propeller revolution step inputs

    remus100('depthHeadingAutopilot',z_d,psi_d,n_d,V_c,beta_c)
        Depth and heading autopilots

    Inputs:
        z_d:    desired depth, positive downwards (m)
        psi_d:  desired heading angle (deg)
        n_d:    desired propeller revolution (rpm)
        V_c:    current speed (m/s)
        beta_c: current direction (deg)
    """

    def __init__(
            self,
            controlSystem="stepInput",
            r_z=0,
            r_psi=0,
            r_rpm=0,
            V_current=0,
            beta_current=0,
    ):

        # Constants
        self.D2R = math.pi / 180  # deg2rad
        self.rho = 1026  # density of water (kg/m^3)
        g = 9.81  # acceleration of gravity (m/s^2)

        if controlSystem == "depthHeadingAutopilot":
            self.controlDescription = (
                    "Depth and heading autopilots, z_d = "
                    + str(r_z)
                    + ", psi_d = "
                    + str(r_psi)
                    + " deg"
            )

        elif controlSystem == "mpcDepthControl":
            self.controlDescription = (
                    "Depth autopilots, z_d = "
                    + str(r_z)
            )

        else:
            self.controlDescription = (
                "Step inputs for stern planes, rudder and propeller")
            controlSystem = "stepInput"

        self.ref_z = r_z
        self.ref_psi = r_psi
        self.ref_n = r_rpm
        self.V_c = V_current
        self.beta_c = beta_current * self.D2R
        self.controlMode = controlSystem

        # Initialize the AUV model
        self.name = (
            "Remus 100 cylinder-shaped AUV (see 'remus100_raw.py' for more details)")
        self.L = 1.6  # length (m)
        self.diam = 0.19  # cylinder diameter (m)

        self.nu = np.array([0, 0, 0, 0, 0, 0], float)  # velocity vector
        self.u_actual = np.array([0, 0, 0], float)  # control input vector

        self.controls = [
            "Tail rudder (deg)",
            "Stern plane (deg)",
            "Propeller revolution (rpm)"
        ]
        self.dimU = len(self.controls)

        # Actuator dynamics
        self.deltaMax_r = 15 * self.D2R  # max rudder angle (rad)
        self.deltaMax_s = 15 * self.D2R  # max stern plane angle (rad)
        self.nMax = 1525  # max propeller revolution (rpm)
        self.T_delta = 0.1  # rudder/stern plane time constant (s)
        self.T_n = 0.1  # propeller time constant (s)

        if r_rpm < 0.0 or r_rpm > self.nMax:
            print("The RPM value should be in the interval 0 - %s", self.nMax)
            sys.exit()

        if r_z > 100.0 or r_z < 0.0:
            sys.exit('desired depth must be between 0-100 m')

            # Hydrodynamics (Fossen 2021, Section 8.4.2)
        self.S = 0.7 * self.L * self.diam  # S = 70% of rectangle L * diam
        a = self.L / 2  # semi-axes
        b = self.diam / 2
        # self.r_bg = np.array([0, 0, 0.02], float)  # CG w.r.t. to the CO
        self.r_bg = np.array([0, 0, 0.02], float)  # CG w.r.t. to the CO
        self.r_bb = np.array([0, 0, 0], float)  # CB w.r.t. to the CO

        # Parasitic drag coefficient CD_0, i.e. zero lift and alpha = 0
        Cd = 0.42  # from Allen et al. (2000)
        self.CD_0 = Cd * math.pi * b ** 2 / self.S

        # Rigid-body mass matrix expressed in CO
        m = 4 / 3 * math.pi * self.rho * a * b ** 2  # mass of spheriod
        Ix = (2 / 5) * m * b ** 2  # moment of inertia
        Iy = (1 / 5) * m * (a ** 2 + b ** 2)
        Iz = Iy
        MRB_CG = np.diag([m, m, m, Ix, Iy, Iz])  # MRB expressed in the CG
        H_rg = Hmtrx(self.r_bg)
        self.MRB = H_rg.T @ MRB_CG @ H_rg  # MRB expressed in the CO

        # Weight and buoyancy
        self.W = m * g
        self.B = self.W

        # Added moment of inertia in roll: A44 = r44 * Ix
        r44 = 0.3
        MA_44 = r44 * Ix

        # Lamb's k-factors
        e = math.sqrt(1 - (b / a) ** 2)
        alpha_0 = (2 * (1 - e ** 2) / pow(e, 3)) * (0.5 * math.log((1 + e) / (1 - e)) - e)
        beta_0 = 1 / (e ** 2) - (1 - e ** 2) / (2 * pow(e, 3)) * math.log((1 + e) / (1 - e))

        k1 = alpha_0 / (2 - alpha_0)
        k2 = beta_0 / (2 - beta_0)
        k_prime = pow(e, 4) * (beta_0 - alpha_0) / (
                (2 - e ** 2) * (2 * e ** 2 - (2 - e ** 2) * (beta_0 - alpha_0)))

        # Added mass system matrix expressed in the CO
        self.MA = np.diag([m * k1, m * k2, m * k2, MA_44, k_prime * Iy, k_prime * Iy])

        # Mass matrix including added mass
        self.M = self.MRB + self.MA
        self.Minv = np.linalg.inv(self.M)

        # Natural frequencies in roll and pitch
        self.w_roll = math.sqrt(self.W * (self.r_bg[2] - self.r_bb[2]) /
                                self.M[3][3])
        self.w_pitch = math.sqrt(self.W * (self.r_bg[2] - self.r_bb[2]) /
                                 self.M[4][4])

        S_fin = 0.00665  # fin area

        # Tail rudder parameters
        self.CL_delta_r = 0.5  # rudder lift coefficient
        self.A_r = 2 * S_fin  # rudder area (m2)
        self.x_r = -a  # rudder x-position (m)

        # Stern-plane parameters (double)
        self.CL_delta_s = 0.7  # stern-plane lift coefficient
        self.A_s = 2 * S_fin  # stern-plane area (m2)
        self.x_s = -a  # stern-plane z-position (m)

        # Low-speed linear damping matrix parameters
        self.T_surge = 20  # time constant in surge (s)
        self.T_sway = 20  # time constant in sway (s)
        self.T_heave = self.T_sway  # equal for a cylinder-shaped AUV
        self.zeta_roll = 0.3  # relative damping ratio in roll
        self.zeta_pitch = 0.8  # relative damping ratio in pitch
        self.T_yaw = 1  # time constant in yaw (s)

        # Feed forward gains (Nomoto gain parameters)
        self.K_nomoto = 5.0 / 20.0  # K_nomoto = r_max / delta_max
        self.T_nomoto = self.T_yaw  # Time constant in yaw

        # Heading autopilot reference model
        self.psi_d = 0  # position, velocity and acc. states
        self.r_d = 0
        self.a_d = 0
        self.wn_d = 0.1  # desired natural frequency
        self.zeta_d = 1  # desired realtive damping ratio
        self.r_max = 5.0 * math.pi / 180  # maximum yaw rate

        # Heading autopilot (Equation 16.479 in Fossen 2021)
        # sigma = r-r_d + 2*lambda*ssa(psi-psi_d) + lambda^2 * integral(ssa(psi-psi_d))
        # delta = (T_nomoto * r_r_dot + r_r - K_d * sigma
        #       - K_sigma * (sigma/phi_b)) / K_nomoto
        self.lam = 0.1
        self.phi_b = 0.1  # boundary layer thickness
        self.K_d = 0.5  # PID gain
        self.K_sigma = 0.05  # SMC switching gain

        self.e_psi_int = 0  # yaw angle error integral state

        # Depth autopilot
        self.wn_d_z = 0.02  # desired natural frequency, reference model
        self.Kp_z = 0.1  # heave proportional gain, outer loop
        self.T_z = 100.0  # heave integral gain, outer loop
        self.Kp_theta = 5.0  # pitch PID controller
        self.Kd_theta = 2.0
        self.Ki_theta = 0.3
        self.K_w = 5.0  # optional heave velocity feedback gain

        self.z_int = 0  # heave position integral state
        self.z_d = 0  # desired position, LP filter initial state
        self.theta_int = 0  # pitch angle integral state

        # MPC
        self.Np = 60  # 预测步长
        self.Nc = 60  # 控制步长

    def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
        """
        [nu,u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) integrates
        the AUV equations of motion using Euler's method.
        """

        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float)  # current velocity
        Dnu_c = np.array([nu[5] * v_c, -nu[5] * u_c, 0, 0, 0, 0], float)  # derivative
        nu_r = nu - nu_c  # relative velocity
        alpha = math.atan2(nu_r[2], nu_r[0])  # angle of attack
        U = math.sqrt(nu[0] ** 2 + nu[1] ** 2 + nu[2] ** 2)  # vehicle speed
        U_r = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2 + nu_r[2] ** 2)  # relative speed

        # Commands and actual control signals
        delta_r_c = u_control[0]  # commanded tail rudder (rad)
        delta_s_c = u_control[1]  # commanded stern plane (rad)

        n_c = u_control[2]  # commanded propeller revolution (rpm)

        delta_r = u_actual[0]  # actual tail rudder (rad)
        delta_s = u_actual[1]  # actual stern plane (rad)
        n = u_actual[2]  # actual propeller revolution (rpm)

        # Propeller coeffs. KT and KQ are computed as a function of advance no.
        # Ja = Va/(n*D_prop) where Va = (1-w)*U = 0.944 * U; Allen et al. (2000)
        D_prop = 0.14  # propeller diameter corresponding to 5.5 inches
        t_prop = 0.1  # thrust deduction number
        n_rps = n / 60  # propeller revolution (rps)
        Va = 0.944 * U  # advance speed (m/s)

        # Ja_max = 0.944 * 2.5 / (0.14 * 1525/60) = 0.6632
        Ja_max = 0.6632

        # Single-screw propeller with 3 blades and blade-area ratio = 0.718.
        # Coffes. are computed using the Matlab MSS toolbox:
        # >> [KT_0, KQ_0] = wageningen(0,1,0.718,3)
        KT_0 = 0.4566
        KQ_0 = 0.0700
        # >> [KT_max, KQ_max] = wageningen(0.6632,1,0.718,3)
        KT_max = 0.1798
        KQ_max = 0.0312

        # Propeller thrust and propeller-induced roll moment
        # Linear approximations for positive Ja values
        # KT ~= KT_0 + (KT_max-KT_0)/Ja_max * Ja
        # KQ ~= KQ_0 + (KQ_max-KQ_0)/Ja_max * Ja

        if n_rps > 0:  # forward thrust

            X_prop = self.rho * pow(D_prop, 4) * (
                    KT_0 * abs(n_rps) * n_rps + (KT_max - KT_0) / Ja_max *
                    (Va / D_prop) * abs(n_rps))
            K_prop = self.rho * pow(D_prop, 5) * (
                    KQ_0 * abs(n_rps) * n_rps + (KQ_max - KQ_0) / Ja_max *
                    (Va / D_prop) * abs(n_rps))

        else:  # reverse thrust (braking)

            X_prop = self.rho * pow(D_prop, 4) * KT_0 * abs(n_rps) * n_rps
            K_prop = self.rho * pow(D_prop, 5) * KQ_0 * abs(n_rps) * n_rps

            # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
        CRB = m2c(self.MRB, nu_r)
        CA = m2c(self.MA, nu_r)

        # CA-terms in roll, pitch and yaw can destabilize the model if quadratic
        # rotational damping is missing. These terms are assumed to be zero
        CA[4][0] = 0  # Quadratic velocity terms due to pitching
        CA[0][4] = 0
        CA[4][2] = 0
        CA[2][4] = 0
        CA[5][0] = 0  # Munk moment in yaw
        CA[0][5] = 0
        CA[5][1] = 0
        CA[1][5] = 0

        C = CRB + CA

        # Dissipative forces and moments
        D = np.diag([
            self.M[0][0] / self.T_surge,
            self.M[1][1] / self.T_sway,
            self.M[2][2] / self.T_heave,
            self.M[3][3] * 2 * self.zeta_roll * self.w_roll,
            self.M[4][4] * 2 * self.zeta_pitch * self.w_pitch,
            self.M[5][5] / self.T_yaw
        ])

        # Linear surge and sway damping
        D[0][0] = D[0][0] * math.exp(-3 * U_r)  # vanish at high speed where quadratic
        D[1][1] = D[1][1] * math.exp(-3 * U_r)  # drag and lift forces dominates

        tau_liftdrag = forceLiftDrag(self.diam, self.S, self.CD_0, alpha, U_r)
        tau_crossflow = crossFlowDrag(self.L, self.diam, self.diam, nu_r)

        # Restoring forces and moments
        g = gvect(self.W, self.B, eta[4], eta[3], self.r_bg, self.r_bb)
        # Horizontal- and vertical-plane relative speed
        U_rh = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2)
        U_rv = math.sqrt(nu_r[0] ** 2 + nu_r[2] ** 2)

        # Rudder and stern-plane drag
        X_r = -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r * delta_r ** 2
        X_s = -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s * delta_s ** 2

        # Rudder sway force
        Y_r = -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r * delta_r

        # Stern-plane heave force
        Z_s = -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s * delta_s

        # Generalized force vector
        tau = np.array([
            (1 - t_prop) * X_prop + X_r + X_s,
            Y_r,
            Z_s,
            K_prop / 10,  # scaled down by a factor of 10 to match exp. results
            self.x_s * Z_s,
            self.x_r * Y_r
        ], float)

        # AUV dynamics
        tau_sum = tau + tau_liftdrag + tau_crossflow - np.matmul(C + D, nu_r) - g
        nu_dot = Dnu_c + np.matmul(self.Minv, tau_sum)

        # Actuator dynamics
        delta_r_dot = (delta_r_c - delta_r) / self.T_delta
        delta_s_dot = (delta_s_c - delta_s) / self.T_delta
        n_dot = (n_c - n) / self.T_n

        # Forward Euler integration [k+1]
        nu += sampleTime * nu_dot
        delta_r += sampleTime * delta_r_dot
        delta_s += sampleTime * delta_s_dot
        n += sampleTime * n_dot
        # Amplitude saturation of the control signals
        if abs(delta_r) >= self.deltaMax_r:
            delta_r = np.sign(delta_r) * self.deltaMax_r

        if abs(delta_s) >= self.deltaMax_s:
            delta_s = np.sign(delta_s) * self.deltaMax_s

        if abs(n) >= self.nMax:
            n = np.sign(n) * self.nMax

        u_actual = np.array([delta_r, delta_s, n], float)
        return nu, u_actual

    def stateModel_6DOF(self, eta, nu, u_actual, u_control, sampleTime):
        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float)  # current velocity
        Dnu_c = np.array([nu[5] * v_c, -nu[5] * u_c, 0, 0, 0, 0], float)  # derivative
        nu_r = nu - nu_c  # relative velocity
        alpha = math.atan2(nu_r[2], nu_r[0])  # angle of attack
        U = math.sqrt(nu[0] ** 2 + nu[1] ** 2 + nu[2] ** 2)  # vehicle speed
        U_r = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2 + nu_r[2] ** 2)  # relative speed

        # Commands and actual control signals
        delta_r_c = u_control[0]  # commanded tail rudder (rad)
        delta_s_c = u_control[1]  # commanded stern plane (rad)

        n_c = u_control[2]  # commanded propeller revolution (rpm)

        delta_r = u_actual[0]  # actual tail rudder (rad)
        delta_s = u_actual[1]  # actual stern plane (rad)
        n = u_actual[2]  # actual propeller revolution (rpm)

        # Propeller coeffs. KT and KQ are computed as a function of advance no.
        # Ja = Va/(n*D_prop) where Va = (1-w)*U = 0.944 * U; Allen et al. (2000)
        D_prop = 0.14  # propeller diameter corresponding to 5.5 inches
        t_prop = 0.0  # thrust deduction number
        n_rps = n / 60  # propeller revolution (rps)
        Va = 0.944 * U  # advance speed (m/s)

        # Ja_max = 0.944 * 2.5 / (0.14 * 1525/60) = 0.6632
        Ja_max = 0.6632

        # Single-screw propeller with 3 blades and blade-area ratio = 0.718.
        # Coffes. are computed using the Matlab MSS toolbox:
        # >> [KT_0, KQ_0] = wageningen(0,1,0.718,3)
        KT_0 = 0.4566
        KQ_0 = 0.0700
        # >> [KT_max, KQ_max] = wageningen(0.6632,1,0.718,3)
        KT_max = 0.1798
        KQ_max = 0.0312

        # Propeller thrust and propeller-induced roll moment
        # Linear approximations for positive Ja values
        # KT ~= KT_0 + (KT_max-KT_0)/Ja_max * Ja
        # KQ ~= KQ_0 + (KQ_max-KQ_0)/Ja_max * Ja

        if n_rps > 0:  # forward thrust

            X_prop = self.rho * pow(D_prop, 4) * (
                    KT_0 * abs(n_rps) * n_rps + (KT_max - KT_0) / Ja_max *
                    (Va / D_prop) * abs(n_rps))
            K_prop = self.rho * pow(D_prop, 5) * (
                    KQ_0 * abs(n_rps) * n_rps + (KQ_max - KQ_0) / Ja_max *
                    (Va / D_prop) * abs(n_rps))

        else:  # reverse thrust (braking)

            X_prop = self.rho * pow(D_prop, 4) * KT_0 * abs(n_rps) * n_rps
            K_prop = self.rho * pow(D_prop, 5) * KQ_0 * abs(n_rps) * n_rps

            # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
        CRB = m2c(self.MRB, nu_r)
        CA = m2c(self.MA, nu_r)

        # CA-terms in roll, pitch and yaw can destabilize the model if quadratic
        # rotational damping is missing. These terms are assumed to be zero
        CA[4][0] = 0  # Quadratic velocity terms due to pitching
        CA[0][4] = 0
        CA[4][2] = 0
        CA[2][4] = 0
        CA[5][0] = 0  # Munk moment in yaw
        CA[0][5] = 0
        CA[5][1] = 0
        CA[1][5] = 0

        C = CRB + CA

        # Dissipative forces and moments
        D = np.diag([
            self.M[0][0] / self.T_surge,
            self.M[1][1] / self.T_sway,
            self.M[2][2] / self.T_heave,
            self.M[3][3] * 2 * self.zeta_roll * self.w_roll,
            self.M[4][4] * 2 * self.zeta_pitch * self.w_pitch,
            self.M[5][5] / self.T_yaw
        ])

        # Linear surge and sway damping
        D[0][0] = D[0][0] * math.exp(-3 * U_r)  # vanish at high speed where quadratic
        D[1][1] = D[1][1] * math.exp(-3 * U_r)  # drag and lift forces dominates

        # tau_liftdrag = forceLiftDrag(self.diam, self.S, self.CD_0, alpha, U_r)
        # tau_crossflow = crossFlowDrag(self.L, self.diam, self.diam, nu_r)

        # Restoring forces and moments
        g = gvect(self.W, self.B, eta[4], eta[3], self.r_bg, self.r_bb)

        # Horizontal- and vertical-plane relative speed
        U_rh = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2)
        U_rv = math.sqrt(nu_r[0] ** 2 + nu_r[2] ** 2)

        # Rudder and stern-plane drag
        X_r = -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r * delta_r ** 2
        X_s = -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s * delta_s ** 2

        # Rudder sway force
        Y_r = -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r * delta_r

        # Stern-plane heave force
        Z_s = -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s * delta_s

        # Generalized force vector
        tau = np.array([
            (1 - t_prop) * X_prop,  # + X_r + X_s,
            Y_r,
            Z_s,
            K_prop / 10,  # scaled down by a factor of 10 to match exp. results
            self.x_s * Z_s,
            self.x_r * Y_r
        ], float)

        # AUV dynamics
        # tau_sum = tau + tau_liftdrag + tau_crossflow - np.matmul(C + D, nu_r) - g
        # nu_dot = Dnu_c + np.matmul(self.Minv, tau_sum)
        # tau_sum = tau + tau_liftdrag + tau_crossflow
        tau_sum = tau
        nu_dot = -np.matmul(np.matmul(self.Minv, C + D), nu_r) + np.matmul(self.Minv, tau_sum) - np.matmul(self.Minv,
                                                                                                           g) + Dnu_c
        # Actuator dynamics
        delta_r_dot = (delta_r_c - delta_r) / self.T_delta
        delta_s_dot = (delta_s_c - delta_s) / self.T_delta
        n_dot = (n_c - n) / self.T_n

        # Forward Euler integration [k+1]
        nu += sampleTime * nu_dot
        delta_r += sampleTime * delta_r_dot
        delta_s += sampleTime * delta_s_dot
        n += sampleTime * n_dot
        # Amplitude saturation of the control signals
        if abs(delta_r) >= self.deltaMax_r:
            delta_r = np.sign(delta_r) * self.deltaMax_r

        if abs(delta_s) >= self.deltaMax_s:
            delta_s = np.sign(delta_s) * self.deltaMax_s

        if abs(n) >= self.nMax:
            n = np.sign(n) * self.nMax

        u_actual = np.array([delta_r, delta_s, n], float)

        return nu, u_actual

    def depthHeadingAutopilot(self, eta, nu, sampleTime):
        """
        [delta_r, delta_s, n] = depthHeadingAutopilot(eta,nu,sampleTime)
        simultaneously control the heading and depth of the AUV using control
        laws of PID type. Propeller rpm is given as a step command.

        Returns:

            u_control = [ delta_r   rudder angle (rad)
                         delta_s    stern plane angle (rad)
                         n          propeller revolution (rpm) ]

        """
        z = eta[2]  # heave position (depth)
        theta = eta[4]  # pitch angle
        psi = eta[5]  # yaw angle
        w = nu[2]  # heave velocity
        q = nu[4]  # pitch rate
        r = nu[5]  # yaw rate
        e_psi = psi - self.psi_d  # yaw angle tracking error
        e_r = r - self.r_d  # yaw rate tracking error
        z_ref = self.ref_z  # heave position (depth) setpoint
        psi_ref = self.ref_psi * self.D2R  # yaw angle setpoint

        #######################################################################
        # Propeller command
        #######################################################################
        n = self.ref_n

        #######################################################################
        # Depth autopilot (successive loop closure)
        #######################################################################
        # LP filtered desired depth command
        self.z_d = math.exp(-sampleTime * self.wn_d_z) * self.z_d \
                   + (1 - math.exp(-sampleTime * self.wn_d_z)) * z_ref

        # PI controller
        theta_d = self.Kp_z * ((z - self.z_d) + (1 / self.T_z) * self.z_int)
        delta_s = -self.Kp_theta * ssa(theta - theta_d) - self.Kd_theta * q \
                  - self.Ki_theta * self.theta_int - self.K_w * w

        # Euler's integration method (k+1)
        self.z_int += sampleTime * (z - self.z_d)
        self.theta_int += sampleTime * ssa(theta - theta_d)

        #######################################################################
        # Heading autopilot (SMC controller)
        #######################################################################

        wn_d = self.wn_d  # reference model natural frequency
        zeta_d = self.zeta_d  # reference model relative damping factor

        # Integral SMC with 3rd-order reference model
        [delta_r, self.e_psi_int, self.psi_d, self.r_d, self.a_d] = \
            integralSMC(
                self.e_psi_int,
                e_psi, e_r,
                self.psi_d,
                self.r_d,
                self.a_d,
                self.T_nomoto,
                self.K_nomoto,
                wn_d,
                zeta_d,
                self.K_d,
                self.K_sigma,
                self.lam,
                self.phi_b,
                psi_ref,
                self.r_max,
                sampleTime
            )

        u_control = np.array([delta_r, delta_s, n], float)

        return u_control

    def MPC_6DOF(self, eta, nu, sampleTime, u_actual, pre_step):

        # u_actual：当前时刻的控制输入
        # Current velocities
        u_c = self.V_c * math.cos(self.beta_c - eta[5])  # current surge velocity
        v_c = self.V_c * math.sin(self.beta_c - eta[5])  # current sway velocity

        nu_c = np.array([u_c, v_c, 0, 0, 0, 0], float)  # current velocity
        nu_r = nu - nu_c  # relative velocity

        U = math.sqrt(nu[0] ** 2 + nu[1] ** 2 + nu[2] ** 2)  # vehicle speed
        U_r = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2 + nu_r[2] ** 2)  # relative speed

        # Commands and actual control signals
        n = u_actual[2]  # actual propeller revolution (rpm)

        # Propeller coeffs. KT and KQ are computed as a function of advance no.
        # Ja = Va/(n*D_prop) where Va = (1-w)*U = 0.944 * U; Allen et al. (2000)
        D_prop = 0.14  # propeller diameter corresponding to 5.5 inches
        t_prop = 0.0  # thrust deduction number
        n_rps = n / 60  # propeller revolution (rps)
        Va = 0.944 * U  # advance speed (m/s)

        # Ja_max = 0.944 * 2.5 / (0.14 * 1525/60) = 0.6632
        Ja_max = 0.6632

        # Single-screw propeller with 3 blades and blade-area ratio = 0.718.
        # Coffes. are computed using the Matlab MSS toolbox:
        # >> [KT_0, KQ_0] = wageningen(0,1,0.718,3)
        KT_0 = 0.4566
        KQ_0 = 0.0700
        # >> [KT_max, KQ_max] = wageningen(0.6632,1,0.718,3)
        KT_max = 0.1798
        KQ_max = 0.0312

        # Propeller thrust and propeller-induced roll moment
        # Linear approximations for positive Ja values
        # KT ~= KT_0 + (KT_max-KT_0)/Ja_max * Ja
        # KQ ~= KQ_0 + (KQ_max-KQ_0)/Ja_max * Ja

        # Rigi-body/added mass Coriolis/centripetal matrices expressed in the CO
        CRB = m2c(self.MRB, nu_r)
        CA = m2c(self.MA, nu_r)

        # CA-terms in roll, pitch and yaw can destabilize the model if quadratic
        # rotational damping is missing. These terms are assumed to be zero
        CA[4][0] = 0  # Quadratic velocity terms due to pitching
        CA[0][4] = 0
        CA[4][2] = 0
        CA[2][4] = 0
        CA[5][0] = 0  # Munk moment in yaw
        CA[0][5] = 0
        CA[5][1] = 0
        CA[1][5] = 0

        C = CRB + CA

        # Dissipative forces and moments
        D = np.diag([
            self.M[0][0] / self.T_surge,
            self.M[1][1] / self.T_sway,
            self.M[2][2] / self.T_heave,
            self.M[3][3] * 2 * self.zeta_roll * self.w_roll,
            self.M[4][4] * 2 * self.zeta_pitch * self.w_pitch,
            self.M[5][5] / self.T_yaw
        ])

        # Linear surge and sway damping
        D[0][0] = D[0][0] * math.exp(-3 * U_r)  # vanish at high speed where quadratic
        D[1][1] = D[1][1] * math.exp(-3 * U_r)  # drag and lift forces dominates

        # tau_liftdrag = forceLiftDrag(self.diam, self.S, self.CD_0, alpha, U_r)
        # tau_crossflow = crossFlowDrag(self.L, self.diam, self.diam, nu_r)

        # Restoring forces and moments
        g = gvect(self.W, self.B, eta[4], eta[3], self.r_bg, self.r_bb)

        # Horizontal- and vertical-plane relative speed
        U_rh = math.sqrt(nu_r[0] ** 2 + nu_r[1] ** 2)
        U_rv = math.sqrt(nu_r[0] ** 2 + nu_r[2] ** 2)

        mpc_A = np.array(-np.matmul(self.Minv, C + D))
        mpc_B_rhs = np.array([
            [0, 0, (1 - t_prop) * self.rho * pow(D_prop, 4) * (
                    KT_0 * abs(1525 / 60) + (KT_max - KT_0) / Ja_max * (Va / D_prop)) / 60],
            [-0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r, 0, 0],
            [0, -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s, 0],
            [0, 0,
             self.rho * pow(D_prop, 5) * (KQ_0 * abs(1525 / 60) + (KQ_max - KQ_0) / Ja_max * (Va / D_prop)) / 600],
            [0, self.x_s * -0.5 * self.rho * U_rv ** 2 * self.A_s * self.CL_delta_s, 0],
            [self.x_r * -0.5 * self.rho * U_rh ** 2 * self.A_r * self.CL_delta_r, 0, 0]
        ])
        mpc_B = -np.matmul(self.Minv, mpc_B_rhs)
        # 常数项
        mpc_I = -np.matmul(self.Minv, g)
        self.z_d = math.exp(-sampleTime * self.wn_d_z) * self.z_d \
                   + (1 - math.exp(-sampleTime * self.wn_d_z)) * self.ref_z
        delta_Z = (self.z_d - eta[2]) / 10

        mpc_states = ca.SX.sym('x', 6)
        n_states = mpc_states.size()[0]
        mpc_controls = ca.SX.sym('u', 3)
        n_controls = mpc_controls.size()[0]
        mpc_rhs = (mpc_A @ mpc_states) + (mpc_B @ mpc_controls) + mpc_I
        f = ca.Function('f', [mpc_states, mpc_controls], [mpc_rhs], ['system_state', 'control_input'], ['rhs'])

        U = ca.SX.sym('U', n_controls, pre_step)
        X = ca.SX.sym('X', n_states, pre_step + 1)
        P = ca.SX.sym('P', n_states + n_states)

        X[:, 0] = P[:6]

        for i in range(pre_step):
            # 通过前述函数获得下个时刻系统状态变化。
            # 这里需要注意引用的index为[:, i]，因为X为(n_states, N+1)矩阵
            f_value = f(X[:, i], U[:, i])
            X[:, i + 1] = X[:, i] + f_value * sampleTime

        # ff = ca.Function('ff', [U, P], [X], ['input_U', 'target_state'], ['horizon_states'])

        Q = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        R = np.array([[0.5, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        obj = 0  # 初始化优化目标值
        for i in range(pre_step):
            # 在 N 步内对获得优化目标表达式
            obj = obj + ca.mtimes([(X[:, i] - P[6:]).T, Q, X[:, i] - P[6:]]) + ca.mtimes([U[:, i].T, R, U[:, i]])

        gg = []  # 用list来存储优化目标的向量
        for i in range(pre_step):
            # 这里的约束条件只有小车的坐标（x,y）必须在-2至2之间
            # 由于xy没有特异性，所以在这个例子中顺序不重要（但是在更多实例中，这个很重要）
            gg.append(X[0, i])
            gg.append(X[1, i])
            gg.append(X[2, i])
            gg.append(X[3, i])
            gg.append(X[4, i])
            gg.append(X[5, i])

        nlp_prob = {'f': obj, 'x': ca.reshape(U, -1, 1), 'p': P, 'g': ca.vertcat(*gg)}

        opts_setting = {'ipopt.max_iter': 1000,
                        'ipopt.print_level': 0,
                        'print_time': 0,
                        'ipopt.acceptable_tol': 1e-8,
                        'ipopt.acceptable_obj_change_tol': 1e-6}

        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

        lbx = []  # 最低约束条件
        ubx = []  # 最高约束条件
        for _ in range(pre_step):
            lbx.append(-self.deltaMax_r)
            ubx.append(self.deltaMax_r)
            lbx.append(-self.deltaMax_s)
            ubx.append(self.deltaMax_s)
            lbx.append(self.nMax - 25)
            ubx.append(self.nMax)

        mpc_state0 = nu.reshape(-1, 1)  # 小车初始状态
        mpc_state_stable = np.array([0.0, 0.0, 0.0, delta_Z, 0.0, 0.0]).reshape(-1, 1)  # 小车末了状态

        mpc_u0 = np.array(np.tile(u_actual, (pre_step, 1))).reshape(-1, 3)  # 系统初始控制状态，为了统一本例中所有numpy有关

        c_p = np.concatenate((mpc_state0, mpc_state_stable))
        init_control = ca.reshape(mpc_u0, -1, 1)
        res = solver(x0=init_control, p=c_p, lbx=lbx, ubx=ubx)
        u_sol = ca.reshape(res['x'], n_controls, pre_step)
        u_control = np.array(u_sol[:, 0]).reshape(1, 3)[0]
        # ff_value = ff(u_sol, c_p)
        return u_control
