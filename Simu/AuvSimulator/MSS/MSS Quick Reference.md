# MSS Quick Reference

The m-files of the MSS (Marine Systems Simulator) toolbox are compatible with MATLAB (www.mathworks.com) and the free software GNU Octave (www.octave.org), facilitating broad accessibility and application in marine systems simulation. To begin using the MSS toolbox, ensure it is installed and properly set up in your MATLAB/Octave environment. Copy the directory MSS/... to your computer and "add the path with subfolders" to Matlab/Octave. 

```matlab
>> mssHelp            % List MSS commands
>> mssSimulink        % Simulink Library (not available in GNU Octave)
```

## Table of Contents
- [Simulink demos](#simulink-demos)
- [MSS demos (m-files)](#mss-demos-m-files)
- [Examples (m-files)](#examples-m-files)
- [Marine craft simulator (m-files)](#marine-craft-simulator-m-files)
  - [Marine craft models](#marine-craft-models)
  - [Time-series simulation](#time-series-simulation)
- [Utils (m-files)](#utils)
  - [Modelling](#modelling)
  - [Kinematics](#kinematics)
  - [Environment](#environment)
  - [Ship maneuvers and visualization](#ship-maneuvers-and-visualization)
  - [Motion sickness](#motion-sickness)
  - [Transformations](#transformations)
  - [Numerical integration methods](#numerical-integration-methods)
- [GNC (m-files)](#gnc-m-files)
  - [Guidance](#guidance)
  - [Navigation](#navigation)
  - [Control](#control)  
- [HYDRO](#hydro)
  - [Hydrodynamic templates (Simulink)](#hydrodynamic-templates-simulink)
  - [Processing of data from hydrodynamic codes (m-files)](#processing-of-data-from-hydrodynamic-codes-m-files)
  - [Data files (mat-files that can be loaded to workspace and used by Simulink templates)](#data-files-mat-files-that-can-be-loaded-to-workspace-and-used-by-simulink-templates)
  - [Hydrodynamics (m-files)](#hydrodynamics-m-files)
- [Frequency-domain identification (FDI) of radiation models (m-files)](#frequency-domain-identification-fdi-of-radiation-models-m-files)
  - [Demos](#demos)
  - [Utils](#utils)
---

## Simulink demos

```matlab
demoAUVdepthHeadingControl.slx            % Simultaneously heading and depth control of the Remus 100 AUV
demoCS2passiveObserverDP.slx              % Passive observer with wave filtering and nonlinear PID control for the CyberShip2 model ship
demoDPThrusterModels.slx                  % Supply vessel with azimuth thrusters
demoDSRVdepthControl.slx                  % DSRV depth control system
demoKalmanWavefilterAutop.slx             % Kalman-filter based wave filter and heading autopilot for the mariner class cargo ship
demoMarinerPathFollowingCourseControl.slx % Mariner class vessel LOS path-following control using a course autopilot
demoNavalVesselMano.slx                   % Zigzag test for the naval ship Mano  
demoNPSAUV.slx                            % NPS AUV heading control system
demoOtterUSVHeadingControl.slx	          % Otter USV heading control system
demoOtterUSVPathFollowingCourseControl    % Otter USV LOS path-following control using a course autopilot
demoOtterUSVPathFollowingHeadingControl   % Otter USV ILOS and ALOS path-following control using a heading autopilot
demoPanamaxContainerShip.slx              % Panama container ship simulator
demoPassiveWavefilterAutopilot1.slx       % Passive wave filter and heading autopilot design using compass measurements only
demoPassiveWavefilterAutopilot2.slx       % Passive wave filter and heading autopilot design using a compass and yaw rate measurements
demoS175WindCurrentAutopilot.slx          % S175 heading autopilot with wind and current loads
demoSemisubDPsystem.slx                   % Semisubmersible DP system
demoWaveElevation.slx                     % Computation of wave elevation from wave spectra
demoWaypointGuidance.slx                  % Waypoint guidance system
```

## MSS demos (m-files)
```matlab
1) KinDemo     % Euler angle and quaternion kinematics
2) ManDemo     % Maneuvering trials
3) StabDemo    % Straight-line, directional and positional motion stability
4) WaveDemo    % Wave spectra demonstrations
```

## Examples (m-files)
```matlab
exAUVhydrostatics  % Computation of the hydrostatic quantities for a cylinder-shaped AUV 
exBoxShapedShip    % Computation of the transverse metacentric height and the heave/roll periods of a box-shaped ship
exFeedback         % For-loop implementation for numerical integration of a 1st-order system under feedback and feedforward control
exFFT              % Estimation of the wave encounter frequency from time series using the fast-Fourier transform (FFT)
exHybridPath       % Computation of a hybrid continuous path parametrized by waypoints
exINS_AHRS         % Euler angle error-state (indirect) Kalman filter for INS aided by GNSS position and AHRS attitude measurements 
exINS_Euler        % Euler angle error-state (indirect) Kalman filter for INS aided by GNSS position and compass measurements
exINS_MEKF         % Unit quaternion error-state (indirect) Kalman filter for INS aided by position and magnetic field measurements 
exKF               % For-loop implementation (predictor-corrector representation) of a discrete-time linear Kalman filter (KF) 
exKT               % Computation of the Nomoto gain K and time constant T from a step response using nonlinear least-squares
exLinspec          % Linear approximations to the PM, JONSWAP, and Torsethaugen spectra using nonlinear least-squares
exLQFinHor         % LQ finite time-horizon tracking controller for a mass-damper-spring system
exLQR              % Computes the LQR gains for a mass-damper system
exLQtrack          % Computes the LQ optimal tracking gains for a mass-damper system
exMDS              % Plots the step response of a 2nd-order mass-damper system
exMSI              % Plots the ISO 2631-1 (1997) and O'Hanlon and McCauley (1974) Motion Sickness Incidence (MSI) curves
exNomoto           % Bode plots of ships parametrized by Nomoto’s 1st- and 2nd-order models
exObsCtr           % Observability and controllability matrices of a supply vessel
exOtter            % Simulates an Otter USV equipped with two propellers
exPassiveObs       % Plots the loop transfer function of the passive observer used for heading control
exPathGen          % Path generation using cubic polynomials 
exPlotRAO          % Script for plotting motion and force RAOs
exPullout          % Performs a pullout maneuver for two different ships
exQuadProg         % Quadratic programming applied to waypoint trajectory generation
exQuest            % 6-DOF position/attitude vector from camera measurements using the QUEST algorithm
exRefMod           % 2nd-order reference model with nonlinear damping and velocity saturation
exResonance        % Computes the closed-form responses in heave, roll, and pitch for a marine craft exposed to regular waves
exRRD1             % Roll and sway-yaw transfer functions for the Son and Nomoto container ship
exRRD2             % Rudder-roll damping (RRD) system for the Son and Nomoto container ship
exRRD3             % Inverse response in roll for the Son and Nomoto container ship due to a right-half-plane zero  (non-minimum phase)  
exShipHydrostatics % Computation of the hydrostatic quantities for a ship 
exSMC              % Integral sliding mode control (SMC) design for heading control
exSpline           % Path generation using cubic Hermite spline interpolation 
exSTA              % Adaptive-gain super twisting algorithm (STA) for heading control
exTurnCircle       % Generates the turning circle for two different ships
exWageningen       % Computes thrust and torque curves for a propeller using the Wageningen B-series data
exWindForce        % Plots the wind coefficients by Isherwoods (1972) 
exZigZag           % Generates zigzag maneuvers for two different ships and the Remus 100 AUV
```

## Marine craft simulator (m-files)

### Marine craft models

```matlab
clarke83        % Linear maneuvering model parametrized using (L,B,T) found from linear regression of model tests (Clarke et al. 1983)
container       % Nonlinear maneuvering model of a high-speed container ship, L = 175 m, including roll (Son and Nomoto 1982)
DSRV            % Deep Submergence Rescue Vehicle (DSRV), L = 5.0 m (Healey 1992)
frigate         % Nonlinear autopilot model for a frigate, L = 100 m
Lcontainer      % Linearized model of a high-speed container ship, L = 175 m, including the roll mode (Son and Nomoto 1982)
mariner         % Nonlinear maneuvering model for the Mariner class vessel, L = 160 m 
navalvessel     % Nonlinear maneuvering model of a multipurpose naval vessel, L = 51.5 m
npsauv          % Naval Postgraduate School Autonomous Underwater Vehicle (AUV), L = 5.3 m (Healey and Lienard 1993)
osv             % Nonlinear model of an Offshore Supply Vessel (OSV), L = 83.0 m
otter           % OTTER small Uncrewed Surface Vehicle (USV), L = 2.0 m
remus100        % REMUS 100 Autonomous Underwater Vehicle (AUV), L = 1.9 m
rig             % Semisubmersible linear mass-damper-spring model, L = 84.6 m
supply          % Linear DP model of a supply vessel, L = 76.2 m
tanker          % Nonlinear course unstable maneuvering model of a tanker, L = 304 m 
zeefakkel       % Nonlinear autopilot model of a recreational craft, L = 45 m
```

### Time-series simulation

```matlab
SIMclarke83     % Simulate a generic ship model characterized by L, B, and T under PD control
SIMdsrv         % Simulate DSRV.m with an autopilot for depth control using successive-loop closure
SIMfrigate      % Simulate frigate.m using a PID heading autopilot
SIMmariner      % Simulate mariner.m with heading control and 2-D LOS straight-line path course control
SIMotter        % Simulate otter.m with heading control and 2-D LOS straight-line path course control
SIMosv          % Simulate osv.m under nonlinear DP control with constrained control allocation (dynamic optimization)
SIMcontainer    % Simulate container.m and Lcontainer.m under PD control
SIMnavalvessel  % Simulate navalvessel.m under PD control
SIMnpsauv       % Simulate npsauv.m with MIMO PID autopilots for depth and heading control, and 3-D straight-line path following using ALOS
SIMremus100     % Simulate remus100.m with autopilots for depth and heading control, and 3-D straight-line path following using ALOS
SIMrig          % Simulate the 6-DOF semisubmersible model under PID control
SIMsupply       % Simulate the linear supply vessel model under DP control
SIMtanker       % Simulate tanker.m under PD control
SIMzeefakkel    % Simulate zeefakkel.m using a PID heading autopilot
```

### Utils (m-files)

## Modelling

```matlab
addedMassSurge      % Hydrodynamic added mass in surge, A11, approximated by the formula of Söding (1982)
coeffLiftDrag       % Hydrodynamic lift and drag coefficients as a function of angle of attack of a submerged "wing profile"
forceLiftDrag       % Hydrodynamic lift and drag forces as a function of the angle of attack of a submerged "wing profile" 
forceSurgeDamping   % Linear and quadratic damping forces in the surge direction
crossFlowDrag 	    % Crossflow drag computed from strip theory integrals
Dmtrx               % 6x6 linear damping matrix for marine craft (submerged and floating)
Gmtrx               % 6x6 system spring stiffness matrix G
gvect               % 6x1 vector of restoring forces, Euler angles as input
gRvect              % 6x1 vector of restoring forces, Euler angle, or unit quaternion rotation matrix as input
imlay61             % 6x6 hydrodynamic added mass and Coriolis-centripetal matrices MA and CA for a prolate spheroid
m2c                 % 6x6 Coriolis-centripetal matrix C(nu) from system inertia matrix M
rbody               % 6x6 rigid-body system inertia and Coriolis-centripetal matrices MRB and CRB of a general body
spheroid            % 6x6 rigid-body system inertia and Coriolis-centripetal matrices MRB and CRB of a prolate spheroid 
thrConfig           % 3xr thruster configuration matrix for main propellers, tunnel thrusters, and azimuth thrusters
wageningen          % Thrust and torque coefficients of the Wageningen B-series propellers 
```

## Kinematics

```matlab
ecef2llh            % Longitude, latitude, and height from ECEEF positions x, y, and z
euler2q             % Unit quaternion from Euler angles
eulerang            % Computes the Euler angle transformation matrices J, Rzyx and Tzyx
flat2llh            % Longitude, latitude, and height from flat-earth positions x, y, and z
llh2ecef            % ECEEF positions x, y, and z from longitude, latitude, and height
llh2flat            % Flat-Earth positions x, y, and z from longitude, latitude, and height
R2euler             % Euler angles from rotation matrix elements
Rll                 % Euler angle rotation matrix Rll for longitude and latitude
Rquat               % Unit quaternion rotation matrix R in SO(3)
Rzyx                % Euler angle rotation matrix R in SO(3)
Tquat               % Unit quaternion transformation matrix T, representing the attitude dynamics
Tzyx                % Euler angle transformation matrix T, representing the attitude dynamics
q2euler             % Euler angles from a unit quaternion
quatern             % Unit quaternion transformation matrix J
quatprod            % Quaternion product
quest               % Quaternion rotation matrix R(q) and unit quaternion q between two vectors W = R(q) V
quest6dof           % 6-DOF vector eta = [x,y,z,phi,theta,psi] from three marker positions using the QUEST algorithm
```

## Environment

```matlab
blendermann94       % Computes the wind forces and wind coefficients using Blendermann (1994)
encounter           % Encounter frequency as a function of wave peak frequency, vessel speed, and wave direction
hs2vw               % Converts significant wave height into an equivalent wind speed
isherwood72         % Computes the wind forces and coefficients based on Isherwood (1972) 
rand_phases         % Generates a uniformly distributed vector of random phases in the interval [-pi pi]
vw2hs               % Converts average wind speed to significant wave height
waveresponse345     % Steady-state heave, roll, and pitch responses for a ship in regular waves 
wavespec            % Function used to evaluate different types of wave spectra
ww2we               % Function used to transform a vector of wave frequencies to encounter frequencies
```

## Ship maneuvers and visualization

```matlab
animateShip         % Animates a Viking ship moving along a specified path on a North-East (y-x) plot
pullout             % Ship pullout maneuver
turncircle          % Ship turning circle
zigzag              % Zigzag maneuver for 3-DOF models
zigzag6dof          % Zigzag maneuver for 6-DOF models
```

## Motion sickness

```matlab
ISOmsi              % ISO 2631-3, 1997 motion sickness incidence
HMmsi               % O'Hanlon and McCauley (1974) motion sickness incidence
```

## Transformations

```matlab
conversion          % Defines global conversion factors for GNC applications
rad2pipi            % Obsolete, use ssa.m
ssa                 % Smallest signed angle maps an angle in rad to the interval [-pi pi) or [-180 180)
Smtrx               % 3x3 vector skew-symmetric matrix S
Hmtrx               % 6x6 system transformation matrix H
vex                 % Computes a = vex(S(a)) where S is a skew-symmetric matrix
```

## Numerical integration methods

```matlab
euler2              % Integrates a system of ordinary differential equations using Euler’s 2nd-order method
rk4                 % Integrates a system of ordinary differential equations using Runge-Kutta’s 4th-order method
```

### GNC (m-files)

## Guidance

```matlab
ALOS3D               % ALOS guidance laws for heading and pitch control in 3-D
ALOSpsi              % ALOS guidance law for heading control in 2-D (see demoOtterUSVPathFollowingHeadingControl.slx)
crosstrack           % Computes the path-tangential origin and cross-track error for a target
crosstrackWpt        % Computes the cross-track error when the path is a straight line between two waypoints
crosstrackHermiteLOS % Computes the cross-track error and LOS angle to a cubic Hermite spline defined by waypoints
crosstrackWpt3D      % Computes the 3-D tracking errors (along-, cross- and vertical-track errors)
getPathSignal        % Generates the coefficients for subpaths between given waypoints
hermiteSpline        % Computes a cubic Hermite spline and the tangents to the spline for a given waypoint
hybridPath           % Generates coefficients for sub-paths between waypoints
LOSchi               % LOS guidance law for course control in 2-D (see demoOtterUSVPathFollowingCourseControl.slx)
LOSobserver          % Estimates the desired LOS angle and LOS rate from a LOS guidance law command
ILOSpsi              % ILOS guidance law for heading control in 2-D (see demoOtterUSVPathFollowingHeadingControl.slx)
order3               % Path generation using cubic polynomials (see demoWaypointGuidance.slx)
order5               % Path generation using 5th-order polynomials (see demoWaypointGuidance.slx)
refModel             % Third-order reference model for position, velocity and acceleration
```

## Navigation

```matlab
acc2rollpitch        % Static roll and pitch angles from the specific force
EKF_5states          % Estimation of SOG, COG, and course rate from NED positions or latitude-longitude
gravity              % Acceleration of gravity as a function of latitude using the WGS-84 ellipsoid parameters
insSignal            % Basic INS signal generator
```

INS error-state Kalman filters (ESKF)
```matlab
ins_ahrs             % Error-state Kalman filter (ESKF) for INS aided by position and AHRS measurements 
ins_euler            % Error-state Kalman filter (ESKF) for INS aided by position and yaw angle measurements
ins_mekf             % Error-state Kalman filter (ESKF) for INS aided by position and magnetic field measurements
ins_mekf_psi         % Error-state Kalman filter (ESKF) for INS aided by position and yaw angle measurements
```

INS time-series simulation
```matlab
SIMaidedINSeuler     % Simulate the ESKF for aided INS using Euler angles
SIMaidedINSquat      % Simulate the ESKF for aided INS using unit quaternions (MEKF representation)
```

## Control

```matlab
allocPseudoinverse    % Unconstrained control allocation
integralSMCheading    % Integral sliding mode controller for heading control    
lqtracker             % Computes the LQ tracker gain matrices for LTI systems
nomoto                % Generates Bode plots for the 1st- and 2nd-order Nomoto models
PIDnonlinearMIMO      % Nonlinear MIMO PID regulator for dynamic positioning (DP)
ucalloc               % Obsolete, use allocPseudoinverse.m
```

### HYDRO

## Hydrodynamic templates (Simulink)

```matlab
DP_ForceRAO.slx    % Simulink template for a DP vessel where wave loads are computed using force RAOs
DP_MotionRAO.slx   % Simulink template for a DP vessel where wave loads are computed using motion RAOs
MAN_ForceRAO.slx   % Simulink template for the unified maneuvering model where wave loads are computed using force RAOs
```

## Processing of data from hydrodynamic codes (m-files)

```matlab
veres2vessel       % Reads data from ShipX output files and stores the data as a mat-file containing the structure <vessel>
vessel2ss          % computes the fluid-memory transfer functions and stores the data as a mat-file containing the structure <vesselABC>
wamit2vessel       % Reads data from WAMIT output files and stores the data as a mat-file containing the structure <vessel>
```

## Data files (mat-files that can be loaded to workspace and used by Simulink templates)

```matlab
fpso, fpsoABC       % WAMIT data for a FPSO
semisub, semisubABC % WAMIT data for a semisubmersible
tanker, tankerABC   % WAMIT data for a tanker
s175, s175ABC       % ShipX data for a supply vessel
supply, supplyABC   % ShipX data for the S175
```

After loading the data files to the workspace using the Matlab command load, the following data structures are available:

| vessel   | vessel.main | vesselABC |
| -------- | --------    | --------  |
|          main: [1×1 struct]      |      name: 'tanker' | Ar: {6×6 cell} |
|    velocities: 1×60              |         T: draft    | Br: {6×6 cell} |
|      headings: [1×36 double]     |         B: beam     | Cr: {6×6 cell} |
|           MRB: [6×6 double]      |       Lpp: length   | Dr: {6×6 cell} | 
|             C: [6×6×60 double]   |         m: mass     | MRB: [6×6 double] | 
|         freqs: [1×60 double]     | rho: density of water | MA: [6×6 double] | 
|             A: [6×6×60 double]   | k44: radius of gyration | G: [6×6 double] | 
|             B: [6×6×60 double]   | k55: radius of gyration | Minv: [6×6 double] | 
|     motionRAO: [1×1 struct]      | k66: radius of gyration | r_g: [x_g y_g z_g] | 
|      forceRAO: [1×1 struct]      | g: acceleration of gravity | 
|      driftfrc: [1×1 struct]      | nabla: volume displacement | 
|            Bv: [6×6×60 double]   | CB: center of buoyancy |
|                                  | GM_T: transverse metacentric height |
|                                  | GM_L: longitudinal metacentric height |
|                                  | CG: center of gravity | |


## Hydrodynamics (m-files)

```matlab
DPperiods           % Periods and natural frequencies of a marine craft in DP
Hoerner             % 2-D Hoerner cross-flow form coefficient as a function of B and T
loadcond            % Plots the roll and pitch periods as a function of GM_T and GM_L
plotABC             % Plots the hydrodynamic coefficients Aij, Bij, and Cij as a function of frequency 
plotBv              % Plots viscous damping Bvii as a function of frequency 
plotTF              % Plots the motion or force RAO transfer functions
plotWD              % Plots the wave drift amplitudes
```

## Frequency-domain identification (FDI) of radiation models (m-files)

### Demos
```matlab
Demo_FDIRadMod_NA     % FDI using hydrodynamic data without infinite-frequency added mass
Demo_FDIRadMod_WA     % FDI using hydrodynamic data, including infinite-frequency added mass
```

### Utils
```matlab
EditAB                 % Function preparing the data for identification, selecting frequency range, and removing wild-points
FDIRadMod              % Identify the SISO transfer function corresponding to the coupling specified
fit_siso_fresp         % Fit a continuous SISO transfer function to the frequency response data
ident_retardation_FD   % Identification of a parametric radiation convolution model K(s) = P(s)/Q(s)
ident_retardation_FDna % Identification of a parametric model A(jw) = B(w)/(jw) - A(w) 
```

