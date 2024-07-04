function SIMcontainer()
% SIMcontainer is compatibel with MATLAB and GNU Octave (www.octave.org).
% This script simulates the dynamics of a container ship under feedback 
% control. The script concurrently simulates the ship using both a linear 
% model, defined in 'Lcontainer.m', and a nonlinear model, defined in 
% 'container.m'. The outcomes of both simulations are then plotted side by 
% side for comparative analysis.
%
% Dependencies:
%   container.m     - Nonlinear container ship model
%   Lcontainer.m    - Linearized container ship model
%   euler2.m        - Euler's integrations method
%
% Author:    Thor I. Fossen
% Date:      2018-07-21
% Revisions:
%   2024-04-19 : Enhanced compatibility with GNU Octave.

clearvars;
close all;

%% USER INPUTS
t_f = 600;   % Final simulation time (sec)
h   = 0.1;   % Sample time (sec)

Kp = 1;      % Controller P gain
Td = 10;     % Controller derivative time

% initial states:
x1 = [7 0 0 0 0 0 0 0 0 70]';   % x = [ u v r x y psi p phi delta n ]'
x2 = [7 0 0 0 0 0 0 0 0]';     

% Display simulation options
displayControlMethod();

%% MAIN LOOP
N = round(t_f/h);                       % Number of samples
simdata1 = zeros(N+1,length(x1)+2);     % Memory allocation
simdata2 = zeros(N+1,length(x2)+2);     % Memory allocation
                
for i=1:N+1

    time = (i-1) * h;                   % Simulation time in seconds

    r   = x1(3);
    psi = x1(6);
    
    % Control system (constant thrust + PD heading controller)
    psi_ref = deg2rad(5);                           % Desired heading
    delta_c = -Kp * ( ssa(psi-psi_ref) + Td * r );  % PD controller
    n_c = 70;
    
    % Ship model
    [xdot1,U1] = container(x1,[delta_c n_c]);       % Ship models
    [xdot2,U2] = Lcontainer(x2,delta_c);
   
    % Store data for presentation
    simdata1(i,:) = [time, x1', U1]; 
    simdata2(i,:) = [time, x2', U2]; 
    
    % Euler's integration method (k+1)
    x1 = euler2(xdot1,x1,h);                         
    x2 = euler2(xdot2,x2,h);   

end

%% PLOTS
scrSz = get(0, 'ScreenSize'); % Returns [left bottom width height]
legendLocation = 'best';
if isoctave; legendLocation = 'northeast'; end

t1     = simdata1(:,1);
u1     = simdata1(:,2); 
v1     = simdata1(:,3);          
r1     = rad2deg(simdata1(:,4));   
x1     = simdata1(:,5);
y1     = simdata1(:,6);
psi1   = rad2deg(simdata1(:,7));
p1     = rad2deg(simdata1(:,8));
phi1   = rad2deg(simdata1(:,9));
delta1 = rad2deg(simdata1(:,10));
n1     = simdata1(:,11);
U1     = simdata1(:,12);

t2     = simdata2(:,1);
u2     = simdata2(:,2); 
v2     = simdata2(:,3);          
r2     = rad2deg(simdata2(:,4));   
x2     = simdata2(:,5);
y2     = simdata2(:,6);
psi2   = rad2deg(simdata2(:,7));
p2     = rad2deg(simdata2(:,8));
phi2   = rad2deg(simdata2(:,9));
delta2 = rad2deg(simdata2(:,10));
U2     = simdata2(:,11);

% North-East positions
figure(1); 
set(gcf, 'Position', [1, 1, 0.5*scrSz(3), scrSz(4)]);
plot(y1,x1,'r',y2,x2,'b')
grid,axis('equal'),xlabel('East'),ylabel('North'),title('Ship position (m)')
legend('Nonlinear model','Linear model','Location',legendLocation)
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',14)

% Ship speed, yaw rate, yaw angle, roll angle, and rudder angle
figure(2); 
subplot(221),plot(t1,r1,'r',t2,r2,'b'),xlabel('Time (s)')
title('Yaw rate r (deg/s)'),grid
legend('Nonlinear model','Linear model','Location',legendLocation)
subplot(222),plot(t1,phi1,'r',t2,phi2,'b'),xlabel('Time (s)')
title('Roll angle \phi (deg/s)'),grid
legend('Nonlinear model','Linear model','Location',legendLocation)
subplot(223),plot(t1,psi1,'r',t2,psi2,'b'),xlabel('Time (s)')
title('Yaw angle \psi (deg)'),grid
legend('Nonlinear model','Linear model','Location',legendLocation)
subplot(224),plot(t1,delta1,'r',t2,delta2,'b'),xlabel('Time (s)')
title('Rudder angle \delta (deg)'),grid
legend('Nonlinear model','Linear model','Location',legendLocation)
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',14)

% Display the vessel data and an image of the vessel
vesselData = {...
    'Length', '175 m',...
    'Beam', '25.4 m',...
    'Draft', '8.5 m',...    
    'Mass', '21 750 tonnes',...
    'Volume displacement', '21 222 m3',...
    'Service speed', '7.0 m/s',...
    'Max rudder angle', '10 deg',...
    'Max propeller speed', '160 RPM'};
displayVehicleData('High-Speed Container Ship', vesselData, 'container.jpg', 3);

end

%% DISPLAY CONTROL METHOD
function displayControlMethod()
    disp('--------------------------------------------------------------------');
    disp('MSS toolbox: High-Speed Container Ship');
    disp('Linearized and nonlinear models')
    disp('PD heading autopilot');
    disp('--------------------------------------------------------------------');
    disp('Simulating...');
end
