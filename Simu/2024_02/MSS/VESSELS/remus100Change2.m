function [xdot,U, Delta]= remus100Change2(x,ui)
% Simulation algorithm for REMUS This code is based in:
% Prestero, T., 2001. Verification of a six-degree of freedom simulation 
% model for the REMUS autonomous underwater vehicle (Doctoral dissertation, 
% Massachusetts Institute of Technology and Woods Hole Oceanographic 
% Institution).         
            
%              _____F1__________________________________
%          |  /           ||                       ||   \
%          |==      F3    ||       Remus           ||    | 
%          |  \___________||_______________________||___/
%                   F2
% V=[u,v,w,p,q,r,x,y,z,phi,psi,theta];


% TERMS
% ---------------------------------------------
%STATE VECTOR:
%
% x = (u w q xpos zpos theta) ,
% Body-referenced Coordinates
% u  = Surge velocity [m/s]
% v  = Sway velocity  [m/s]
% w  = Heave velocity [m/s]
% p  = Roll rate      [rad/s]
% q  = Pitch rate     [rad/s]
% r  = Yaw rate       [rad/s]
%
% Earth-fixed coordinates
% xpos  = Position in x-direction [m]
% ypos  = Position in y-direction [m]
% zpos  = Position in z-direction [m]
% phi   = Roll angle              [rad]
% theta = Pitch angle             [rad]
% psi   = Yaw angle               [rad]
%
%INPUT VECTOR
% ui = [n delta_s delta_r]'
%  Control Fin Angles
%   n= shaft speed RPM
%   delta_s = angle of stern planes  [rad]
%   delta_r = angle of rudder planes [rad]

% Get state variables
u     = x(1); 
v     = 0; 
w     = x(2); 
p     = 0; 
q     = x(3); 
r     = 0;
phi   = 0; 
theta = x(6); 
psi   = 0;
U=sqrt(u^2+v^2+w^2);
if ui(3)<0
    ui(3)=0;
end


% Get control inputs
delta_s = ui(2); 
delta_r = ui(1);
n=ui(3)/60;
n_max = 1525;                                   % maximum propeller rpm
max_ui = [deg2rad(30) deg2rad(30) n_max/60]';   % deg, deg, rps
%max_ui=[30 30 n_max/60];
if (abs(delta_r) > max_ui(1)), delta_r = sign(delta_r) * max_ui(1); end
if (abs(delta_s) > max_ui(2)), delta_s = sign(delta_s) * max_ui(2); end
if (abs(n)       > max_ui(3)), n = sign(n) * max_ui(3); end
% 45 degree maximum rudder angle
% delta_max = 45 * pi/180;
% 
% % Check control inputs (useful later)
% if abs(delta_s) > delta_max && delta_s < delta_max
%     delta_s = sign(delta_s) * delta_max;
% end 
% 
% if abs(delta_r) > delta_max 
%     delta_r = sign(delta_r) * delta_max;
% end

% Initialize elements of coordinate system transform matrix
% ---------------------------------------- --------------------------------------
c1 = cos(phi); 
c2 = cos(theta); 
c3 = cos(psi); 
s1 = sin(phi); 
s2 = sin(theta); 
s3 = sin(psi); 
t2 = tan(theta);

% ----------------------------------------
% Vehicle Parameters and Coefficients
% ----------------------------------------
W = 2.99e2; % Weight (N)2.99
B = 2.99e2; % Bouyancy (N)%% Note buoyanci incorrect simulation fail with this value3.1

g = 9.81; % Force of gravity 
m = W/g; % Mass of vehicle

Xuu   = -1.62;    % Axial Drag
Xwq   = -3.55e1;  % Added mass cross-term
Xqq   = -1.93;    % Added mass cross-term
Xvr   =  3.55e1;  % Added mass cross-term
Xrr   = -1.93;    % Added mass cross-term
Yvv   = -1.31e3;  % Cross-flow drag
Yrr   =  6.32e-1; % Cross-flow drag
Yuv   = -2.86e1;  % Body lift force and fin lift
Ywp   =  3.55e1;  % Added mass cross-term
Yur   =  5.22;    % Added mass cross-term and fin lift
Ypq   =  1.93;    % Added mass cross-term
Zww   = -1.31e2;  % Cross-flow drag 
Zqq   = -6.32e-1; % Cross-flow drag
Zuw   = -2.86e1;  % Body lift force and fin lift
Zuq   = -5.22;    % Added mass cross-term and fin lift
Zvp   = -3.55e1;  % Added mass cross-term
Zrp   =  1.93;    % Added mass cross-term

% Center of Gravity wrt Origin at CB
xg = 0;
yg = 0;
zg = 1.96e-2;

% Control Fin Coefficients
Yuudr =  9.64;
Nuudr = -6.15;
Zuuds = -9.64; % Fin Lift Force

% Center of Buoyancy wrt Origin at Vehicle Nose
xb = 0;%-6.11e-1;
yb =  0;
zb =  0;

% Propeller Terms
%Xprop =  0.18*n*abs(n);
Xprop = 1026 * 0.14^4 * (... 
        0.4566 * abs(n) * n + (0.1798-0.4566)/0.6632 * (0.944 * U/0.14) * abs(n) );
Kpp   = -1.3e-1;  % Rolling resistance
%Kprop = -2.242e-05*n*abs(n);%-5.43e-1; % Propeller Torque
Kprop = 1026 * 0.14^5 * (...
        0.07 * abs(n) * n + (0.0312-0.07)/0.6632 * (0.944 * U/0.14) * abs(n) );
Kpdot = -7.04e-2; % Added mass

% Cross flow drag and added mass terms
Mww   =  3.18; % Cross-flow drag
Mqq   = -1.88e2; % Cross-flow drag
Mrp   =  4.86; % Added mass cross-term
Muq   = -2; % Added mass cross term and fin lift
Muw   =  2.40e1; % Body and fin lift and munk moment
Mwdot = -1.93; % Added mass
Mvp   = -1.93; % Added mass cross term
Muuds = -6.15; % Fin lift moment
Nvv   = -3.18; % Cross-flow drag
Nrr   = -9.40e1; % Cross-flow drag
Nuv   = -2.40e1; % Body and fin lift and munk moment
Npq   = -4.86; % Added mass cross-term

% Moments of Inertia wrt Origin at CB
Ixx  = 1.77e-1;   
Iyy  = 3.45;  
Izz  = 3.45;

Nwp = -1.93; % Added mass cross-term
Nur = -2.00; % Added mass cross term and fin lift

% Non-linear Moments Coefficients
Xudot = -9.30e-1; % Added mass
Yvdot = -3.55e1;  % Added mass
Nvdot =  1.93;    % Added mass
Mwdot = -1.93;    % Added mass
Mqdot = -4.88;    % Added mass
Zqdot = -1.93;    % Added mass
Zwdot = -3.55e1;  % Added mass
Yrdot =  1.93;    % Added mass
Nrdot = -4.88;    % Added mass


% added
usquare = u^2;
Zuu1 = -3.4083;
Zuu2 = -3.4083;
Zuu3 = -3.4083;
Zuu4 = -3.4083;
Muu1 = -2.1744;
Muu2 = -2.1744;
Muu3 = -2.1744;
Muu4 = -2.1744;
Btau = [Zuu1 Zuu2 Zuu3 Zuu4;
        Muu1 Muu2 Muu3 Muu4] * usquare;
Delta = pinv(Btau) * [Zuuds*u^2*delta_s; Muuds*u^2*delta_s];
tau = Btau * Delta;

% Set total forces from equations of motion
% ----------------------------------------------------- -------------------------
X = -(W-B)*sin(theta) + Xuu*u*abs(u) + (Xwq-m)*w*q + (Xqq + m*xg)*q^2 ...
    + (Xvr+m)*v*r + (Xrr + m*xg)*r^2 -m*yg*p*q - m*zg*p*r ...
    + Xprop;%n 

Y = (W-B)*cos(theta)*sin(phi) + Yvv*v*abs(v) + Yrr*r*abs(r) + Yuv*u*v ...
    + (Ywp+m)*w*p + (Yur-m)*u*r - (m*zg)*q*r + (Ypq - m*xg)*p*q ...
    + Yuudr*u^2*delta_r;%+ Yuudr*u^2*delta_r 

Z = (W-B)*cos(theta)*cos(phi) + Zww*w*abs(w) + Zqq*q*abs(q)+ Zuw*u*w ...
    + (Zuq+m)*u*q + (Zvp-m)*v*p + (m*zg)*p^2 + (m*zg)*q^2 ...
    + (Zrp - m*xg)*r*p + tau(1);%+ Zuuds*u^2*delta_s

K = -(yg*W-yb*B)*cos(theta)*cos(phi) - (zg*W-zb*B)*cos(theta)*sin(phi) ...
    + Kpp*p*abs(p) - (Izz- Iyy)*q*r - (m*zg)*w*p + (m*zg)*u*r + Kprop;%+ Kprop

M = -(zg*W-zb*B)*sin(theta) - (xg*W-xb*B)*cos(theta)*cos(phi) + Mww*w*abs(w) ...
    + Mqq*q*abs(q) + (Mrp - (Ixx-Izz))*r*p + (m*zg)*v*r - (m*zg)*w*q ...
    + (Muq - m*xg)*u*q + Muw*u*w + (Mvp + m*xg)*v*p ...
    + tau(2) ;%Muuds*u^2*

N = -(xg*W-xb*B)*cos(theta)*sin(phi) - (yg*W-yb*B)*sin(theta) ...
    + Nvv*v*abs(v) + Nrr*r*abs(r) + Nuv*u*v ...
    + (Npq - (Iyy- Ixx))*p*q + (Nwp - m*xg)*w*p + (Nur + m*xg)*u*r ...
    + Nuudr*u^2*delta_r ;%Nuudr*u^2*


FORCES = [X Y Z K M N]';





% Accelerations Matrix (Prestero Thesis page 46)
Amat = [(m - Xudot)  0               m*zg            ;
        0            0               0               ;
        0            (m - Zwdot)     (-m*xg - Zqdot) ;
        0            m*yg            0               ;
        m*zg         (-m*xg - Mwdot) (Iyy - Mqdot)   ;
        -m*yg        0               0               ];

% Inverse Mass Matrix
Minv = pinv(Amat);
% Derivatives
xdot = ...
    [Minv(1,1)*X + Minv(1,2)*Y + Minv(1,3)*Z + Minv(1,4)*K + Minv(1,5)*M + Minv(1,6)*N
     Minv(2,1)*X + Minv(2,2)*Y + Minv(2,3)*Z + Minv(2,4)*K + Minv(2,5)*M + Minv(2,6)*N
     Minv(3,1)*X + Minv(3,2)*Y + Minv(3,3)*Z + Minv(3,4)*K + Minv(3,5)*M + Minv(3,6)*N
     c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w
       -s2*u +            c2*s1*v +            c1*c2*w
                             c1*q -               s1*r] ;