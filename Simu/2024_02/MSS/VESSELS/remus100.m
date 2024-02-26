function [xdot,U] = remus100(x,ui,Vc,betaVc,w_c)
% The length of the Remus 100 AUV is 1.6 m, the cylinder diameter is 19 cm  
% and the mass of the vehicle is 31.9 kg. The maximum speed of 2.5 m/s is 
% obtained when the propeller runs at 1525 rpm in zero currents. The
% function calls are:
%   [xdot,U] = remus100(x,ui,Vc,betaVc,alphaVc,w_c)  3-D ocean currents
%   [xdot,U] = remus100(x,ui,Vc,betaVc,alphaVc)      horizontal ocean currents
%   [xdot,U] = remus100(x,ui)                        no ocean currents
% The function returns the time derivative xdot of the state vector: 
%   x = [ u v w p q r x y z phi theta psi ]',     alternatively 
%   x = [ u v w p q r x y z eta eps1 eps2 eps3 ]' 
% in addition to the speed U in m/s (optionally). The state vector can be 
% of dimension 12 (Euler angles) or 13 (unit quaternions):
%
%   u:       surge velocity          (m/s)   纵荡速度
%   v:       sway velocity           (m/s)   横荡速度
%   w:       heave velocity          (m/s)   垂荡速度
%   p:       roll rate               (rad/s) 横滚角速度
%   q:       pitch rate              (rad/s) 俯仰角速度
%   r:       yaw rate                (rad/s) 方向角速度
%   x:       North position          (m)     南北位置
%   y:       East position           (m)     东西位置
%   z:       downwards position      (m)     水下位置
%   phi:     roll angle              (rad)   横滚角       
%   theta:   pitch angle             (rad)   俯仰角
%   psi:     yaw angle               (rad)   方向角
% 
% For the unit quaternion representation, the last three arguments of the 
% x-vector, the Euler angles (phi, theta, psi), are replaced by the unit 
% quaternion q = [eta, eps1, eps2, eps3]'. This increases the dimension of 
% the state vector from 12 to 13.
%
% The control inputs are one tail rudder, two stern planes and a single-screw 
% propeller:
%
%   ui = [ delta_r delta_s n ]'  where
%
%    delta_r:   rudder angle (rad)         舵角
%    delta_s:   stern plane angle (rad)    船尾平面角
%    n:         propeller revolution (rpm) 螺旋桨转速（每分钟）
%
% The arguments Vc (m/s), betaVc (rad), w_c (m/s) are optional arguments for 
% ocean currents 洋流速度
%
%    v_c = [ Vc * cos(betaVc - psi), Vc * sin( betaVc - psi), w_c ]  
% 
% Author:    Thor I. Fossen
% Date:      27 May 2021
% Revisions: 24 Aug 2021  Ocean currents are now expressed in NED 
%            21 Oct 2021  imlay61.m is called using the relative velocity
%            30 Dec 2021  Added the time derivative of the current velocity
%            01 Feb 2022  Updated lift and drag forces
%            06 May 2022  Calibration of drag and propulsion forces using
%                         data from Allen et al. (2000)
%            08 May 2022  Added compability for unit quaternions in 
%                         addition to the Euler angle representation
%            16 Oct 2022  Added vertical currents
%            02 May 2023  Corrected the rudder area A_r
%            07 Oct 2023  Scaled down the propeller roll-induced moment
%
% Refs: 
%      B. Allen, W. S. Vorus and T. Prestero, "Propulsion system 
%           performance enhancements on REMUS AUVs," OCEANS 2000 MTS/IEEE 
%           Conference and Exhibition. Conference Proceedings, 2000, 
%           pp. 1869-1873 vol.3, doi: 10.1109/OCEANS.2000.882209.
%      T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
%           Motion Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley   

if (nargin == 2), Vc = 0; betaVc = 0; w_c = 0; end  % no ocean currents
if (nargin == 4), w_c = 0; end             % no vertical ocean currents

if (length(ui) ~= 3),error('u-vector must have dimension 3!'); end
if (length(x) ~= 12 && length(x) ~= 13)
    error('x-vector must have dimension 12 or 13'); 
end

% Constants
mu = 63.446827;         % Lattitude for Trondheim, Norway (deg) 纬度
g_mu = gravity(mu);     % gravity vector (m/s2)   重力加速度
rho = 1026;             % density of water (m/s2) 水密度

% State vectors and control inputs
nu = x(1:6); 
eta = x(7:12);
delta_r = ui(1);        % tail rudder (rad)
delta_s = ui(2);        % stern plane (rad)
n = ui(3)/60;           % propeller revolution (rps) 输入的螺旋桨转速，转换为每秒

% Amplitude saturation of the control signals 控制约束
n_max = 1525;                                   % maximum propeller rpm
max_ui = [deg2rad(30) deg2rad(30) n_max/60]';   % deg, deg, rps

if (abs(delta_r) > max_ui(1)), delta_r = sign(delta_r) * max_ui(1); end
if (abs(delta_s) > max_ui(2)), delta_s = sign(delta_s) * max_ui(2); end
if (abs(n)       > max_ui(3)), n = sign(n) * max_ui(3); end

% Ocean currents expressed in BODY
u_c = Vc * cos( betaVc - eta(6) ); % 船头方向的洋流速度（纵荡）                             
v_c = Vc * sin( betaVc - eta(6) ); % 船头垂直的洋流速度（横荡）  

nu_c = [u_c v_c w_c 0 0 0]';                  % ocean current velocities 洋流速度向量
Dnu_c = [nu(6)*v_c -nu(6)*u_c 0 0 0 0]';    % time derivative of nu_c    洋流对AUV的加速度（根据船只旋转角速度计算）

% Relative velocities/speed, angle of attack and vehicle speed
nu_r = nu - nu_c;                                 % relative velocity 船只相对洋流的速度
alpha = atan2( nu_r(3), nu_r(1) );                % angle of attack (rad) 攻角
U_r = sqrt( nu_r(1)^2 + nu_r(2)^2 + nu_r(3)^2 );  % relative speed (m/s)  相对洋流速度
U  = sqrt( nu(1)^2 + nu(2)^2 + nu(3)^2 );         % speed (m/s)           绝对速度

% AUV model parameters; Fossen (2021, Section 8.4.2) and Allen et al. (2000)
L_auv = 1.6;             % AUV length (m)   长度
D_auv = 0.19;            % AUV diamater (m) 直径
S = 0.7 * L_auv * D_auv; % planform area S = 70% of rectangle L_auv * D_auv 面积
a = L_auv/2;             % spheroid semi-axes a and b 半轴
b = D_auv/2;                  
r44 = 0.3;               % added moment of inertia in roll: A44 = r44 * Ix 洋流对auv产生的旋转转动惯量
r_bg = [ 0 0 0.02 ]';    % CG w.r.t. to the CO 坐标系转换
r_bb = [ 0 0 0 ]';       % CB w.r.t. to the CO

% Parasitic drag coefficient CD_0, i.e. zero lift and alpha = 0 阻力系数
% F_drag = 0.5 * rho * Cd * (pi * b^2)   
% F_drag = 0.5 * rho * CD_0 * S
Cd = 0.42;                              % from Allen et al. (2000)
CD_0 = Cd * pi * b^2 / S;

% Propeller coeffs. KT and KQ are computed as a function of advance no.
% Ja = Va/(n*D_prop) where Va = (1-w)*U = 0.944 * U; Allen et al. (2000)
D_prop = 0.14;   % propeller diameter corresponding to 5.5 inches 螺旋桨直径
t_prop = 0.1;    % thrust deduction number 推力减额因数（阻力与推力的比值）
Va = 0.944 * U;  % advance speed (m/s) 推进速度

%                  最大速度    最大转速
% Ja_max = 0.944 * 2.5 / (0.14 * 1525/60) = 0.6632 增益？
Ja_max = 0.6632;
        
% Single-screw propeller with 3 blades and blade-area ratio = 0.718.    
% >> [KT_0, KQ_0] = wageningen(0,1,0.718,3) 推力(thrust)和力矩(torque)系数
KT_0 = 0.4566;
KQ_0 = 0.0700;
% >> [KT_max, KQ_max] = wageningen(0.6632,1,0.718,3) 
KT_max = 0.1798;
KQ_max = 0.0312;
        
% Propeller thrust and propeller-induced roll moment
% Linear approximations for positive Ja values
% KT ~= KT_0 + (KT_max-KT_0)/Ja_max * Ja   
% KQ ~= KQ_0 + (KQ_max-KQ_0)/Ja_max * Ja  
      
if n > 0   % forward thrust
    X_prop = rho * D_prop^4 * (... 
        KT_0 * abs(n) * n + (KT_max-KT_0)/Ja_max * (Va/D_prop) * abs(n) );        
    K_prop = rho * D_prop^5 * (...
        KQ_0 * abs(n) * n + (KQ_max-KQ_0)/Ja_max * (Va/D_prop) * abs(n) );           
            
else    % reverse thrust (braking)
        
    X_prop = rho * D_prop^4 * KT_0 * abs(n) * n; 
    K_prop = rho * D_prop^5 * KQ_0 * abs(n) * n;
            
end            

% Tail rudder (single)
CL_delta_r = 0.5;        % rudder lift coefficient (-) 升力系数
A_r = 2 * 0.10 * 0.05;   % rudder area (m2)            面积
x_r = -a;                % rudder x-position (m)       坐标

% Stern plane (double)
CL_delta_s = 0.7;        % stern-plane lift coefficient (-) 
A_s = 2 * 0.10 * 0.05;   % stern-plane area (m2)
x_s = -a;                % stern-plane z-position (m)

% Low-speed linear damping matrix parameters 线性阻尼
T1 = 20;                 % time constant in surge (s)
T2 = 20;                 % time constant in sway (s)
zeta4 = 0.3;             % relative damping ratio in roll
zeta5 = 0.8;             % relative damping ratio in pitch
T6 = 5;                  % time constant in yaw (s)

% Rigid-body mass and hydrodynamic added mass 刚体质量和流体力学附加质量
[MRB,CRB] = spheroid(a,b,nu(4:6),r_bg); % 钢体质量分布矩阵，科里奥利向心矩阵
[MA,CA] = imlay61(a, b, nu_r, r44);     % 流体力学附加质量矩阵，附加科里奥利向心矩阵

% Nonlinear quadratic velocity terms in pitch and yaw. Munk moments 
% are set to zero since only linear rotational damping is used in the model
% 线性化
CA(5,1) = 0;   
CA(5,3) = 0;
CA(6,1) = 0;
CA(6,2) = 0;

% 将洋流和 AUV 自身的"效果"相加
M = MRB + MA;
C = CRB + CA;
m = MRB(1,1); W = m * g_mu; B = W;

% Dissipative forces and moments 耗散力/力矩
D = Dmtrx([T1 T2 T6],[zeta4 zeta5],MRB,MA,[W r_bg' r_bb']);
D(1,1) = D(1,1) * exp(-3*U_r);   % vanish at high speed where quadratic
D(2,2) = D(2,2) * exp(-3*U_r);   % drag and lift forces dominates
D(6,6) = D(6,6) * exp(-3*U_r);

tau_liftdrag = forceLiftDrag(D_auv,S,CD_0,alpha,U_r);
tau_crossflow = crossFlowDrag(L_auv,D_auv,D_auv,nu_r);

% Kinematics 计算欧拉角或者四元数
if (length(x) == 13)
    [J,R] = quatern(x(10:13));
else
    [J,R] = eulerang(x(10),x(11),x(12));
end

% Restoring forces and moments 恢复力？
g = gRvect(W,B,R,r_bg,r_bb);

% Horizontal- and vertical-plane relative speed 水平/垂直相对速度
U_rh = sqrt( nu_r(1)^2 + nu_r(2)^2 );  
U_rv = sqrt( nu_r(1)^2 + nu_r(3)^2 );  

% Rudder and stern-plane drag 舵角和安定面的阻力？
X_r = -0.5 * rho * U_rh^2 * A_r * CL_delta_r * delta_r^2; 
X_s = -0.5 * rho * U_rv^2 * A_s * CL_delta_s * delta_s^2;

% Rudder sway force  舵角的横荡力
Y_r = -0.5 * rho * U_rh^2 * A_r * CL_delta_r * delta_r;

% Stern-plane heave force 安定面的垂荡力
Z_s = -0.5 * rho * U_rv^2 * A_s * CL_delta_s * delta_s;

% Generalized propulsion force vector
tau = zeros(6,1);                                
tau(1) = (1-t_prop) * X_prop + X_r + X_s;
tau(2) = Y_r;
tau(3) = Z_s;
tau(4) = K_prop / 10;  % scaled down by a factor of 10 to match exp. results
tau(5) = x_s * Z_s;
tau(6) = x_r * Y_r;

% State-space model
xdot = [ Dnu_c + M \ ...
            (tau + tau_liftdrag + tau_crossflow - C * nu_r - D * nu_r  - g)
         J * nu ]; 