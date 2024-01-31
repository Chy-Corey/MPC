% ExKF Discrete-time Kalman filter (KF) implementation demonstrating
% how the "predictor-corrector representation" can be applied to a
% linear model:
% 
%  dx/dt = A * x + B * u + E * w 
%      y = C * x + v
%
%   x_k+1 = Ad * x_k + Bd * u_k + Ed * w_k 
%     y_k = Cd * x_k + v_k
%
% Ad = I + h * A, Bd = h * B, Cd = C and Ed = h * E (h = sampling time). 
%
% The case study is a ship model in yaw with heading angle measured at frequency
% f_m [Hz], which can be chosen smaller or equal to the sampling frequency 
% f_s [Hz]. The ratio between the frequencies must be a non-negative integer:
%
%     Integer:  Z = f_s/f_m >= 1  
%
% Author:    Thor I. Fossen
% Date:      17 Oct 2018
% Revisions: 28 Feb 2020, minor updates of notation
%            29 Mar 2020, added ssa() and new logic for no measurements 
%            8 Dec 2021,  map inital yaw angle to [-pi, pi)

%% USER INPUTS
f_s = 10;    % sampling frequency [Hz]
f_m = 1;     % yaw angle measurement frequency [Hz]

Z = f_s/f_m;
if ( mod(Z,1) ~= 0 || Z < 1 )
    error("f_s is not specified such that Z = f_s/f_m >= 1"); 
end

% simulation parameters
N  = 1000;		  % no. of iterations

h  = 1/f_s; 	  % sampling time: h  = 1/f_s (s) 
h_m = 1/f_m;

% model paramters for mass-damper system (x1 = yaw angle, x2 = yaw rate)
A = [0 1
     0 -0.1 ];
B = [0
     1];
E = [0
     1];
C = [1 0]; 

% discrete-time matrices
Ad = eye(2) + h * A;
Bd = h * B;
Cd = C;
Ed = h * E;
 
% initial values for x and u
x = [0 0]';	        
u = 0;

% initialization of Kalman filter
x_prd = [ssa(10) 0]';                  % map inital yaw angle to [-pi, pi)
P_prd = diag([1 1]);
Qd = 1;
Rd = 10;

%% MAIN LOOP
simdata = zeros(N+1,7);                    % table of simulation data
ydata = [0 x_prd(1)];                      % table of measurement data

for i=1:N+1
   t = (i-1) * h;                          % time (s)             

   % Plant
   u = 0.1 * sin(0.1*t);                   % input
   w = 0.1 * randn(1);                     % process noise
   x_dot = A * x + B * u + E * w;
   
   % measurements are Z times slower than the sampling time
   if mod( t, h_m ) == 0
       y = x(1) + 0.1 * randn(1);
       ydata = [ydata; t, y];
       
       % KF gain
       K = P_prd * Cd' * inv( Cd * P_prd * Cd' + Rd );
       IKC = eye(2) - K * Cd;
       
       % corrector
       P_hat = IKC * P_prd * IKC' + K * Rd * K';   
       x_hat = x_prd + K * ssa(y - Cd * x_prd);   % smallest signed angle
   else
       P_hat =P_prd;                % no measurement
       x_hat = x_prd;
   end
   
   % store simulation data in a table   
   simdata(i,:) = [t x' x_hat' P_hat(1,1) P_hat(2,2) ];    
      
   % Predictor (k+1)  
   x_prd = Ad * x_hat + Bd * u;
   P_prd = Ad * P_hat * Ad' + Ed * Qd * Ed';
   
   % Euler integration (k+1)
   x = x + h * x_dot;
end

%% PLOTS
t     = simdata(:,1); 
x     = simdata(:,2:3); 
x_hat = simdata(:,4:5); 
X_hat = simdata(:,6:7);

t_m = ydata(:,1);
y_m = ydata(:,2);

clf
figure(gcf)

subplot(211),plot(t_m,y_m,'xb',t,x_hat(:,1),'r')
xlabel('time (s)'),title('Yaw angle x_1'),grid
legend(['Measurement y = x_1 at ', num2str(f_m), ' Hz'],...
    ['Estimate x_1hat at ', num2str(f_s), ' Hz']);

subplot(212),plot(t,x(:,2),'b',t,x_hat(:,2),'r')
xlabel('time (s)'),title('Yaw rate x_2'),grid
legend(['True yaw rate x_2 at ', num2str(f_s), ' Hz'],...
    ['Estimate x_2hat at ', num2str(f_s), ' Hz']);