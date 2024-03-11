clear;
%x_target = 0:1:4000;
%z_target=15*sin(0.005*x_target)-100;
%plot(x_target,z_target)

x_target = 0:1:4000;
z_target = 0 * x_target - 100;