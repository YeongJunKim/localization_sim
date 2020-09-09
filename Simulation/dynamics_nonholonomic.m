function [A B] = dynamics_nonholonomic(dt)
% A: matlabFunction with nonlinear dynamics
% B: matlabFunction with linearized dynamics
syms x y theta
syms v_l v_theta
syms x_main y_main
f_main.x(x,y,theta,v_l,v_theta) = x + v_l * cos(theta) * dt;
f_main.y(x,y,theta,v_l,v_theta) = y + v_l * sin(theta) * dt;
f_main.theta(x,y,theta,v_l,v_theta) = theta + v_theta  * dt;
f_main = [f_main.x f_main.y f_main.theta]';
Jacobian_F_main = jacobian(f_main,[x,y,theta]);
A = matlabFunction(f_main);
B = matlabFunction(Jacobian_F_main);