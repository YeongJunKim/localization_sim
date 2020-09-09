function [A B] = relative_measurement_update()
% A: matlabFunction with nonlinear measurement_update
% B: matlabFunction with linearized measurement_update
syms x1 y1 y2 x2 theta v_l v_theta
h_main.dist1(x1,y1,x2,y2,theta,v_l,v_theta) = ([x2 y2]' - [x1 y1]');

h_main = [h_main.dist1]';

Jacobian_H_main = jacobian(h_main,[x1,y1,theta]);

A = matlabFunction(h_main);

B = matlabFunction(Jacobian_H_main);
end