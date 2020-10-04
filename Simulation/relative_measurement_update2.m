function [A, B] = relative_measurement_update2()
% A: matlabFunction with nonlinear measurement_update
% B: matlabFunction with linearized measurement_update
syms x1 y1 y2 x2 theta v_l v_theta
syms pj1_1 pj1_2 pj2_1 pj2_2 pj3_1 pj3_2

dist1(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = norm([pj1_1 pj1_2]' - [x1 y1]');
dist2(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = norm([pj2_1 pj2_2]' - [x1 y1]');
dist3(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = norm([pj3_1 pj3_2]' - [x1 y1]');
% eta1(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = atan((pj1_2-y1)/(pj1_1-x1))-theta;
% eta2(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = atan((pj1_2-y1)/(pj1_1-x1))-theta;
% eta3(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = atan((pj1_2-y1)/(pj1_1-x1))-theta;
eta1(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = theta;
eta2(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = theta;
eta3(x1,y1,theta,pj1_1, pj1_2, pj2_1, pj2_2, pj3_1, pj3_2) = theta;
h_main = [dist1 dist2 dist3 eta1 eta2 eta3 theta]';


Jacobian_H_main = jacobian(h_main,[x1,y1,theta]);

A = matlabFunction(h_main);

B = matlabFunction(Jacobian_H_main);
end