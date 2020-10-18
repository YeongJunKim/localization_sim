function [A B] = measurement_update2(anchors_pos)
% A: matlabFunction with nonlinear measurement_update
% B: matlabFunction with linearized measurement_update
syms x y theta
syms v_l v_theta

h_main.dist1(x,y,theta) = norm(x-anchors_pos(:,1));
h_main.dist2(x,y,theta) = norm(x-anchors_pos(:,2));
h_main.dist3(x,y,theta) = norm(x-anchors_pos(:,3));
h_main.dist4(x,y,theta) = norm(x-anchors_pos(:,4));

h_main.theta(x,y,theta,v_l,v_theta) = theta;

% hh = [];
% for i = 1:3
%     hh(i).a = h_main.dist1;
% end
% disp(hh);

h_main = [h_main.dist1 h_main.dist2 h_main.dist3 h_main.dist4  h_main.theta]';
Jacobian_H_main = jacobian(h_main,[x,y,theta]);
A = matlabFunction(h_main);
B = matlabFunction(Jacobian_H_main);
end