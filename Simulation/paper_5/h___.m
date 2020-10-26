function [A1, A2, A3, B1, B2, B3] = h___()
syms x1 x2 theta xj1 xj2
h.dist(x1,x2,theta,xj1,xj2) = sqrt((x1-xj1)^2 + (x2-xj2)^2);
h.eta(x1,x2,theta,xj1,xj2) = atan2((xj2-x2), (xj1-x1));
h.theta(x1,x2,theta,xj1,xj2) = theta;

h.xdiff(x1,x2,theta,xj1,xj2) = xj1 - x1;
h.ydiff(x1,x2,theta,xj1,xj2) = xj2 - x2;
h.theta(x1,x2,theta,xj1,xj2) = theta;

% h1 = [h.dist]';
% h2 = [h.eta]';
% h3 = [h.theta]';

h1 = [h.xdiff]';
h2 = [h.ydiff]';
h3 = [h.theta]';

% h3 = [h.dist h.eta h.theta]';
jh1 = jacobian(h1,[x1,x2, theta]);
jh2 = jacobian(h2,[x1,x2, theta]);
jh3 = jacobian(h3,[x1,x2, theta]);
A1 = matlabFunction(h1);
A2 = matlabFunction(h2);
A3 = matlabFunction(h3);
B1 = matlabFunction(jh1);
B2 = matlabFunction(jh2);
B3 = matlabFunction(jh3);
end