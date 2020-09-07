% paper: Distributed Multirobot Localization
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-09-07
% EKFDCL class is in
% https://www.github.com/YeongJunKim/matlab/filters/EKFDCL

clear all;

addpath('./../../matlab/filters/EKFDCL');

global app;

app.agent_num = 3;
app.nx = 3;


app.initial_state = zeros(app.nx * app.agent_num, 1);
app.adjacency = zeros(app.agent_num, app.agent_num);

app.initial_state(1:3,1) = [0 0 1]';
app.initial_state(4:6,1) = [0 1 0]';
app.initial_state(7:9,1) = [1 0 1.5]';

app.adjacency(1,2) = 1;
app.adjacency(2,3) = 1;
app.adjacency(3,1) = 1;

%% simulation init
app.dt = 0.5;
app.iteration = 140;

P = diag([0.1, 0.1, 0.1]);
Q = diag([0.1, 0.1, 0.1]);
R = diag([0.1, 0.1, 0.1]);

P_init = blkdiag(P,P,P);
Q_init = blkdiag(Q,Q,Q);
R_init = R;

syms x y theta
syms v_l v_theta
syms x_main y_main

f_main.x(x,y,theta,v_l,v_theta) = x + v_l * cos(theta) * app.dt;
f_main.y(x,y,theta,v_l,v_theta) = y + v_l * sin(theta) * app.dt;
f_main.theta(x,y,theta,v_l,v_theta) = theta + v_theta * app.dt;
f_main = [f_main.x f_main.y f_main.theta]';
Jacobian_F_main = jacobian(f_main,[x,y,theta]);
F = matlabFunction(f_main);
J_F = matlabFunction(Jacobian_F_main);


app.estimator = EKFDCL(P_init,Q_init,R_init, ... 
    F,J_F,app.adjacency,app.initial_state,3);



x_appended = zeros(9,[]);
x_appended(:,1) = app.initial_state;
for ct = 1:app.iteration
   for ag = 1:app.agent_num
      app.input(:,ag,ct) = [0.2 0.1+normrnd(0,0.1)]'; 
   end
   input = reshape(app.input(:,:,ct),[],1);
   x_appended(:,ct+1) = app.estimator.estimate_no_relative(input,0);
end

app.estimator = EKFDCL(P_init,Q_init,R_init, ... 
    F,J_F,app.adjacency,app.initial_state,3);
app.estimator.count = 2;
for ct = 1:app.iteration
    disp(ct);
    input = reshape(app.input(:,:,ct),[],1) + normrnd([0 0 0 0 0 0 ]', 0.1);
    
    rn = rand();
    if(rn < 0.5)
        x_hat = app.estimator.estimate_no_relative(input, 0);
    else
        rn = rand();
        if(rn < 0.2)
           rn = rand();
           if(rn < 0.8)
               fprintf("#1 get absolute position\r\n");
                app.estimator.x_pre(1:3) = x_appended(1:3,ct);
           end
           if(rn < 0.5)
               fprintf("#2 get absolute position\r\n");
                app.estimator.x_pre(4:6) = x_appended(4:6,ct);
           end
           if(rn < 0.2)
               fprintf("#3 get absolute position\r\n");
                app.estimator.x_pre(7:9) = x_appended(7:9,ct);
           end
        end
        
        rn = rand();
        if(rn < 0.33)
            measurement = x_appended(4:6,ct+1) - x_appended(1:3,ct+1) + normrnd([0,0,0]', 0.01);
            x_hat = app.estimator.estimate(1,2,input, measurement);
        elseif(rn < 0.66)
            measurement = x_appended(7:9,ct+1) - x_appended(4:6,ct+1) + normrnd([0,0,0]', 0.01);
            x_hat = app.estimator.estimate(2,3,input, measurement);
        else
            measurement = x_appended(1:3,ct+1) - x_appended(7:9,ct+1) + normrnd([0,0,0]', 0.01);
            x_hat = app.estimator.estimate(3,1,input, measurement);
        end
    end
    
%     x_hat(3) =  wrapToPi(x_hat(3));
%     x_hat(6) =  wrapToPi(x_hat(6));
%     x_hat(9) =  wrapToPi(x_hat(9));
    pause(0.01);
    figure(1);
    clf;
    interval = 1:ct;
    x(interval) = x_appended(1,interval);
    y(interval) = x_appended(2,interval);
    plot(x,y, 'r-'); hold on; grid on;
    x(interval) = x_appended(4,interval);
    y(interval) = x_appended(5,interval);
    plot(x,y, 'g-'); hold on; grid on;
    x(interval) = x_appended(7,interval);
    y(interval) = x_appended(8,interval);
    plot(x,y, 'c-'); hold on; grid on;
    x(interval) = app.estimator.x_appended(1,interval);
    y(interval) = app.estimator.x_appended(2,interval);
    plot(x,y, 'r-*'); hold on; grid on;
    x(interval) = app.estimator.x_appended(4,interval);
    y(interval) = app.estimator.x_appended(5,interval);
    plot(x,y, 'g-*'); hold on; grid on;
    x(interval) = app.estimator.x_appended(7,interval);
    y(interval) = app.estimator.x_appended(8,interval);
    plot(x,y, 'c-*'); hold on; grid on;
end

%%






































