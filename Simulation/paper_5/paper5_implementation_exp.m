% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs

% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-10-18
%
% https://www.github.com/YeongJunKim/matlab/filters/
%
% Important thing to achieve the sim & ex.

clc;
clear all;
% close all;


addpath('./../../../matlab/filters/');
addpath('./../../../matlab/filters/FIR');
addpath('./../../../matlab/filters/RDFIR');
addpath('./../../../matlab/filters/RDEKF');
addpath('./../../../matlab/filters/EKF');
addpath('./..');

global app;
global estimator;


r = exp_settings();disp(r);
r = exp_initialization();disp(r);

addpath('./experiment_data/');
load('ex3_exp_data.mat');

%
app.iteration = inf;
for ct = 1:app.agent_num
    if size(experiment_data(ct).user_input, 2) < app.iteration
        app.iteration = size(experiment_data(ct).user_input, 2);
    end
end
fprintf("iteration is : %d\r\n", app.iteration);
% make trajectory only measured input data
interval = 1:app.iteration;
for ct = 2:app.iteration
    for ag = 1:app.agent_num
        app.result.agent(ag).trajectory.only_odom_input(:,ct) = dynamics_nonholonomic(app.result.agent(ag).trajectory.only_odom_input(:,ct-1), experiment_data(ag).odom_data(:,ct-1), 1.5);
        app.result.agent(ag).trajectory.only_user_input(:,ct) = dynamics_nonholonomic(app.result.agent(ag).trajectory.only_user_input(:,ct-1), experiment_data(ag).user_input(:,ct-1), 1.5);
    end
end

for ag = 1:app.agent_num
    x = zeros(1,app.iteration); x(interval) = app.result.agent(ag).trajectory.only_odom_input(1,interval);
    y = zeros(1,app.iteration); y(interval) = app.result.agent(ag).trajectory.only_odom_input(2,interval);
    app.ax1_plots{ag}.XData = x;
    app.ax1_plots{ag}.YData = y;
end

% init ahrsv1 data
for ag = 1:app.agent_num
    experiment_data(ag).ahrsv1(:) = experiment_data(ag).ahrsv1(:) - experiment_data(ag).ahrsv1(1);
end


b_measurement = zeros(5,2);
b_measurement(1,1) = experiment_data(1).measurement(1,1);
b_measurement(2,1) = experiment_data(1).measurement(3,1);
b_measurement(3,1) = experiment_data(1).measurement(2,1);
b_measurement(4,1) = experiment_data(1).measurement(4,1);
b_measurement(1,2) = experiment_data(2).measurement(1,1);
b_measurement(2,2) = experiment_data(2).measurement(3,1);
b_measurement(3,2) = experiment_data(2).measurement(2,1);
b_measurement(4,2) = experiment_data(2).measurement(4,1);
b_measurement(1:4,1) = experiment_data(1).measurement(:,1);
b_measurement(1:4,2) = experiment_data(1).measurement(:,2);
for ct = 2:101
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
            %             fprintf("agent %d \r\n", ag);
            u = experiment_data(ag).odom_data(:,ct);
            
            alpha = 1;
            measurement = zeros(5,1);
            measurement(1) = experiment_data(ag).measurement(1,ct);
            measurement(2) = experiment_data(ag).measurement(3,ct);
            measurement(3) = experiment_data(ag).measurement(2,ct);
            measurement(4) = experiment_data(ag).measurement(4,ct);
            measurement(1:4) = experiment_data(ag).measurement(1:4,ct);
            
            for j = 1:4
                if abs(b_measurement(j,ag) - measurement(j)) > 1
                    alpha = 0;
                    disp(alpha);
                end
            end
            b_measurement(:,ag) = measurement(:);
            
            measurement(5) = wrapTo2Pi(deg2rad(experiment_data(ag).ahrsv1(ct)));
            
            estimator{app.index_RDFIR, ag}.FIR_PEFFME_run(0,u,measurement,alpha);
        else
            
        end
    end
end

% known robot's estimated trajectory
figure(99);
clf;
interval = 1:100;
for ag = 1:2
    plot(estimator{app.index_RDFIR, ag}.x_appended(1,interval), estimator{app.index_RDFIR, ag}.x_appended(2,interval), 'DisplayName', num2str(i)); hold on;
    plot(app.result.agent(ag).trajectory.only_user_input(1,interval), app.result.agent(ag).trajectory.only_user_input(2,interval), 'DisplayName', strcat(num2str(i), "real")); hold on;
end

%% make real
for ct = 1:app.iteration
    
    for i = 1:app.agent_num
        x = app.result.agent(i).trajectory.real(1,ct);
        y = app.result.agent(i).trajectory.real(2,ct);
        theta = app.result.agent(i).trajectory.real(3,ct);
        ul = app.result.agent(i).input(1,ct);
        ua = app.result.agent(i).input(2,ct);
        app.result.agent(i).trajectory.real(:,ct+1) = app.F(x, y, theta, ul, ua);
    end
end








function r = dynamics_nonholonomic(x, u, dt)
r = zeros(3,1);

r(1) = x(1) + u(1) * cos(x(3)) * dt;
r(2) = x(2) + u(1) * sin(x(3)) * dt;
r(3) = x(3) + u(2) * dt;
end












