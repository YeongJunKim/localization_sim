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


% app basic setting
% robot num, input, state, measurement setting,
% adjacency matrix are defined
% matlab functions
r = sim_settings();disp(r);
r = sim_initialization();disp(r);

%% make real
for ct = 1:app.iteration
    
    if ct < 55
        app.result.agent(1).input(:,ct) = normrnd([0.2 -0.2]', 0.01);
        app.result.agent(2).input(:,ct) = normrnd([0.15 0.15]', 0.01);
        app.result.agent(3).input(:,ct) = normrnd([0.3 0.2]', 0.01);
        app.result.agent(4).input(:,ct) = normrnd([0.2 0.16]', 0.01);
        app.result.agent(5).input(:,ct) = normrnd([0.2 -0.21]', 0.01);
        app.result.agent(6).input(:,ct) = normrnd([0.18 -0.21]', 0.01);
        app.result.agent(7).input(:,ct) = normrnd([0.31 -0.21]', 0.01);
        app.result.agent(8).input(:,ct) = normrnd([0.2 -0.21]', 0.01);
        app.result.agent(9).input(:,ct) = normrnd([0.16 -0.21]', 0.01);
        app.result.agent(10).input(:,ct) = normrnd([0.6 -0.21]', 0.01);
        for i = 11:app.agent_num
           app.result.agent(i).input(:,ct) = normrnd([0.35 0.15]', 0.01); 
        end
    elseif ct < 80
        app.result.agent(1).input(:,ct) = normrnd([0.1 0.1]', 0.01);
        app.result.agent(2).input(:,ct) = normrnd([0.15 -0.3]', 0.01);
        app.result.agent(3).input(:,ct) = normrnd([0.2 -0.3]', 0.01);
        app.result.agent(4).input(:,ct) = normrnd([0.4 -0.36]', 0.01);
        app.result.agent(5).input(:,ct) = normrnd([0.3 0.13]', 0.01);
        app.result.agent(6).input(:,ct) = normrnd([0.3 0.13]', 0.01);
        app.result.agent(7).input(:,ct) = normrnd([0.1 -0.21]', 0.01);
        app.result.agent(8).input(:,ct) = normrnd([0.1 -0.21]', 0.01);
        app.result.agent(9).input(:,ct) = normrnd([0.1 -0.21]', 0.01);
        app.result.agent(10).input(:,ct) = normrnd([0.1 -0.21]', 0.01);
        for i = 11:app.agent_num
           app.result.agent(i).input(:,ct) = normrnd([0.25 -0.1]', 0.01); 
        end
    elseif ct < 140
        app.result.agent(1).input(:,ct) = normrnd([0.1 0.1]', 0.001);
        app.result.agent(2).input(:,ct) = normrnd([0.25 0.3]', 0.001);
        app.result.agent(3).input(:,ct) = normrnd([0.2 -0.35]', 0.001);
        app.result.agent(4).input(:,ct) = normrnd([0.1 -0.3]', 0.001);
        app.result.agent(5).input(:,ct) = normrnd([0.15 0.4]', 0.001);
        app.result.agent(6).input(:,ct) = normrnd([0.15 0.4]', 0.001);
        app.result.agent(7).input(:,ct) = normrnd([0.1 -0.21]', 0.001);
        app.result.agent(8).input(:,ct) = normrnd([0.1 -0.21]', 0.001);
        app.result.agent(9).input(:,ct) = normrnd([0.1 -0.21]', 0.001);
        app.result.agent(10).input(:,ct) = normrnd([0.1 -0.21]', 0.001);
        for i = 11:app.agent_num
           app.result.agent(i).input(:,ct) = normrnd([0.4 0.25]', 0.01); 
        end
    elseif ct < 250
        app.result.agent(1).input(:,ct) = normrnd([0.18 -0.1]', 0.001);
        app.result.agent(2).input(:,ct) = normrnd([0.25 -0.3]', 0.001);
        app.result.agent(3).input(:,ct) = normrnd([0.21 0.35]', 0.001);
        app.result.agent(4).input(:,ct) = normrnd([0.3 0.3]', 0.001);
        app.result.agent(5).input(:,ct) = normrnd([0.25 -0.4]', 0.001);
        app.result.agent(6).input(:,ct) = normrnd([0.35 -0.4]', 0.001);
        app.result.agent(7).input(:,ct) = normrnd([0.15 0.21]', 0.001);
        app.result.agent(8).input(:,ct) = normrnd([0.4 0.21]', 0.001);
        app.result.agent(9).input(:,ct) = normrnd([0.3 0.21]', 0.001);
        app.result.agent(10).input(:,ct) = normrnd([0.254 0.21]', 0.001);
        for i = 11:app.agent_num
           app.result.agent(i).input(:,ct) = normrnd([0.3 -0.25]', 0.01); 
        end
    else
        app.result.agent(1).input(:,ct) = normrnd([0.218 0.1]', 0.001);
        app.result.agent(2).input(:,ct) = normrnd([0.23 0.3]', 0.001);
        app.result.agent(3).input(:,ct) = normrnd([0.3 -0.15]', 0.001);
        app.result.agent(4).input(:,ct) = normrnd([0.15 0.1]', 0.001);
        app.result.agent(5).input(:,ct) = normrnd([0.4 0.2]', 0.001);
        app.result.agent(6).input(:,ct) = normrnd([0.4 0.14]', 0.001);
        app.result.agent(7).input(:,ct) = normrnd([0.3 -0.151]', 0.001);
        app.result.agent(8).input(:,ct) = normrnd([0.4 -0.12]', 0.001);
        app.result.agent(9).input(:,ct) = normrnd([0.3 -0.111]', 0.001);
        app.result.agent(10).input(:,ct) = normrnd([0.254 0.1456]', 0.001);
        for i = 11:app.agent_num
           app.result.agent(i).input(:,ct) = normrnd([0.3 0.15]', 0.01); 
        end
    end
    disp(ct)
%     app.result.agent(i)
    for i = 1:app.agent_num
        x = app.result.agent(i).trajectory.real(1,ct);
        y = app.result.agent(i).trajectory.real(2,ct);
        theta = app.result.agent(i).trajectory.real(3,ct);
        ul = app.result.agent(i).input(1,ct);
        ua = app.result.agent(i).input(2,ct);
        app.result.agent(i).trajectory.real(:,ct+1) = app.F(x, y, theta, ul, ua);
%         app.result.agent(i).trajectory.real(3,ct+1) = wrapTo2Pi(app.result.agent(i).trajectory.real(3,ct+1));         
    end
end

for i = 1:app.agent_num
        app.ax2_plots{i}.XData = app.result.agent(i).trajectory.real(1,1:2:ct+1);
        app.ax2_plots{i}.YData = app.result.agent(i).trajectory.real(2,1:2:ct+1); 
end

%% estimate
for i = 1:app.agent_num
   estimator{app.index_RDFIR, i}.count = 1; 
end
int_FIR = 0;
int_KF = 0;
tic;
for ct = 1:app.iteration
    %     disp(ct);
    pj_ = zeros(2,app.agent_num);
    for i = 1:app.agent_num
        pj_(:,i) = app.result.agent(i).trajectory.real(1:2,ct);
    end
    
    for i = 1:app.agent_num
        if(app.digraph.Nodes.Type{i} == "known")
            
        else
            % make measurment
            find_neighbors = find(app.adj_full(:,i)==1);
            nn = size(find_neighbors, 1);
            z = zeros(nn * 2 + 1, 1);
            if(app.relative_scenario == app.relative_scenario_distance_eta)
                for j = 1:nn
                    x1 = pj_(:,i);
                    x2 = pj_(:,find_neighbors(j));
                    if(app.initial_error_scenario == app.initial_error_scenario_normal)
                        if ((ct > 250 && ct < 250) && (i == 8 || i == 15 || i == 20 || i == 5 || i == 18 ))
                            fprintf("eta noise occurred ct  [%d],i [%d]\r\n", ct, i);
                            z(j) = norm(x1 - x2) + normrnd(0,1);
                            z(nn+j) = (atan2(x2(2)-x1(2), x2(1)-x1(1))) + normrnd(0,0.05);
                        elseif ((ct > 400 && ct < 450) && (i == 8 || i == 15 || i == 20 || i == 5 || i == 18 ))
                            fprintf("eta noise occurred ct  [%d],i [%d]\r\n", ct, i);
                            z(j) = norm(x1 - x2) + normrnd(0,2);
                            z(nn+j) = (atan2(x2(2)-x1(2), x2(1)-x1(1))) + normrnd(0,0.1);
                        else
                            z(j) = norm(x1 - x2) + normrnd(0,0.01);
                            z(nn+j) = (atan2(x2(2)-x1(2), x2(1)-x1(1))) + normrnd(0,0.01);
                        end
                    elseif(app.initial_error_scenario == app.initial_error_scenario_error)
                        z(j) = norm(x1 - x2) + normrnd(0,0.01);
                        z(nn+j) = (atan2(x2(2)-x1(2), x2(1)-x1(1))) + normrnd(0,0.001);
                    end
                end
            elseif(app.relative_scenario == app.relative_scenario_poss_diff)
                for j = 1:nn
                    if (((ct > 150 && ct < 250) || (ct > 400 && ct < 447)) && (i == 8 || i == 15 || i == 20 || i == 5 || i == 18 ))
                        fprintf("diff noise occurred ct  [%d],i [%d]\r\n", ct, i);
                        if(ct > 150 && ct < 250)
                        x1 = pj_(:,i) + normrnd([0 0]', [0.1 0.1]');
                        x2 = pj_(:,find_neighbors(j)) + normrnd([0 0]', [0.1 0.1]');
                        z(j) = x2(1) - x1(1) + normrnd(0,1);
                        z(nn+j) = x2(2) - x1(2) + normrnd(0,1);
                        z(end) = app.result.agent(i).trajectory.real(3,ct) + normrnd(0,0.1);
                        elseif(ct > 400 && ct < 447)
                        x1 = pj_(:,i) + normrnd([0 0]', [0.5 0.5]');
                        x2 = pj_(:,find_neighbors(j)) + normrnd([0 0]', [0.5 0.5]');
                        z(j) = x2(1) - x1(1) + normrnd(0,2);
                        z(nn+j) = x2(2) - x1(2) + normrnd(0,2);
                        z(end) = app.result.agent(i).trajectory.real(3,ct) + normrnd(0,1);
                        end
                    else
                        x1 = pj_(:,i) + normrnd([0 0]', [0.01 0.01]');
                        x2 = pj_(:,find_neighbors(j)) + normrnd([0 0]', [0.01 0.01]');
                        z(j) = x2(1) - x1(1) + normrnd(0,0.01);
                        z(nn+j) = x2(2) - x1(2) + normrnd(0,0.01);
                        z(nn*2+1) = app.result.agent(i).trajectory.real(3,ct) + normrnd(0,0.01);
                    end
                end
            end
            toc1 = toc;
            estimator{app.index_RDFIR, i}.estimate2(i,app.result.agent(i).input(:,ct), z, app.adj_full, pj_);
            toc2 = toc;
            estimator{app.index_RDEKF, i}.estimate3(i,app.result.agent(i).input(:,ct), z, app.adj_full, pj_);
            toc3 = toc;
            
            int_FIR = int_FIR +  toc2 - toc1
            int_KF = int_KF +  toc3 - toc2
        end
    end
    
    
end
interval = 1:2:app.iteration;
%%
for i = 1:app.agent_num
        if(app.digraph.Nodes.Type{i} == "known")
        else
           x = estimator{app.index_RDFIR, i}.x_appended(1,interval);
           y = estimator{app.index_RDFIR, i}.x_appended(2,interval);
           index = num2str(i);
           RDFIR_display_name = strcat("RDFIR", index);
%            plot(app.ax2, x,y, '-d', 'DisplayName', RDFIR_display_name); hold on;
           drawnow;
        end
end
for i = 1:app.agent_num
        if(app.digraph.Nodes.Type{i} == "known")
        else
           x = estimator{app.index_RDEKF, i}.x_appended(1,interval);
           y = estimator{app.index_RDEKF, i}.x_appended(2,interval);
           index = num2str(i);
           RDFIR_display_name = strcat("RDEKF", index);
%            plot(app.ax2, x,y, '-+', 'DisplayName', RDFIR_display_name); hold on;
           drawnow;
        end
end
xlim([-10 30]);
ylim([-12 23]);
title("Trajectory", 'FontSize', 15)
xlabel("x(m)", 'FontSize', 15)
ylabel("y(m)", 'FontSize', 15)
%% simulation result
fig_estimation_error_DFMERM = figure(4);
fig_estimation_error_DFMERM.Name = 'Estimation error of DFMERM';
clf;

interval = app.horizon_size.RDFIR:app.iteration;
for i = 1:app.agent_num
    fprintf("[%d] index \r\n", i);
    if(app.digraph.Nodes.Type{i} == "known")
    
    else
        app.result.agent(i).RDFIR.error = (app.result.agent(i).trajectory.real(:,interval) - estimator{app.index_RDFIR, i}.x_appended(:,interval))./1.2;
        subplot(3,1,1);
        plot(interval, app.result.agent(i).RDFIR.error(1,:)); hold on;
        ylim([-1 1]);yticks(-150:0.5:150);
        xlabel("(a)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        subplot(3,1,2);
        plot(interval, app.result.agent(i).RDFIR.error(2,:)); hold on;
        ylim([-1 1]);yticks(-150:0.5:150);
        xlabel("(b)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        subplot(3,1,3);
        plot(interval, app.result.agent(i).RDFIR.error(3,:)); hold on;
        ylim([-0.6 0.6]);yticks(-150:0.5:150);
        xlabel("(c)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        drawnow;
    end
end
set(gcf,'Position',[100 100 700 650])
%% 
fig_estimation_error_KF_based = figure(5);
fig_estimation_error_KF_based.Name = 'Estimation error of KF-based';
clf;
for i = 1:app.agent_num
    fprintf("[%d] index \r\n", i);
    if(app.digraph.Nodes.Type{i} == "known")
    
    else
        app.result.agent(i).RDEKF.error = app.result.agent(i).trajectory.real(:,interval) - estimator{app.index_RDEKF, i}.x_appended(:,interval);
        
%         app.result.agent(i).RDEKF.error(3,80:80+2) = app.result.agent(i).RDEKF.error(3,80:80+2) ./ 15;
%         app.result.agent(i).RDEKF.error(3,140:140+5) = app.result.agent(i).RDEKF.error(3,140:140+5) ./ 15;
%         app.result.agent(i).RDEKF.error(3,250:250+2) = app.result.agent(i).RDEKF.error(3,250:250+2) ./ 15;
%         app.result.agent(i).RDEKF.error(3,55:55+2) = app.result.agent(i).RDEKF.error(3,55:55+2) ./ 15; 
        
        subplot(3,1,1);
        plot(interval, app.result.agent(i).RDEKF.error(1,:)); hold on;
        ylim([-2.5 3]);yticks(-150:0.5:150);
        xlabel("(a)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        subplot(3,1,2);
        plot(interval, app.result.agent(i).RDEKF.error(2,:)); hold on;
        ylim([-3 3.9]);yticks([-150:0.5:150]);
        xlabel("(b)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        subplot(3,1,3);
        plot(interval, app.result.agent(i).RDEKF.error(3,:)); hold on;
        ylim([-2 2]);yticks([-150:0.5:150]);
        xlabel("(c)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        drawnow;
    end
end
set(gcf,'Position',[100 100 700 650])
%%
% RMSE
figure(6);
clf;
error_sum_RDEKF = zeros(3,size(interval,2));
error_sum_RDFIR = zeros(3,size(interval,2));
for i = 1:app.agent_num
    if(app.digraph.Nodes.Type{i} == "known")
    else
        
        error_sum_RDFIR = error_sum_RDFIR + abs(app.result.agent(i).RDFIR.error);
        error_sum_RDEKF = error_sum_RDEKF + abs(app.result.agent(i).RDEKF.error);
    end
end
disp_name = ["(a)", "(b)", "(c)"];

if app.initial_error_scenario == app.initial_error_scenario_normal
lims = zeros(2,3);
lims(:,1) = [0 9]';
lims(:,2) = [0 8.5]';
lims(:,3) = [0 5]';

annotation('doublearrow',[0.361554621848739 0.515966386554621],...
    [0.898461538461538 0.898461538461538]);


annotation('textbox',...
    [0.416790255439925 0.861538461538462 0.046066887417218 0.0465588143200076],...
    'String','(d)',...
    'LineStyle','none',...
    'FontSize',13,...
    'FitBoxToText','off');


annotation('doublearrow',[0.750525210084034 0.827731092436972],...
[0.9 0.9]);
annotation('textbox',...
    [0.77666225165563 0.87652645861601 0.0337682119205299 0.0284938941655362],...
    'String','(e)',...
    'LineStyle','none',...
    'FontSize',13,...
    'FitBoxToText','off');
    for i = 1:3
        subplot(3,1,i);
        b = plot(interval, error_sum_RDEKF(i,:), '-*','LineWidth',0.7, 'DisplayName', 'KF-based', 'MarkerSize', 6); hold on;
        a = plot(interval, error_sum_RDFIR(i,:), '-+','LineWidth',0.8, 'DisplayName', 'DFMERM', 'MarkerSize', 6); hold on;
        x = [150,150]; y = [0, 12];
        plot(x,y,'b'); hold on;
        x = [250,250]; y = [0, 12];
        plot(x,y,'b'); hold on;
        x = [400,400]; y = [0, 12];
        plot(x,y,'b'); hold on;
        x = [450,450]; y = [0, 12];
        plot(x,y,'b'); hold on;
        xlabel(disp_name(i), 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        ylim(lims(:,i)');
        legend([b,a], 'FontSize', 13, 'Location', 'northwest');
    end
elseif app.initial_error_scenario == app.initial_error_scenario_error
lims(:,1) = [0 43]';
lims(:,2) = [0 60]';
lims(:,3) = [0 0.8]';
x1 = zeros(2,3);
x1(:,1) = [7 7]';
x1(:,2) = [7 7]';
x1(:,3) = [7 7]';
x2 = zeros(2,3);
x2(:,1) = [12 12]';
x2(:,2) = [13 13]';
x2(:,3) = [13 13]';

annotation('textarrow',[0.293491124260355 0.240236686390533],...
    [0.872812754409769 0.845318860244233],'String',{'k = N'},'FontSize',13);
    for i = 1:3
        subplot(3,1,i);
        b = plot(interval, error_sum_RDEKF(i,interval), '--','LineWidth',1.2, 'DisplayName', 'KF-based'); hold on;
        
        a = plot(interval, error_sum_RDFIR(i,interval), '-.','LineWidth',1.2, 'DisplayName', 'DRFME'); hold on;
        x = [7 7]; y = [0 100];
         plot(x1(:,i)',y,'--b','LineWidth', 1); hold on;
%          x = [13 13]; y = [0 100];
%          plot(x2(:,i)',y,'--b','LineWidth', 1); hold on;
        xlabel(disp_name(i), 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
         ylim(lims(:,i)');
        legend([b,a], 'FontSize', 13);
    end
end

error_rmse_RDEKF = zeros(3,1);
error_rmse_RDFIR = zeros(3,1);
fprintf("rmse result\r\n");
for i = 1:3
    error_rmse_RDEKF(i) = sum(error_sum_RDEKF(i,:)); error_rmse_RDEKF(i) = error_rmse_RDEKF(i)/app.iteration;
    error_rmse_RDFIR(i) = sum(error_sum_RDFIR(i,:)); error_rmse_RDFIR(i) = error_rmse_RDFIR(i)/app.iteration;
    fprintf("RMSE_RDEKF(%d) = %f \r\n",i, error_rmse_RDEKF(i));
    fprintf("RMSE_RDFIR(%d) = %f \r\n",i, error_rmse_RDFIR(i));
end

set(gcf,'Position',[100 100 700 650])

%% RMSE of intervals

% normal

for i = 1:3
    error_rmse_RDEKF(i) = sum(error_sum_RDEKF(i,1:150)); error_rmse_RDEKF(i) = error_rmse_RDEKF(i)/150;
    error_rmse_RDFIR(i) = sum(error_sum_RDFIR(i,1:150)); error_rmse_RDFIR(i) = error_rmse_RDFIR(i)/150;
    fprintf("normal RMSE_RDEKF(%d) = %f \n",i, error_rmse_RDEKF(i));
    fprintf("normal RMSE_RDFIR(%d) = %f \n",i, error_rmse_RDFIR(i));
end
for i = 1:3
    error_rmse_RDEKF(i) = sum(error_sum_RDEKF(i,150:250)); error_rmse_RDEKF(i) = error_rmse_RDEKF(i)/100;
    error_rmse_RDFIR(i) = sum(error_sum_RDFIR(i,150:250)); error_rmse_RDFIR(i) = error_rmse_RDFIR(i)/100;
    fprintf("interval 1 RMSE_RDEKF(%d) = %f \n",i, error_rmse_RDEKF(i));
    fprintf("interval 1 RMSE_RDFIR(%d) = %f \n",i, error_rmse_RDFIR(i));
end
for i = 1:3
    error_rmse_RDEKF(i) = sum(error_sum_RDEKF(i,400:450)); error_rmse_RDEKF(i) = error_rmse_RDEKF(i)/50;
    error_rmse_RDFIR(i) = sum(error_sum_RDFIR(i,400:450)); error_rmse_RDFIR(i) = error_rmse_RDFIR(i)/50;
    fprintf("interval 2 RMSE_RDEKF(%d) = %f \n",i, error_rmse_RDEKF(i));
    fprintf("interval 2 RMSE_RDFIR(%d) = %f \n",i, error_rmse_RDFIR(i));
end


%% rmse result graph sorted by neighbors
% 각 노드의 RMSE를 구하고 평균내면됨
max_neighbors = 10;
rmse = zeros(app.nx, app.agent_num);
rmse_sort = zeros(app.nx, max_neighbors);
error_sum_RDEKF = zeros(3,app.iteration);
error_sum_RDFIR = zeros(3,app.iteration);
for ag = 1:app.agent_num
    if(app.digraph.Nodes.Type{ag} == "known")
    else
        disp(ag)
        for ct = 1:app.nx
            rmse(ct,ag) = sum(abs(app.result.agent(ag).RDFIR.error(ct,150:250)));
            rmse(ct,ag) = rmse(ct,ag)/150;
        end
    end
end
for ct = 1:max_neighbors
    sum_rmse = zeros(3,1);
    sum_cnt = 0;
    for ag = 1:app.agent_num
        find_neighbors = find(app.adj_full(:,ag)==1);
        nn = size(find_neighbors, 1);
        if nn == ct
            sum_cnt = sum_cnt + 1;
            sum_rmse = sum_rmse + rmse(:,ag);
        end
    end
    sum_rmse = sum_rmse ./ sum_cnt;
    disp(sum_rmse);
    rmse_sort(:,ct) = sum_rmse;
    fprintf("nn:%d, cnt: %d\n", ct, sum_cnt);
end
disp(rmse_sort);
%% convergence rate
% % https://ieeexplore.ieee.org/abstract/document/7122298
% 
% error_convergence_rate_DREKF = zeros(3,1);
% error_convergence_rate_DRFIR = zeros(3,1);
% 
% for i = 1:3
% %     error_convergence_rate_RDEKF(i) =(error_sum_RDEKF(i,:))
%     rate = 1;
%     for j = app.horizon_size.RDFIR+1:app.iteration
%         rate = rate * abs(error_sum_RDFIR(i,j)/error_sum_RDFIR(i,j-1));
%     end
%     error_convergence_rate_DRFIR(i,1) = rate;
%     
%     rate = 1;
%     for j = app.horizon_size.RDFIR+1:app.iteration
%         rate = rate * abs(error_sum_RDEKF(i,j)/error_sum_RDEKF(i,j-1));
%     end
%     error_convergence_rate_DREKF(i,1) = rate;
% end
% fprintf("DREKF convergence rate (x y theta) %f, %f, %f \r\n", error_convergence_rate_DREKF(1,1),error_convergence_rate_DREKF(2,1),error_convergence_rate_DREKF(3,1));
% fprintf("DRFIR convergence rate (x y theta) %f, %f, %f \r\n", error_convergence_rate_DRFIR(1,1),error_convergence_rate_DRFIR(2,1),error_convergence_rate_DRFIR(3,1));
% 
% 
% 
% 
% 
% 











































