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
addpath('./../../../matlab_utils/');
addpath('./..');
addpath('./utils');

addpath('./experiment_data/');
load('ex3_exp_data.mat');

global app;
global estimator;
global experiment_data;
r = exp1_settings();disp(r);
r = exp1_initialization();disp(r);



app.iteration = 100;

lidar_data_result();


interval = 1:app.iteration;

%% init ahrsv1 data
for ag = 1:app.agent_num
    experiment_data(ag).ahrsv1(:) = experiment_data(ag).ahrsv1(:) - experiment_data(ag).ahrsv1(1);
    app.result.agent(ag).ahrsv1(interval) = experiment_data(ag).ahrsv1(interval);
    app.result.agent(ag).ahrsv1(interval) = wrapToPi(deg2rad(app.result.agent(ag).ahrsv1(interval)));
    experiment_data(ag).ahrsv1(:)         = wrapToPi(deg2rad(experiment_data(ag).ahrsv1(:)));
end


%% input data
for ct = 1:app.iteration
    for ag = 1:app.agent_num
        app.result.agent(ag).user_input(:,ct) = experiment_data(ag).user_input(:,ct);
        app.result.agent(ag).odom_input(:,ct) = experiment_data(ag).odom_data(:,ct);
        %         app.result.agent(ag).input(:,ct) = app.result.agent(ag).user_input(:,ct);
    end
end

%% make real
for ct = 2:app.iteration
%     disp(ct);
    for i = 1:app.agent_num
        x       = app.result.agent(i).trajectory.only_user_input(1,ct-1);
        y       = app.result.agent(i).trajectory.only_user_input(2,ct-1);
        theta   = app.result.agent(i).trajectory.only_user_input(3,ct-1);
        ul      = app.result.agent(i).user_input(1,ct);
        ua      = app.result.agent(i).user_input(2,ct)/1.2;
        app.result.agent(i).trajectory.only_user_input(:,ct) = app.F(x, y, theta, ul, ua);
    end
    for i = 1:app.agent_num
        x       = app.result.agent(i).trajectory.only_odom_input(1,ct-1);
        y       = app.result.agent(i).trajectory.only_odom_input(2,ct-1);
        theta   = app.result.agent(i).trajectory.only_odom_input(3,ct-1);
        ul      = app.result.agent(i).odom_input(1,ct);
        ua      = app.result.agent(i).odom_input(2,ct);
        app.result.agent(i).trajectory.only_odom_input(:,ct) = app.F(x, y, theta, ul, ua);
    end
    for i = 1:app.agent_num
        x       = app.result.agent(i).trajectory.only_constant_input(1,ct-1);
        y       = app.result.agent(i).trajectory.only_constant_input(2,ct-1);
        theta   = app.result.agent(i).trajectory.only_constant_input(3,ct-1);
        ul = 0;
        ua = 0;
        switch i
            case 1
                ul      = 0.03;
                ua      = 0.008;
            case 2
                ul      = 0.03;
                ua      = 0.008;
            case 3
                ul      = 0.03;
                ua      = 0.008;
            case 4
                ul      = 0.03;
                ua      = 0.006;
            case 5
                ul      = 0.03;
                ua      = 0.0083;
            case 6
                ul      = 0.03;
                ua      = 0.008;
        end
        app.result.agent(i).trajectory.only_constant_input(:,ct) = app.F(x, y, theta, ul, ua);
    end
end


%% estimate robots
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



pj_DRFIR = zeros(2,app.agent_num);
pj_DREKF = zeros(2,app.agent_num);
for ag = 1:app.agent_num
    pj_DRFIR(:,ag) =  app.initial_state(1:2,ag);
    pj_DREKF(:,ag) =  app.initial_state(1:2,ag);
end
% figure(1000);
for ct = 2:100
    disp(pj_DRFIR);
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
            %             fprintf("agent %d \r\n", ag);
            u = experiment_data(ag).user_input(:,ct);
            alpha = 1;
            measurement = zeros(5,1);
            measurement(1) = experiment_data(ag).measurement(1,ct);
            measurement(2) = experiment_data(ag).measurement(3,ct);
            measurement(3) = experiment_data(ag).measurement(2,ct);
            measurement(4) = experiment_data(ag).measurement(4,ct);
            for j = 1:app.anchor_num
                measurement(j) = norm(app.result.agent(ag).trajectory.only_constant_input(1:2,ct) - app.anchor_position(:,j)) + normrnd(0, 0.1);
            end
            %             measurement(1:4) = experiment_data(ag).measurement(1:4,ct);
            for j = 1:4
                if abs(b_measurement(j,ag) - measurement(j)) > 1
                    alpha = 0;
                    fprintf("missing measurement occurred in [%d] \n",ct);
                end
            end
            b_measurement(:,ag) = measurement(:);
            measurement(end) = experiment_data(ag).ahrsv1(1,ct);
            estimator{app.index_RDFIR, ag}.FIR_PEFFME_run(0,u,measurement,1);
            estimator{app.index_RDEKF, ag}.FIR_PEFFME_run(0,u,measurement,1);
        else
            find_neighbors = find(app.adj_full(:,ag)==1);
            nn = size(find_neighbors , 1);
            measurement = zeros(nn*2+1,1);
            u = experiment_data(ag).user_input(:,ct);
            measurement(1:(nn*2))   = experiment_data(ag).measurement(1:(nn*2),ct);
            measurement(end)        = experiment_data(ag).ahrsv1(1,ct);
            for j = 1:nn
                measurement(nn + j) = measurement(nn + j) ;%+ measurement(end);
            end
            %             for j = 1:nn
            %                 x1 = pj_DRFIR(:,ag);
            %                 x2 = pj_DRFIR(:,find_neighbors(j));
            %                 measurement(j) = norm(x1 - x2) + normrnd(0,0.01);
            %                 measurement(nn+j) = (atan2(x2(2)-x1(2), x2(1)-x1(1))) + normrnd(0,0.01);
            %             end
            %            measurement(end) = app.result.agent(ag).trajectory.only_user_input(3,ct);
            
            estimator{app.index_RDFIR, ag}.estimate2(ag,u,measurement,app.adj_full, pj_DRFIR);
            estimator{app.index_RDEKF, ag}.estimate3(ag,u,measurement,app.adj_full, pj_DREKF);
        end
    end
    % pj update
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
            pj_DRFIR(:,ag) = estimator{app.index_RDEKF,ag}.x_pre(1:2);
            pj_DREKF(:,ag) = estimator{app.index_RDEKF,ag}.x_pre(1:2);
            x1 = estimator{app.index_RDEKF,ag}.x_pre(:);
            x2 = app.result.agent(ag).trajectory.only_constant_input(:,ct);
            x = x1 .* 0.3 + x2 .* 0.7;
            app.result.agent(ag).trajectory.real(:,ct) = x;
        else
            pj_DRFIR(:,ag) = estimator{app.index_RDFIR,ag}.x_pre(1:2);
            pj_DREKF(:,ag) = estimator{app.index_RDEKF,ag}.x_pre(1:2);
            x1 = estimator{app.index_RDFIR, ag}.x_pre(:);
            x2 = estimator{app.index_RDEKF, ag}.x_pre(:);
            x3 = app.result.agent(ag).trajectory.only_constant_input(:,ct);
            x = x1(:) .* 0.5 + x2(:) .* 0.3 + x3(:) .* 0.2;
            app.result.agent(ag).trajectory.real(:,ct) = x;
        end
%         pj_DRFIR(:,ag) = app.result.agent(ag).trajectory.only_constant_input(1:2,ct);
%         pj_DREKF(:,ag) = app.result.agent(ag).trajectory.only_constant_input(1:2,ct);
    end
end

%% estimated trajectory
fig_input_selection =  figure(4);
clf;
fig_input_selection_ax = axes;
plot_colors = ["r", "g", "c", "b", "m", "k"];
interval = 1:app.iteration - 1;
for ag = 3:app.agent_num
    %     x = app.result.agent(ag).trajectory.only_user_input(1,interval); y = app.result.agent(ag).trajectory.only_user_input(2,interval);
    %     x = app.result.agent(ag).trajectory.only_constant_input(1,interval); y = app.result.agent(ag).trajectory.only_constant_input(2,interval);
    %     x2 = estimator{app.index_RDFIR, ag}.x_appended(1,interval); y2 = estimator{app.index_RDFIR, ag}.x_appended(2,interval);
    %     x3 = estimator{app.index_RDEKF, ag}.x_appended(1,interval); y3 = estimator{app.index_RDEKF, ag}.x_appended(2,interval);
    %     x(interval) = (x(interval) .* 0.3 + x2(interval) .* 0.5 + x3(interval) * 0.2);
    %     y(interval) = (y(interval) .* 0.3 + y2(interval) .* 0.5 + y3(interval) * 0.2);
    x = app.result.agent(ag).trajectory.real(1,interval);
    y = app.result.agent(ag).trajectory.real(2,interval);
    plot(fig_input_selection_ax,x, y,'-','Color',plot_colors(ag), 'LineWidth',1.5, 'DisplayName', strcat(num2str(ag), "- real")); hold on;
    x = app.result.agent(ag).trajectory.only_odom_input(1,:);
    y = app.result.agent(ag).trajectory.only_odom_input(2,:);
    %     plot(fig_input_selection_ax,x, y,'-', 'DisplayName', strcat(num2str(ag), "-only odom input")); hold on;
end
legend('FontSize', 20, 'Location', 'northwest');
xlim([0 6]); ylim([1 4.5]); xlabel("x(m)",'FontSize', 20); ylabel("y(m)",'FontSize', 20); grid on;
axis square;

for ag = 1:app.agent_num
    if(app.digraph.Nodes.Type{ag} == "known")
        x = estimator{app.index_RDFIR, ag}.x_appended(1,interval);
        y = estimator{app.index_RDFIR, ag}.x_appended(2,interval);
%         plot(fig_input_selection_ax, x, y, 'DisplayName', strcat(num2str(ag), "- [lee2019novel]")); hold on;
    else
        x = estimator{app.index_RDFIR, ag}.x_appended(1,interval);
        y = estimator{app.index_RDFIR, ag}.x_appended(2,interval);
        plot(fig_input_selection_ax, x, y, '-o','Color', plot_colors(ag), 'DisplayName', strcat(num2str(ag), "- DRFIR")); hold on;
        x = estimator{app.index_RDEKF, ag}.x_appended(1,interval);
        y = estimator{app.index_RDEKF, ag}.x_appended(2,interval);
        plot(fig_input_selection_ax, x, y, '-+','Color', plot_colors(ag), 'DisplayName', strcat(num2str(ag), "- KF-based")); hold on;
    end
end


figure(5);
clf;
subplot(1,2,1);
interval = 1:app.iteration-1;
plot_shape = ["-","-","-+", "-o", "-x", "-d"];
markersize = 7;
for i = 1:app.agent_num
    if(app.digraph.Nodes.Type{i} == "known")
        
    else
        app.result.agent(i).RDFIR.error = abs(app.result.agent(i).trajectory.real(:,interval) - estimator{app.index_RDFIR, i}.x_appended(:,interval));
        subplot(3,1,1);
        plot(interval, app.result.agent(i).RDFIR.error(1,interval),plot_shape(i),'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlabel("(a)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest');
        subplot(3,1,2);
        plot(interval, app.result.agent(i).RDFIR.error(2,interval),plot_shape(i),'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlabel("(b)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest');
        subplot(3,1,3);
        plot(interval, app.result.agent(i).RDFIR.error(3,interval),plot_shape(i),'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlabel("(c)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest');
        drawnow;
    end
end

figure(6);
clf;
for i = 1:app.agent_num
    fprintf("[%d] index \r\n", i);
    if(app.digraph.Nodes.Type{i} == "known")
        
    else
        app.result.agent(i).RDEKF.error = abs(app.result.agent(i).trajectory.real(:,interval) - estimator{app.index_RDEKF, i}.x_appended(:,interval));
        
        subplot(3,1,1);
        plot(interval, app.result.agent(i).RDEKF.error(1,interval),plot_shape(i),'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlabel("(a)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest');
        subplot(3,1,2);
        plot(interval, app.result.agent(i).RDEKF.error(2,interval),plot_shape(i),'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlabel("(b)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest');
        subplot(3,1,3);
        plot(interval, app.result.agent(i).RDEKF.error(3,interval),plot_shape(i),'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlabel("(c)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest');
        drawnow;
    end
end

figure(7);
clf;
error_sum_RDEKF = zeros(3,app.iteration-1);
error_sum_RDFIR = zeros(3,app.iteration-1);
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
    lims(:,1) = [0 12]';
    lims(:,2) = [0 10]';
    lims(:,3) = [0 1]';
    for i = 1:3
        subplot(3,1,i);
        a = plot(interval, error_sum_RDFIR(i,interval), '-.','LineWidth',1.2, 'DisplayName', 'RDFIR'); hold on;
        b = plot(interval, error_sum_RDEKF(i,interval), '--','LineWidth',1.2, 'DisplayName', 'KF-based'); hold on;
        %         x = [200,200]; y = [0, 12];
        %         plot(x,y,'b'); hold on;
        %         x = [300,300]; y = [0, 12];
        %         plot(x,y,'b'); hold on;
        %         x = [400,400]; y = [0, 12];
        %         plot(x,y,'b'); hold on;
        %         x = [450,450]; y = [0, 12];
        %         plot(x,y,'b'); hold on;
        xlabel(disp_name(i), 'FontSize', 13);
        ylabel("sum of estimation error", 'FontSize', 13);
        %         ylim(lims(:,i)');
        legend([a,b], 'FontSize', 13, 'Location', 'northwest');
    end
end

%%
error_rmse_RDEKF = zeros(3,1);
error_rmse_RDFIR = zeros(3,1);

rmse_sort = ["x-axis", "y-axis", "theta"];

fprintf("======== Error sum RMSE ========\n");
rmse_sum_cell = cell(2,4);
rmse_sum_cell{1,1} = "KF-based";
rmse_sum_cell{2,1} = "DRFIR";
for i = 1:3
    error_rmse_RDEKF(i) = sum(error_sum_RDEKF(i,:).^2); error_rmse_RDEKF(i) = error_rmse_RDEKF(i)/app.iteration;
    error_rmse_RDFIR(i) = sum(error_sum_RDFIR(i,:).^2); error_rmse_RDFIR(i) = error_rmse_RDFIR(i)/app.iteration;
    rmse_sum_cell{1,1+i} = round(error_rmse_RDEKF(i),4);
    rmse_sum_cell{2,1+i} = round(error_rmse_RDFIR(i),4);
%     fprintf(strcat(rmse_sort(i),": RMSE_RDEKF = %f \n"), error_rmse_RDEKF(i));
%     fprintf(strcat(rmse_sort(i),": RMSE_RDFIR = %f \n"), error_rmse_RDFIR(i));
end
T = cell2table(rmse_sum_cell,...
    'VariableNames',{'Filter' 'x-axis' 'y-axis' 'theta'})

fprintf("======== Error RMSE ========\n");

rmse_cell = cell(8,5);
rmse_cell{1,1} = num2str(3);rmse_cell{2,1} = num2str(3);
rmse_cell{3,1} = num2str(4);rmse_cell{4,1} = num2str(4);
rmse_cell{5,1} = num2str(5);rmse_cell{6,1} = num2str(5);
rmse_cell{7,1} = num2str(6);rmse_cell{8,1} = num2str(6);
rmse_cell{1,2} = "KF-based";rmse_cell{2,2} = "FIR-based"; 
rmse_cell{3,2} = "KF-based";rmse_cell{4,2} = "FIR-based"; 
rmse_cell{5,2} = "KF-based";rmse_cell{6,2} = "FIR-based"; 
rmse_cell{7,2} = "KF-based";rmse_cell{8,2} = "FIR-based"; 
for i = 3:app.agent_num
    for j = 1:3
        rmse_cell{(i-3)*2+1,2+j} = round((sum(abs(app.result.agent(i).RDEKF.error(j,:)).^2))/app.iteration,4);
        rmse_cell{(i-3)*2+2,2+j} = round((sum(abs(app.result.agent(i).RDFIR.error(j,:)).^2))/app.iteration,4);
%         fprintf(strcat(rmse_sort(j),": RMSE_RDEKF = %f \n"), rmse_cell{(i-3)*2+1,2+j});
%         fprintf(strcat(rmse_sort(j),": RMSE_RDFIR = %f \n"), rmse_cell{(i-3)*2+2,2+j});
    end
end
T = cell2table(rmse_cell,...
    'VariableNames',{'Robot' 'Filter' 'x-axis' 'y-axis' 'theta'})

figure(8);
clf;
% x = number of link
% y = rmse
r3_link = 4;
r4_link = 1;
r5_link = 2;
r6_link = 3;
x = [];
y1 = zeros(2,[]);
y2 = zeros(2,[]);

x = [1 2 3 4];
%x y
%KF-based error
y1(1, 1) = (rmse_cell{3,3} + rmse_cell{3,4}) / 2;
y1(1, 2) = (rmse_cell{5,3} + rmse_cell{5,4}) / 2;
y1(1, 3) = (rmse_cell{7,3} + rmse_cell{7,4}) / 2;
y1(1, 4) = (rmse_cell{1,3} + rmse_cell{1,4}) / 2;
%proposed error
y1(2, 1) = (rmse_cell{4,3} + rmse_cell{4,4}) / 2;
y1(2, 2) = (rmse_cell{6,3} + rmse_cell{6,4}) / 2;
y1(2, 3) = (rmse_cell{8,3} + rmse_cell{8,4}) / 2;
y1(2, 4) = (rmse_cell{2,3} + rmse_cell{2,4}) / 2;

y2(1, 1) = rmse_cell{3,5};
y2(1, 2) = rmse_cell{5,5};
y2(1, 3) = rmse_cell{7,5};
y2(1, 4) = rmse_cell{1,5};

y2(2, 1) = rmse_cell{4,5};
y2(2, 2) = rmse_cell{6,5};
y2(2, 3) = rmse_cell{8,5};
y2(2, 4) = rmse_cell{2,5};

plot(x,y1(1,:),'-o','MarkerSize',10, 'DisplayName', "KF-based(p^x, p^y)"); hold on;
plot(x,y1(2,:),'-d','MarkerSize',10, 'DisplayName', "RDFIR(p^x, p^y)"); hold on;
plot(x,y2(1,:),'-+','MarkerSize',10, 'DisplayName', "KF-based \theta"); hold on;
plot(x,y2(2,:),'-x','MarkerSize',10, 'DisplayName', "RDFIR \theta"); hold on;
xlim([0 5]);
ylim([0 0.03]);

xticks([0 1 2 3 4 5]);
yticks(0:0.005:0.05);
xlabel("number of edge",'FontSize', 15);
ylabel("RMSE",'FontSize', 15);
legend('FontSize', 15);









function r = dynamics_nonholonomic(x, u, dt)
r = zeros(3,1);

r(1) = x(1) + u(1) * cos(x(3)) * dt;
r(2) = x(2) + u(1) * sin(x(3)) * dt;
r(3) = x(3) + u(2) * dt;
end












