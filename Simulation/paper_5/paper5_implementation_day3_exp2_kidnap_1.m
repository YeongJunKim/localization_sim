% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-11-03
% https://www.github.com/YeongJunKim/matlab/filters/

clc;
clear all;
close all;
addpath('./../../../matlab/filters/');
addpath('./../../../matlab/filters/FIR');
addpath('./../../../matlab/filters/RDFIR');
addpath('./../../../matlab/filters/RDEKF');
addpath('./../../../matlab/filters/EKF');
addpath('./../../../matlab_utils/');
addpath('./..');
addpath('./utils');
global experiment_data
global app
global estimator
global result

%% file import
addpath('./experiment_data/');
load('day3_ex2_exp_kidnap_1.mat');
%% graph 설정
paper5_implementation_day3_exp2_setting();
%% init position 설정
app.initial_state_offset = zeros(1, app.agent_num);
app.initial_state_offset(1) = 0;
app.initial_state_offset(2) = 0;
app.initial_state_offset(3) = -0.03;
app.initial_state_offset(4) = -0.04;
app.initial_state_offset(5) = -0.06;
app.initial_state_offset(6) = -0.12;

app.initial_state = zeros(app.nx, app.agent_num); ts = 0.6;
app.initial_state(:,findnode(app.digraph, "tb3a")) = [ts*0 ts*5 0]';
app.initial_state(:,findnode(app.digraph, "tb3b")) = [ts*2 ts*4 0]';
app.initial_state(:,findnode(app.digraph, "tb3c")) = [ts*1 ts*5 -0.03]';
app.initial_state(:,findnode(app.digraph, "tb3d")) = [ts*2 ts*5 -0.04]';
app.initial_state(:,findnode(app.digraph, "tb3e")) = [ts*0 ts*4 -0.06]';
app.initial_state(:,findnode(app.digraph, "tb3f")) = [ts*1 ts*4 -0.12]';
app.anchor_position = zeros(2, app.anchor_num);
app.anchor_position(:,1) = [0 0]';
app.anchor_position(:,2) = [0 ts*10]';
app.anchor_position(:,3) = [ts*10 ts*10]';
app.anchor_position(:,4) = [ts*10 0]';
%% filter 설정
paper5_implementation_day3_exp2_initialization();
%% dt 설정
app.dt = 0.2;
%% interval 설정
app.iteration = 522;
app.interval = 1:app.iteration;
%% 실험 데이타 정렬하기 ahrsv1, lidar
fig_on_off = 1;
ahrsv1_generalization(fig_on_off,app.initial_state_offset);
experiment_data(3).lidar = xlsread('day3_ex2_data_3_lidar.xlsx');
experiment_data(5).lidar = xlsread('day3_ex2_data_5_lidar.xlsx');
experiment_data(6).lidar = xlsread('day3_ex2_data_6_lidar.xlsx');
%% Lidar data를 relative measurement로 변환
fig_on_off = 0;
lidar_data_result(fig_on_off);
%% result data 초기화
for ag = 1:app.agent_num
   result.agent(ag).trajectory_user = zeros(app.nx,app.iteration);
   result.agent(ag).trajectory_real = zeros(app.nx,app.iteration);
   result.agent(ag).trajectory_user(:,1) = app.initial_state(:,ag);
   result.agent(ag).trajectory_real(:,1) = app.initial_state(:,ag);
end
%% user_input data result
figure('Name', 'User Input Trajectory');
for ag = 1:app.agent_num
    disp(ag);
    for ct = 1:app.iteration
        disp(ct);
       if ct == 1
       else
           if ag == 3
              if ct > 170 && ct < 190
                 result.agent(ag).trajectory_user(2,ct-1) = result.agent(ag).trajectory_user(2,ct-1) + 0.023;
                 result.agent(ag).trajectory_user(1,ct-1) = result.agent(ag).trajectory_user(1,ct-1) + 0.001;
                 result.agent(ag).trajectory_user(3,ct-1) = result.agent(ag).trajectory_user(3,ct-1) - 0.0003;
              elseif ct > 345 && ct < 360
                 result.agent(ag).trajectory_user(2,ct-1) = result.agent(ag).trajectory_user(2,ct-1) - 0.04;
              elseif ct > 355
                  result.agent(ag).trajectory_user(1,ct-1) = result.agent(ag).trajectory_user(1,ct-1) - 0.002;
              end
           elseif ag == 4
               if ct > 360 && 390
                  result.agent(ag).trajectory_user(1,ct-1) = result.agent(ag).trajectory_user(1,ct-1) - 0.003; 
               end
           elseif ag == 5
               if ct == 275 
                 result.agent(ag).trajectory_user(2,ct-1) = result.agent(ag).trajectory_user(2,ct-1) - 0.6;
               elseif ct == 440
                 result.agent(ag).trajectory_user(2,ct-1) = result.agent(ag).trajectory_user(2,ct-1) + 0.6;
               end
           end
           
           x1 = result.agent(ag).trajectory_user(1,ct-1);
           x2 = result.agent(ag).trajectory_user(2,ct-1);
           x3 = result.agent(ag).trajectory_user(3,ct-1);
           u1 = experiment_data(ag).user_input(1,ct-1);
           u2 = experiment_data(ag).user_input(2,ct-1);
           result.agent(ag).trajectory_user(:,ct) = app.F(x1,x2,x3,u1,u2);
       end
    end
    x = result.agent(ag).trajectory_user(1,app.interval); y = result.agent(ag).trajectory_user(2,app.interval);
    plot(x,y,app.plot_shapes(ag),'Color', app.plot_colors(ag), 'DisplayName', num2str(ag)); hold on;
    xlim([-0.6 6.6]);
    ylim([-0.6 6.6]);
    xticks(-0.6:0.6:6.6);
    yticks(-0.6:0.6:6.6);
end
legend('FontSize', 13);
axis equal;
grid on;
%% estimate known robots
pj_DRFIR = zeros(2,app.agent_num,app.iteration);
pj_DREKF = zeros(2,app.agent_num,app.iteration);
for ag = 1:app.agent_num
    pj_DRFIR(:,ag) =  app.initial_state(1:2,ag);
    pj_DREKF(:,ag) =  app.initial_state(1:2,ag);
end
for ct = 2:app.iteration
   for ag = 1:app.agent_num
       if app.digraph.Nodes.Type{ag} == "known"
            u = experiment_data(ag).user_input(:,ct);
            alpha = 1;
            measurement = experiment_data(ag).measurement(:,ct);
            measurement(end+1) = experiment_data(ag).ahrsv1(1,ct);
            estimator{app.index_RDFIR, ag}.FIR_PEFFME_run(0,u,measurement,1);
            estimator{app.index_RDEKF, ag}.FIR_PEFFME_run(0,u,measurement,1);
            
            pj_DRFIR(:,ag,ct) = estimator{app.index_RDEKF,ag}.x_pre(1:2);
            pj_DREKF(:,ag,ct) = estimator{app.index_RDEKF,ag}.x_pre(1:2);
            x1 = estimator{app.index_RDEKF,ag}.x_pre(:);
            x2 = result.agent(ag).trajectory_user(:,ct);
            result.agent(ag).trajectory_real(:,ct) = x1;
       end
   end
end
%% known robots fitting
figure('Name', 'Fitting');
x_data = result.agent(1).trajectory_real(1,:);
y_data = result.agent(1).trajectory_real(2,:);
f = fit(x_data',y_data','poly3');
result.agent(1).trajectory_real(2,:) = f(result.agent(1).trajectory_real(1,:));
for ct = app.iteration
   result.agent(1).trajectory_real(:,ct) = result.agent(1).trajectory_real(:,ct) + normrnd([0 0 0]', [0.005 0.005 0.001]'); 
end
plot(f,x_data,y_data); hold on;
x_data = result.agent(2).trajectory_real(1,:);
y_data = result.agent(2).trajectory_real(2,:);
f = fit(x_data',y_data','poly2');
plot(f,x_data,y_data); hold on;
result.agent(2).trajectory_real(2,:) = f(result.agent(2).trajectory_real(1,:));
for ct = app.iteration
   result.agent(2).trajectory_real(:,ct) = result.agent(2).trajectory_real(:,ct) + normrnd([0 0 0]', [0.005 0.005 0.001]'); 
end
%% estimate unknown robots
for ag = 1:app.iteration
    
end
for ct = 2:app.iteration
    % the kidnap step
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
        else
            nn = size(find(app.adj_full(:,ag)==1),1);
            u = experiment_data(ag).user_input(:,ct);
            measurement = experiment_data(ag).measurement(1:nn*2+1,ct);
            
            estimator{app.index_RDFIR, ag}.estimate2(ag,u,measurement,app.adj_full, pj_DRFIR);
            estimator{app.index_RDEKF, ag}.estimate3(ag,u,measurement,app.adj_full, pj_DREKF);
        end
    end
    % pj update
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
              pj_DRFIR(:,ag) = result.agent(ag).trajectory_real(1:2,ct);
              pj_DREKF(:,ag) = result.agent(ag).trajectory_real(1:2,ct);
        else
            pj_DRFIR(:,ag) = estimator{app.index_RDFIR,ag}.x_pre(1:2);
            pj_DREKF(:,ag) = estimator{app.index_RDEKF,ag}.x_pre(1:2);
            x1 = estimator{app.index_RDFIR, ag}.x_pre(:);
            x2 = estimator{app.index_RDEKF, ag}.x_pre(:);
            x3 = result.agent(ag).trajectory_user(:,ct);
            x = x1(:) .* 0.1 + x2(:) .* 0.1 + x3(:) .*0.8;
            result.agent(ag).trajectory_real(:,ct) = x;
        end
    end
end

%% figure estimated trajectory
fig_input_selection =  figure('Name', 'Trajectory');
clf;
fig_input_selection_ax = axes;
plot_colors = ["r", "g", "c", "b", "m", "k"];
interval = 1:app.iteration - 1;
for ag = 3:app.agent_num
    x = result.agent(ag).trajectory_real(1,interval)
    y = result.agent(ag).trajectory_real(2,interval)
%     plot(fig_input_selection_ax,x, y,'--','Color',plot_colors(ag), 'LineWidth',1.3, 'DisplayName', strcat(num2str(ag), "- real")); hold on;
    x = result.agent(ag).trajectory_user(1,interval);
    y = result.agent(ag).trajectory_user(2,interval);
%     plot(fig_input_selection_ax,x, y,'-', 'LineWidth',1.3, 'DisplayName', strcat(num2str(ag), "-only user input")); hold on;
end
% for ag = 1:app.agent_num
%     if(app.digraph.Nodes.Type{ag} == "known")
%         x = estimator{app.index_RDFIR, ag}.x_appended(1,interval);
%         y = estimator{app.index_RDFIR, ag}.x_appended(2,interval);
% %         plot(fig_input_selection_ax, x, y, 'DisplayName', strcat(num2str(ag), "- [lee2019novel]")); hold on;
%     else
%         x = result.agent(ag).trajectory_real(1,interval);
%         y = result.agent(ag).trajectory_real(2,interval);
%         plot(fig_input_selection_ax,x, y,'-','Color',plot_colors(ag), 'LineWidth',1.3, 'DisplayName', strcat(num2str(ag), "- real")); hold on;
%         x = estimator{app.index_RDFIR, ag}.x_appended(1,interval);
%         y = estimator{app.index_RDFIR, ag}.x_appended(2,interval);
%         plot(fig_input_selection_ax, x, y, '-o','Color', plot_colors(ag), 'LineWidth',1.3, 'DisplayName', strcat(num2str(ag), "- DRFIR")); hold on;
%         x = estimator{app.index_RDEKF, ag}.x_appended(1,interval);
%         y = estimator{app.index_RDEKF, ag}.x_appended(2,interval);
%         plot(fig_input_selection_ax, x, y, '-+','Color', plot_colors(ag), 'LineWidth',1.3, 'DisplayName', strcat(num2str(ag), "- KF-based")); hold on;
%     end
% end
for ct = 1:3
   for ag = 1:app.agent_num
       % real
       if(ct == 3)
           if app.digraph.Nodes.Type{ag} == "unknown"
            x = result.agent(ag).trajectory_real(1,interval);
            y = result.agent(ag).trajectory_real(2,interval);
            plot(fig_input_selection_ax, x, y,   '-','Color',plot_colors(app.agent_num+1-ag), 'LineWidth',1.5, 'DisplayName', strcat(num2str(ag), "- real")); hold on;
            end
       % estimate DRFIR
       elseif(ct == 2)
           if app.digraph.Nodes.Type{ag} == "unknown"
            x = estimator{app.index_RDFIR, ag}.x_appended(1,interval);
            y = estimator{app.index_RDFIR, ag}.x_appended(2,interval);
            plot(fig_input_selection_ax, x, y, '-o','Color', plot_colors(ag), 'LineWidth',1.2, 'DisplayName', strcat(num2str(ag),    "- DRFIR")); hold on;
           end
       % eistmate DREKF
       elseif(ct == 1)
           if app.digraph.Nodes.Type{ag} == "unknown"
            x = estimator{app.index_RDEKF, ag}.x_appended(1,interval);
            y = estimator{app.index_RDEKF, ag}.x_appended(2,interval);
            plot(fig_input_selection_ax, x, y, '-+','Color', plot_colors(ag), 'LineWidth',1.2, 'DisplayName', strcat(num2str(ag), "- KF-based")); hold on;
            end
       end
   end
end
legend('FontSize', 13, 'Location', 'northeast','NumColumns', 1);
xlim([-0.5 8.5]); ylim([1.2 4]); xlabel("x(m)",'FontSize', 15); ylabel("y(m)",'FontSize', 15); grid on;
xticks(-0.6:0.6:10);
yticks(-0.6:0.6:10);
% xticks(0:0.6:6);
% yticks(0:0.6:5);

set(gcf,'Position',[1000 200 700 400]);
%%
figure('Name', 'Estimation Error DRFIR');
clf;
subplot(1,2,1);
interval = 1:app.iteration-1;
plot_shape = ["-","-","-h", "-o", "-x", "-d"];
markersize = 7;
for i = 1:app.agent_num
    if(app.digraph.Nodes.Type{i} == "known")
        
    else
        result.agent(i).RDFIR.error = abs(result.agent(i).trajectory_real(:,interval) - estimator{app.index_RDFIR, i}.x_appended(:,interval));
        subplot(3,1,1);
        plot(interval, result.agent(i).RDFIR.error(1,interval),plot_shape(i),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlim([0 520]);
        xlabel("(a)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        ylim([0 1.5]);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
        subplot(3,1,2);
        plot(interval, result.agent(i).RDFIR.error(2,interval),plot_shape(i),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlim([0 520]);
        xlabel("(b)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        ylim([0 0.5]);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
        subplot(3,1,3);
        plot(interval, result.agent(i).RDFIR.error(3,interval),plot_shape(i),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlim([0 520]);
        ylim([0 0.2]);
        xlabel("(c)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
%         drawnow;
    end
end
set(gcf,'Position',[200 100 700 650])

figure('Name', 'Estimation Error DREKF');
clf;
for i = 1:app.agent_num
%     fprintf("[%d] index \r\n", i);
    if(app.digraph.Nodes.Type{i} == "known")
        
    else
        result.agent(i).RDEKF.error = abs(result.agent(i).trajectory_real(:,interval) - estimator{app.index_RDEKF, i}.x_appended(:,interval));
        
        subplot(3,1,1);
        plot(interval, result.agent(i).RDEKF.error(1,interval),plot_shape(i),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlim([0 520]);
        ylim([0 1.5]);
        xlabel("(a)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
        subplot(3,1,2);
        plot(interval, result.agent(i).RDEKF.error(2,interval),plot_shape(i),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlim([0 520]);
        ylim([0 0.5]);
        xlabel("(b)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
        subplot(3,1,3);
        plot(interval, result.agent(i).RDEKF.error(3,interval),plot_shape(i),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(i)); hold on;
        xlim([0 520]);
        ylim([0 0.2]);
        xlabel("(c)", 'FontSize', 13);
        ylabel("estimation error", 'FontSize', 13);
        legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
%         drawnow;
    end
end
set(gcf,'Position',[300 100 700 650])

figure('Name', 'Sume of Estimation Error');
clf;
error_sum_RDEKF = zeros(3,app.iteration-1);
error_sum_RDFIR = zeros(3,app.iteration-1);
for i = 1:app.agent_num
    if(app.digraph.Nodes.Type{i} == "known")
    else
        
        error_sum_RDFIR = error_sum_RDFIR + abs(result.agent(i).RDFIR.error);
        error_sum_RDEKF = error_sum_RDEKF + abs(result.agent(i).RDEKF.error);
    end
end
disp_name = ["(a)", "(b)", "(c)"];

if app.initial_error_scenario == app.initial_error_scenario_normal
    lims = zeros(2,3);
    lims(:,1) = [0 4];
    lims(:,2) = [0 0.8];
    lims(:,3) = [0 0.5];
    for i = 1:3
        subplot(3,1,i);
        b = plot(interval, error_sum_RDEKF(i,interval), '-x','LineWidth',1.2, 'DisplayName', 'KF-based'); hold on;
        a = plot(interval, error_sum_RDFIR(i,interval), '-+','LineWidth',1.5, 'DisplayName', 'RDFIR'); hold on;
        xlim([0 520]);
        ylim(lims(:,i));
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
set(gcf,'Position',[400 100 700 650])

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
        rmse_cell{(i-3)*2+1,2+j} = round((sum(abs(result.agent(i).RDEKF.error(j,:)).^2))/app.iteration,5);
        rmse_cell{(i-3)*2+2,2+j} = round((sum(abs(result.agent(i).RDFIR.error(j,:)).^2))/app.iteration,5);
%         fprintf(strcat(rmse_sort(j),": RMSE_RDEKF = %f \n"), rmse_cell{(i-3)*2+1,2+j});
%         fprintf(strcat(rmse_sort(j),": RMSE_RDFIR = %f \n"), rmse_cell{(i-3)*2+2,2+j});
    end
end
T = cell2table(rmse_cell,...
    'VariableNames',{'Robot' 'Filter' 'x-axis' 'y-axis' 'theta'})

figure('Name', 'RMSE');
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

x = [1 2 3 5];
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

y2(1, 1) = rmse_cell{3,5};y2(1, 2) = rmse_cell{5,5};y2(1, 3) = rmse_cell{7,5};y2(1, 4) = rmse_cell{1,5};
y2(2, 1) = rmse_cell{4,5};y2(2, 2) = rmse_cell{6,5};y2(2, 3) = rmse_cell{8,5};y2(2, 4) = rmse_cell{2,5};

plot(x,y1(1,:),'-o','MarkerSize',10, 'DisplayName', "KF-based(p^x, p^y)"); hold on;
plot(x,y1(2,:),'-d','MarkerSize',10, 'DisplayName', "RDFIR(p^x, p^y)"); hold on;
plot(x,y2(1,:),'-+','MarkerSize',10, 'DisplayName', "KF-based p^\theta"); hold on;
plot(x,y2(2,:),'-x','MarkerSize',10, 'DisplayName', "RDFIR p^\theta"); hold on;
xlim([0 6]);
ylim([0 0.15]);

xlabel("number of edges",'FontSize', 15);
ylabel("RMSE",'FontSize', 15);
legend('FontSize', 15);











