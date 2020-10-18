% paper: Distributed Localization Using Finite Impulse Response Filter.
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-09-08
% EKFDCL class is in
% https://www.github.com/YeongJunKim/matlab/filters/

clear all;

addpath('./../../../matlab/filters/');
addpath('./../../../matlab/filters/DFIR');
addpath('./../../../matlab/filters/EKF');
addpath('./..');
global app;

make_video = 1;
absolute_on = 1;

%% invironment setup
app.agent_num = 5;
app.nx = 3;
app.nu = 2;
app.nh = 7;
app.nz = 7;

app.initial_state = zeros(app.nx * app.agent_num, 1);
app.adjacency = zeros(app.agent_num, app.agent_num);


% ramdom state initialization
for ct = 1:app.agent_num
    app.initial_state((ct-1)*app.nx+1:(ct-1)*app.nx+app.nx, 1) = rand(3,1);
    app.initial_state((ct-1)*app.nx+1,1) = app.initial_state((ct-1)*app.nx+1,1) * 2;
    app.initial_state((ct-1)*app.nx+2,1) = app.initial_state((ct-1)*app.nx+1,1) * normrnd(0, 0.3);
    % same direction
    app.initial_state((ct-1)*app.nx+3,1) = 0;
end

app.adjacency(1,[2, 4]) = 1;
app.adjacency(2,3) = 1;
app.adjacency(3,[1, 2, 5]) = 1;
app.adjacency(4,[1, 2, 5]) = 1;
app.adjacency(5,[3, 4]) = 1;
app.anchor_pos = zeros(2,4);
app.anchor_pos(:,1) = [0 0]';
app.anchor_pos(:,2) = [0 5]';
app.anchor_pos(:,3) = [5 5]';
app.anchor_pos(:,4) = [5 0]';

app.property = ["known", "known", "unknown", "unknown", "known"];

for ct = 1:app.agent_num
    app.data.agent(ct).property = app.property(ct);
    app.data.agent(ct).initial_state = zeros(app.nx, 1);
    app.data.agent(ct).input = zeros(app.nu, []);
    app.data.agent(ct).measurement = zeros(app.nz, []);
    app.data.agent(ct).trajectory.real = zeros(app.nx, []);
    app.data.agent(ct).trajectory.estimated = zeros(app.nx, []);
    app.data.agent(ct).trajectory.se =  zeros(app.nx, []);
    app.data.agent(ct).trajectory.rmse = 0;
    app.data.agent(ct).initial_state(:,1) = app.initial_state((ct-1)*app.nx+1:(ct-1)*app.nx+app.nx, 1);
    app.data.agent(ct).trajectory.real(:, 1) = app.initial_state((ct-1)*app.nx+1:(ct-1)*app.nx+app.nx, 1);
    app.data.agent(ct).trajectory.estimated(:, 1) = app.initial_state((ct-1)*app.nx+1:(ct-1)*app.nx+app.nx, 1);
end

%% initicating plots
agent_names = "agent";
figure(1);
clf;
ax1 = axes;
ax1_plots = cell(app.agent_num, 1);

for ct = 1:app.agent_num
    ax1_plots{ct} = plot(ax1, app.initial_state((ct-1)*app.nx+1),app.initial_state((ct-1)*app.nx+1)+1, '*'); hold on;
end
xlim([-10 10]);
ylim([-10 10]);
xlabel("x(m)");
ylabel("y(m)");
hold off;

figure(2);
clf;
ax2 = axes;
ax2_plots = cell(app.agent_num, 1);

for ct = 1:app.agent_num
    agent_plot_name = strcat(agent_names, num2str(ct));
    if ct == 3 || ct == 4
    agent_plot_name = strcat(agent_plot_name, "(unknown)");
    else
    agent_plot_name = strcat(agent_plot_name, "(known)");
    end
    ax2_plots{ct} = plot(ax2, app.initial_state((ct-1)*app.nx+1),app.initial_state((ct-1)*app.nx+1)+1, '*', 'DisplayName', agent_plot_name); hold on;
end
legend;
% xlim([-10 10]);
% ylim([-10 10]);
xlabel("x(m)", 'FontSize', 12);
ylabel("y(m)", 'FontSize', 12);
title("trajectory", 'FontSize', 13);
hold on;


%% simulation init
app.dt = 0.1;
app.iteration = 140;

[app.F, app.J_F] = dynamics_nonholonomic(app.dt);
[app.H, app.J_H] = measurement_update(app.anchor_pos);
[app.R, app.J_R] = relative_measurement_update2();

app.cp_targets = 2;
app.INDEX_DFIR = 1;
app.INDEX_EKF = 2;

estimator = cell(app.cp_targets,app.agent_num);

for ct = 1:app.agent_num
    estimator{app.INDEX_DFIR,ct} = DFIR(app.nh,app.nx,app.nz,app.nu,app.F,app.J_F,app.R,app.J_R,app.R,app.J_R,app.data.agent(ct).initial_state(:),3);
    estimator{app.INDEX_EKF ,ct} = EKF(diag([0.01, 0.1, 0.05]), diag([0.002, 0.002, 0.002]), diag([0.01, 0.015, 0.01, 0.015, 0.01, 0.005, 0.005]), app.F, app.J_F, app.R, app.J_R, app.data.agent(ct).initial_state(:));
end

%% make real
for ct = 1:app.iteration
    
    if ct < 30
        app.data.agent(1).input(:,ct) = normrnd([0.2 -0.2]', 0.1);
        app.data.agent(2).input(:,ct) = normrnd([0.1 0.15]', 0.1);
        app.data.agent(3).input(:,ct) = normrnd([0.1 0.2]', 0.1);
        app.data.agent(4).input(:,ct) = normrnd([0.1 0.16]', 0.1);
        app.data.agent(5).input(:,ct) = normrnd([0.1 -0.21]', 0.1);
    elseif ct < 60
        app.data.agent(1).input(:,ct) = normrnd([0.1 0.1]', 0.1);
        app.data.agent(2).input(:,ct) = normrnd([0.15 -0.3]', 0.1);
        app.data.agent(3).input(:,ct) = normrnd([0.2 -0.3]', 0.1);
        app.data.agent(4).input(:,ct) = normrnd([0.4 -0.36]', 0.1);
        app.data.agent(5).input(:,ct) = normrnd([0.3 0.13]', 0.1);
    else
        app.data.agent(1).input(:,ct) = normrnd([0.1 0.1]', 0.1);
        app.data.agent(2).input(:,ct) = normrnd([0.25 0.3]', 0.1);
        app.data.agent(3).input(:,ct) = normrnd([0.2 -0.35]', 0.1);
        app.data.agent(4).input(:,ct) = normrnd([0.1 -0.3]', 0.1);
        app.data.agent(5).input(:,ct) = normrnd([0.15 0.4]', 0.1);
    end
    for i = 1:app.agent_num
        x = app.data.agent(i).trajectory.real(1,ct);
        y = app.data.agent(i).trajectory.real(2,ct);
        theta = app.data.agent(i).trajectory.real(3,ct);
        ul = app.data.agent(i).input(1,ct);
        ua = app.data.agent(i).input(2,ct);
        app.data.agent(i).trajectory.real(:,ct+1) = app.F(x, y, theta, ul, ua);
    end
    for i = 1:app.agent_num
        
        
    end
end
for i = 1:app.agent_num
        ax2_plots{i}.XData = app.data.agent(i).trajectory.real(1,1:ct+1);
        ax2_plots{i}.YData = app.data.agent(i).trajectory.real(2,1:ct+1); 
end
%% estimate
for ct = 1:app.iteration
        x1 = app.data.agent(1).trajectory.real(1,ct+1);
        y1 = app.data.agent(1).trajectory.real(2,ct+1);
        x2 = app.data.agent(2).trajectory.real(1,ct+1);
        y2 = app.data.agent(2).trajectory.real(2,ct+1);
        x3 = app.data.agent(5).trajectory.real(1,ct+1);
        y3 = app.data.agent(5).trajectory.real(2,ct+1);
        
        pj = [x1 y1 x2 y2 x3 y3];
        d1 = norm(app.data.agent(3).trajectory.real(1:2,ct+1) - app.data.agent(1).trajectory.real(1:2,ct+1));
        d2 = norm(app.data.agent(3).trajectory.real(1:2,ct+1) - app.data.agent(2).trajectory.real(1:2,ct+1));
        d3 = norm(app.data.agent(3).trajectory.real(1:2,ct+1) - app.data.agent(5).trajectory.real(1:2,ct+1));
        
%         e1 = atan(y1-app.data.agent(3).trajectory.real(2,ct+1)/x1-app.data.agent(3).trajectory.real(1,ct+1))-app.data.agent(3).trajectory.real(3,ct+1);
%         e2 = atan(y2-app.data.agent(3).trajectory.real(2,ct+1)/x2-app.data.agent(3).trajectory.real(1,ct+1))-app.data.agent(3).trajectory.real(3,ct+1);
%         e3 = atan(y3-app.data.agent(3).trajectory.real(2,ct+1)/x3-app.data.agent(3).trajectory.real(1,ct+1))-app.data.agent(3).trajectory.real(3,ct+1);
        e1 = app.data.agent(3).trajectory.real(3,ct+1);
        e2 = app.data.agent(3).trajectory.real(3,ct+1);
        e3 = app.data.agent(3).trajectory.real(3,ct+1);
        
        
        measurement = [d1 d2 d3 e1 e2 e3 app.data.agent(3).trajectory.real(3,ct+1)] + normrnd([0,0,0,0,0,0,0], [0.001 0.001 0.001 0.001 0.001 0.001 0.0001]);
        estimator{app.INDEX_DFIR, 3}.estimate2(app.data.agent(3).input(:,ct), pj', measurement');
        measurement = [d1 d2 d3 e1 e2 e3 app.data.agent(3).trajectory.real(3,ct+1)] + normrnd([0,0,0,0,0,0,0], [0.1 0.01 0.01 0.01 0.1 0.01 0.001]);
        estimator{app.INDEX_EKF, 3}.estimate(app.data.agent(3).input(:,ct), pj', measurement');
        pj = [x1 y1 x2 y2 x3 y3];
        d1 = norm(app.data.agent(4).trajectory.real(1:2,ct+1) - app.data.agent(1).trajectory.real(1:2,ct+1));
        d2 = norm(app.data.agent(4).trajectory.real(1:2,ct+1) - app.data.agent(2).trajectory.real(1:2,ct+1));
        d3 = norm(app.data.agent(4).trajectory.real(1:2,ct+1) - app.data.agent(5).trajectory.real(1:2,ct+1));
        
        e1 = app.data.agent(4).trajectory.real(3,ct+1);
        e2 = app.data.agent(4).trajectory.real(3,ct+1);
        e3 = app.data.agent(4).trajectory.real(3,ct+1);
        measurement = [d1 d2 d3 e1 e2 e3 app.data.agent(4).trajectory.real(3,ct+1)] + normrnd([0,0,0,0,0,0,0], [0.001 0.001 0.001 0.001 0.001 0.001 0.0001]);
        estimator{app.INDEX_DFIR, 4}.estimate2(app.data.agent(4).input(:,ct), pj', measurement');
        measurement = [d1 d2 d3 e1 e2 e3 app.data.agent(4).trajectory.real(3,ct+1)]+ normrnd([0,0,0,0,0,0,0], [0.01 0.1 0.01 0.1 0.01 0.01 0.01]);
        estimator{app.INDEX_EKF, 4}.estimate(app.data.agent(4).input(:,ct), pj', measurement');
end


%% plot
figure(2);
x = zeros(1,app.iteration);
y = zeros(1,app.iteration);
x(:) = estimator{app.INDEX_DFIR, 3}.x_appended(1,:);
y(:) = estimator{app.INDEX_DFIR, 3}.x_appended(2,:);
plot(x,y,'-d', 'DisplayName', 'agent3(proposed)'); hold on;
x(:) = estimator{app.INDEX_DFIR, 4}.x_appended(1,:);
y(:) = estimator{app.INDEX_DFIR, 4}.x_appended(2,:);
plot(x,y,'-d', 'DisplayName', 'agent4(proposed)'); hold on;
x(:) = estimator{app.INDEX_EKF, 3}.x_appended(1,:);
y(:) = estimator{app.INDEX_EKF, 3}.x_appended(2,:);
plot(x,y,'-o', 'DisplayName', 'agent3(EKF)'); hold on;
x(:) = estimator{app.INDEX_EKF, 4}.x_appended(1,:);
y(:) = estimator{app.INDEX_EKF, 4}.x_appended(2,:);
plot(x,y,'-o', 'DisplayName', 'agent4(EKF)');
legend('FontSize', 12);

%%
figure(3);
clf;
subplot(2,1,1);
x = zeros(1, app.iteration);
y = zeros(1, app.iteration);
z = zeros(1, app.iteration);
interval = 1:app.iteration;

x(interval) = app.data.agent(3).trajectory.real(1,interval) - estimator{app.INDEX_DFIR, 3}.x_appended(1,interval);
y(interval) = app.data.agent(3).trajectory.real(2,interval) - estimator{app.INDEX_DFIR, 3}.x_appended(2,interval);
z(interval) = app.data.agent(3).trajectory.real(3,interval) - estimator{app.INDEX_DFIR, 3}.x_appended(3,interval);
estimator{app.INDEX_DFIR, 3}.x_se(1,interval) = x(interval).^2;
estimator{app.INDEX_DFIR, 3}.x_se(2,interval) = y(interval).^2;
estimator{app.INDEX_DFIR, 3}.x_se(3,interval) = z(interval).^2;

y = x.^2 + y.^2;
y = y/2;
plot(interval,y,'-o','DisplayName', 'proposed'); hold on;


x(interval) = app.data.agent(3).trajectory.real(1,interval) - estimator{app.INDEX_EKF, 3}.x_appended(1,interval);
y(interval) = app.data.agent(3).trajectory.real(2,interval) - estimator{app.INDEX_EKF, 3}.x_appended(2,interval);
z(interval) = app.data.agent(3).trajectory.real(3,interval) - estimator{app.INDEX_EKF, 3}.x_appended(3,interval);
estimator{app.INDEX_EKF, 3}.x_se(1,interval) = x(interval).^2;
estimator{app.INDEX_EKF, 3}.x_se(2,interval) = y(interval).^2;
estimator{app.INDEX_EKF, 3}.x_se(3,interval) = z(interval).^2;
y = x.^2 + y.^2;
plot(interval,y, '-x','DisplayName', 'EKF'); hold on;
legend('Fontsize', 12);
title('robot3 estimation error','Fontsize', 13);
ylabel('error(m^2)','Fontsize', 12);
xlabel('time(s)','Fontsize', 12);


subplot(2,1,2);
x(interval) = app.data.agent(4).trajectory.real(1,interval) - estimator{app.INDEX_DFIR, 4}.x_appended(1,interval);
y(interval) = app.data.agent(4).trajectory.real(2,interval) - estimator{app.INDEX_DFIR, 4}.x_appended(2,interval);
z(interval) = app.data.agent(4).trajectory.real(3,interval) - estimator{app.INDEX_DFIR, 4}.x_appended(3,interval);
estimator{app.INDEX_DFIR, 4}.x_se(1,interval) = x(interval).^2;
estimator{app.INDEX_DFIR, 4}.x_se(2,interval) = y(interval).^2;
estimator{app.INDEX_DFIR, 4}.x_se(3,interval) = z(interval).^2;
y = x.^2 + y.^2;
y = y/2;
plot(interval,y,'-o','DisplayName', 'proposed'); hold on;


x(interval) = app.data.agent(4).trajectory.real(1,interval) - estimator{app.INDEX_EKF, 4}.x_appended(1,interval);
y(interval) = app.data.agent(4).trajectory.real(2,interval) - estimator{app.INDEX_EKF, 4}.x_appended(2,interval);
z(interval) = app.data.agent(4).trajectory.real(3,interval) - estimator{app.INDEX_EKF, 4}.x_appended(3,interval);
estimator{app.INDEX_EKF, 4}.x_se(1,interval) = x(interval).^2;
estimator{app.INDEX_EKF, 4}.x_se(2,interval) = y(interval).^2;
estimator{app.INDEX_EKF, 4}.x_se(3,interval) = z(interval).^2;
y = x.^2 + y.^2;
plot(interval,y, '-x','DisplayName', 'EKF'); hold on;
legend('Fontsize', 12);
title('robot4 estimation error','Fontsize', 13);
ylabel('error(m^2)','Fontsize', 12);
xlabel('time(s)','Fontsize', 12);



%% RMSE
for ct = 3:4
   estimator{app.INDEX_DFIR, ct}.x_rmse = sum(estimator{app.INDEX_DFIR, ct}.x_se(1:2,interval))/2;
   estimator{app.INDEX_EKF, ct}.x_rmse = sum(estimator{app.INDEX_EKF, ct}.x_se(1:2,interval));
   estimator{app.INDEX_DFIR, ct}.x_rmse = sum(estimator{app.INDEX_DFIR, ct}.x_rmse);
   estimator{app.INDEX_EKF, ct}.x_rmse = sum(estimator{app.INDEX_EKF, ct}.x_rmse);
   
   fprintf("ROBOT %d DFIR RMSE %f \r\n",ct, estimator{app.INDEX_DFIR, ct}.x_rmse);
   fprintf("ROBOT %d EKF RMSE %f \r\n",ct, estimator{app.INDEX_EKF, ct}.x_rmse);
end

























































