% paper: Distributed Kalman filter for cooperative localization with integrated measurements
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-07-28
% DKFCL class is in https://www.github.com/YeongJunKim/matlab/filters/DKFCL

% system eq
% x_{i,k+1} = A * x_{i,k} + B * w_{i,k}
% y_{i,k} = C * x_{i,k} + v_{i,k}
% z_{i,j,k} = D(x_{i,k} - x_{j,k})
% x = [x_position y_position x_vel y_vel]'
% u = [x_vel y_vel]'


%%
clear all
clc
clf
%% add path
addpath('./../../../matlab/filters/DKFCL');
addpath('./..');
%% global variables
global app

make_video = 1;

%% robot init
app.agent_num = 8;
app.adjacency = zeros(app.agent_num, app.agent_num);
app.adjacency(1,[7 8]) = [0.6 0.4];
app.adjacency(2,[1 8]) = [0.3 0.7];
app.adjacency(3,[1 2]) = [0.7 0.3];
app.adjacency(4,[2 3]) = [0.5 0.5];
app.adjacency(5,[3 4]) = [0.8 0.2];
app.adjacency(6,[4 5]) = [0.2 0.8];
app.adjacency(7,[5 6]) = [0.7 0.3];
app.adjacency(8,[6 7]) = [0.3 0.7];

%% simulation init
app.dt = 1;
app.iteration = 140;
for ct = 1:app.agent_num
    app.agent(ct).data.nx = 4;
    app.agent(ct).data.ny = 4;
    app.agent(ct).data.nu = 2;
    app.agent(ct).data.nz = 4;
    app.agent(ct).trajectory.real = zeros(app.agent(ct).data.nx, app.iteration);
    app.agent(ct).trajectory.filtered.al1 = zeros(app.agent(ct).data.nx, app.iteration);
    app.agent(ct).trajectory.filtered.al2 = zeros(app.agent(ct).data.nx, app.iteration);
    app.agent(ct).trajectory.measurement = zeros(app.agent(ct).data.ny, app.iteration);
    app.agent(ct).trajectory.time = zeros(1, app.iteration);
    app.agent(ct).trajectory.input = zeros(app.agent(ct).data.nu, app.iteration);
    app.agent(ct).trajectory.real(:,1) = normrnd([10 0 0 0]', [5 0 0 0]');
    app.agent(ct).trajectory.filtered.al1(:,1) = app.agent(ct).trajectory.real(:,1);
    app.agent(ct).trajectory.filtered.al2(:,1) = app.agent(ct).trajectory.real(:,1);
end
%% plotting init
figure(1);
clf;
ax = axes;
app.plot_agent = cell(1,app.agent_num);
app.plot_edge = cell(app.agent_num, app.agent_num);
for ct = 1:app.agent_num
    
    app.plot_agent{ct} = plot(ax, app.agent(ct).trajectory.real(1,1), app.agent(ct).trajectory.real(2,1), 'r*'); hold on; grid on;
end
for i = 1:app.agent_num-1
    for j = 1:app.agent_num
        if i == j
        else
            if(app.adjacency(i,j) == 1)
                x = zeros(1,2);
                y = zeros(1,2);
                x(1) = app.agent(i).trajectory.real(1,1); x(2) = app.agent(j).trajectory.real(1,1);
                y(1) = app.agent(i).trajectory.real(2,1); y(2) = app.agent(j).trajectory.real(2,1);
                app.plot_edge{i,j} = plot(ax, x,y, 'b-'); hold on;
            end
        end
    end
end
xlim([-25 25])
ylim([-25 25])
figure(2);
clf;
ax2 = axes;
app.plot_trajectory_real = cell(1,app.agent_num);
app.plot_trajectory_al1 = cell(1,app.agent_num);
app.plot_trajectory_al2 = cell(1,app.agent_num);
for ag = 1:app.agent_num
%     app.plot_trajectory_real{ag} = plot(ax2, 0, 0, '-d'); hold on; grid on;
    lege1 = ['real_ ' num2str(ag)];
    app.plot_trajectory_real{ag} = plot(ax2,0,0, '-d', 'DisplayName', lege1); hold on;
%     app.plot_trajectory_al1{ag} = plot(ax2, 0, 0, '-*'); hold on; grid on;
    lege2 = ['filtered_{al1} ' num2str(ag)];
     app.plot_trajectory_al1{ag} = plot(ax2,0,0, '-*', 'DisplayName', lege2); hold on;
%     app.plot_trajectory_al2{ag} = plot(ax2, 0, 0, '-o'); hold on; grid on;
    lege3 = ['filtered_{al2} ' num2str(ag)];
    app.plot_trajectory_al2{ag} = plot(ax2,0,0, '-*', 'DisplayName', lege3); hold on;
end
xlim([-25 25])
ylim([-25 25])
lg = legend;
lg.NumColumns = 8;
hold off;



figure(3);
clf;
ax3 = axes;
app.plot_absolute_error_al1 = cell(1,app.agent_num);
app.plot_absoulte_error_al2 = cell(1,app.agent_num);
app.plot_rmse_al1 = cell(1,1);
app.plot_rmse_al2 = cell(1,1);
for ag = 1:app.agent_num
    lege1 = ['al1 error' num2str(ag)];
    app.plot_absolute_error_al1{ag} = plot(ax3,0,0, '-d', 'DisplayName', lege1); hold on;
    lege2 = ['al2 error' num2str(ag)];
    app.plot_absolute_error_al2{ag} = plot(ax3,0,0, '-o', 'DisplayName', lege2); hold on;
end

app.plot_rmse_al1 = plot(ax3, 0,0, '-*', 'DisplayName', "algorithm1"); hold on; grid on;
app.plot_rmse_al2 = plot(ax3, 0,0, '-d', 'DisplayName', "algorithm2"); hold on; grid on;
hold off;
lg2 = legend;
lg2.NumColumns = 9;
lg2.Location = "Northwest";

%% filtering class init
% examples
% global filters
% for ct = 1:app.agent_num
%    filters.FIR(ct).filter = FIR();
%    filters.EKF(ct).filter = EKF();
%    filters.PF(ct).filter = PF();
% end

P = diag([0.1, 0.1, 0.1, 0.1]);
Q = diag([0.01, 0.01, 0.01, 0.01]);
V = diag([2,2]);
R = diag([4,4]);
% R = app.adjacency * 2;
A = [1 0 app.dt 0;
    0 1 0 app.dt ;
    0 0 1 0;
    0 0 0 1];
B = [app.dt^2/2 0 0 0;
    0 app.dt^2/2 0 0;
    0 0 app.dt 0;
    0 0 0 app.dt];
C = [1 0 0 0;
    0 1 0 0];
D = [1 0 0 0;
    0 1 0 0];

for ag = 1:app.agent_num
    if(ag == 1 || ag == 5)
        C_ = C;
        C_(2,2) = 0;
    else
        C_ = C;
    end
    
    DKFCL_FILTER(ag) = DKFCL(P,Q,V,R,A,B,C_,D,app.adjacency,app.agent(ag).trajectory.real(:,1),1,ag);
end
for ag = 1:app.agent_num
    if(ag == 1 || ag == 5)
        C_ = C;
        C_(2,2) = 0;
    else
        C_ = C;
    end
    
    DKFCL_FILTER(ag+app.agent_num) = DKFCL(P,Q,V,R,A,B,C_,D,app.adjacency,app.agent(ag).trajectory.real(:,1),2,ag);
end
%% run simulation
% make real data
angle = 0;
d = zeros(app.agent_num, 1);
for ct = 2:app.iteration
    
    for ag = 1:app.agent_num
        if ct > 30
            d(ag) = norm(app.agent(ag).trajectory.real(:,ct-1));
        else
            d(ag) = norm(app.agent(ag).trajectory.real(:,ct-1));
        end
    end
    for ag = 1:app.agent_num
        app.agent(ag).trajectory.input(1,ct) = d(ag) * cos(angle) - app.agent(ag).trajectory.real(1,ct-1);
        app.agent(ag).trajectory.input(2,ct) = d(ag) * sin(angle) - app.agent(ag).trajectory.real(2,ct-1);
        %  app.agent(ag).trajectory.input(:,ct) = normrnd([normrnd(0,3) normrnd(0,3)], 0.1)';
        
        app.agent(ag).trajectory.real(:,ct) = dynamics([app.agent(ag).trajectory.real(1:2,ct-1)' app.agent(ag).trajectory.input(:,ct)']',1);
    end
    %     update_plot(ct);
    %     pause(0.05);
    angle = angle + 0.05;
end

% run filtering
for ct = 2:app.iteration
    for ag = 1:app.agent_num
        %prior_x update
        
    end
    for ag = 1:app.agent_num
        u_ = app.agent(ag).trajectory.input(:,ct);
        y_ = app.agent(ag).trajectory.real(1:2,ct) + normrnd([0 0]', 1);
        z_ = zeros(app.agent(1).data.nx, app.agent_num);
        
        for j = 1:app.agent_num
            z_(:,j) = app.agent(ag).trajectory.real(:,ct) - app.agent(j).trajectory.real(:,ct) + normrnd([0 0 0 0]', 1);
        end
        x_neighbor_ = zeros(app.agent(1).data.nx, app.agent_num);
        p_neighbor_ = zeros(app.agent(1).data.nx, app.agent(1).data.nx, app.agent_num);
        
        for j = 1:app.agent_num
            x_neighbor_(:,j) = DKFCL_obtain_x_prior(DKFCL_FILTER(j));
            p_neighbor_(:,:,j) = DKFCL_obtain_y_prior(DKFCL_FILTER(j));
        end
        
        app.agent(ag).trajectory.filtered.al1(:,ct) = DKFCL_run(DKFCL_FILTER(ag), u_, y_, z_, x_neighbor_, p_neighbor_);
        
        
        
        
        for j = 1:app.agent_num
            x_neighbor_(:,j) = DKFCL_obtain_x_prior(DKFCL_FILTER(j+app.agent_num));
            p_neighbor_(:,:,j) = DKFCL_obtain_y_prior(DKFCL_FILTER(j+app.agent_num));
        end
        app.agent(ag).trajectory.filtered.al2(:,ct) = DKFCL_run(DKFCL_FILTER(ag+app.agent_num), u_, y_, z_, x_neighbor_, p_neighbor_);
    end
    update_plot(ct);
    drawnow;
    
    if make_video == 1
       F1(ct-1) = getframe(1); 
       F2(ct-1) = getframe(2);
       F3(ct-1) = getframe(3);
    end
    
    pause(0.1);
end
%% plot
% figure(2);
% clf;
% x = zeros(1, app.iteration);
% y = zeros(1, app.iteration);
% for ag = 1:app.agent_num
%     x(:) = app.agent(ag).trajectory.real(1,:); y(:) = app.agent(ag).trajectory.real(2,:);
%     lege1 = ['real_ ' num2str(ag)];
%     plot(x,y, '-d', 'DisplayName', lege1); hold on;
%     x(:) = app.agent(ag).trajectory.filtered.al1(1,:); y(:) = app.agent(ag).trajectory.filtered.al1(2,:);
%     lege2 = ['filtered_{al1} ' num2str(ag)];
%     plot(x,y, '-*', 'DisplayName', lege2); hold on;
%     x(:) = app.agent(ag).trajectory.filtered.al2(1,:); y(:) = app.agent(ag).trajectory.filtered.al2(2,:);
%     lege3 = ['filtered_{al2} ' num2str(ag)];
%     plot(x,y, '-*', 'DisplayName', lege3); hold on;
% end
% hold off;
% lg = legend;
% lg.NumColumns = 8;

% figure(3);
% subplot(2,1,1);
% diff_sum.al1 = zeros(2, app.iteration);
% diff_sum.al2 = zeros(2, app.iteration);
% for ag = 1:app.agent_num
%     diff = abs(app.agent(ag).trajectory.real(1:2,:) - app.agent(ag).trajectory.filtered.al1(1:2,:));
%     x(:) = diff(1,:);   y(:) = diff(2,:);
%     lege1 = ['al1 error' num2str(ag)];
%     plot(1:app.iteration, x(:)+y(:), '-d', 'DisplayName', lege1); hold on;
%     diff_sum.al1 = diff_sum.al1 + diff;
%     diff = abs(app.agent(ag).trajectory.real(1:2,:) - app.agent(ag).trajectory.filtered.al2(1:2,:));
%     x(:) = diff(1,:);   y(:) = diff(2,:);
%     lege2 = ['al2 error' num2str(ag)];
%     plot(1:app.iteration, x(:)+y(:), '-o', 'DisplayName', lege2); hold on;
%     diff_sum.al2 = diff_sum.al2 + diff;
% end
% hold off;
% legend;
% subplot(2,1,2);
% x(:) = diff_sum.al1(1,:);
% y(:) = diff_sum.al1(2,:);
% rmse = sqrt(x(:).^2 + y(:).^2);
% plot(1:app.iteration, rmse(:), '-*', 'DisplayName', "algorithm1"); hold on;
% 
% x(:) = diff_sum.al2(1,:);
% y(:) = diff_sum.al2(2,:);
% rmse = sqrt(x(:).^2 + y(:).^2);
% plot(1:app.iteration, rmse(:), '-d', 'DisplayName', "algorithm2");
% hold off;
% legend;

%% video

if make_video == 1
    video_name = sprintf('robot_%s_%s',datestr(now,'yymmdd'),datestr(now,'HHMMSS'));
    video = VideoWriter(video_name,'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.05;   % 영상의 FPS, 값이 클수록 영상이 빨라짐
    open(video);
    writeVideo(video,F1);
    close(video);
    video_name = sprintf('robot_%s_%s',datestr(now,'yymmdd'),datestr(now,'HHMMSS'));
    video = VideoWriter(video_name,'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.05;   % 영상의 FPS, 값이 클수록 영상이 빨라짐
    open(video);
    writeVideo(video,F2);
    close(video);
    video_name = sprintf('robot_%s_%s',datestr(now,'yymmdd'),datestr(now,'HHMMSS'));
    video = VideoWriter(video_name,'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.05;   % 영상의 FPS, 값이 클수록 영상이 빨라짐
    open(video);
    writeVideo(video,F3);
    close(video);
end





%% local functions
function update_plot(index)
global app
for ag = 1:app.agent_num
    app.plot_agent{ag}.XData = app.agent(ag).trajectory.real(1,index);
    app.plot_agent{ag}.YData = app.agent(ag).trajectory.real(2,index);
end
for i = 1:app.agent_num-1
    for j = 1:app.agent_num
        if i == j
        else
            if(app.adjacency(i,j) == 1)
                x = zeros(1,2);
                y = zeros(1,2);
                x(1) = app.agent(i).trajectory.real(1,index); x(2) = app.agent(j).trajectory.real(1,index);
                y(1) = app.agent(i).trajectory.real(2,index); y(2) = app.agent(j).trajectory.real(2,index);
                app.plot_edge{i,j}.XData = x(:);
                app.plot_edge{i,j}.YData = y(:);
            end
        end
    end
end

for ag = 1:app.agent_num
   app.plot_trajectory_real{ag}.XData = app.agent(ag).trajectory.real(1,1:index);
   app.plot_trajectory_real{ag}.YData = app.agent(ag).trajectory.real(2,1:index);
   app.plot_trajectory_al1{ag}.XData = app.agent(ag).trajectory.filtered.al1(1,1:index);
   app.plot_trajectory_al1{ag}.YData = app.agent(ag).trajectory.filtered.al1(2,1:index);
   app.plot_trajectory_al2{ag}.XData = app.agent(ag).trajectory.filtered.al2(1,1:index);
   app.plot_trajectory_al2{ag}.YData = app.agent(ag).trajectory.filtered.al2(2,1:index);
end

interval = 1:index;
diff_sum.al1 = zeros(2, index);
diff_sum.al2 = zeros(2, index);
for ag = 1:app.agent_num
    diff = abs(app.agent(ag).trajectory.real(1:2,interval) - app.agent(ag).trajectory.filtered.al1(1:2,interval));
    diff_sum.al1 = diff_sum.al1 + diff;
    x(interval) = diff(1,interval);   y(interval) = diff(2,interval);
    app.plot_absolute_error_al1{ag}.YData = x(interval) + y(interval);
    app.plot_absolute_error_al1{ag}.XData = interval;
    diff = abs(app.agent(ag).trajectory.real(1:2,interval) - app.agent(ag).trajectory.filtered.al2(1:2,interval));
    diff_sum.al2 = diff_sum.al2 + diff;
    x(interval) = diff(1,interval);   y(interval) = diff(2,interval);
    app.plot_absolute_error_al2{ag}.YData = x(interval) + y(interval);
    app.plot_absolute_error_al2{ag}.XData = interval;
end

x(interval) = diff_sum.al1(1,interval);
y(interval) = diff_sum.al1(2,interval);
rmse = sqrt(x(interval).^2 + y(interval).^2);
app.plot_rmse_al1.XData = interval;
app.plot_rmse_al1.YData = rmse(interval);
x(interval) = diff_sum.al2(1,interval);
y(interval) = diff_sum.al2(2,interval);
rmse = sqrt(x(interval).^2 + y(interval).^2);
app.plot_rmse_al2.XData = interval;
app.plot_rmse_al2.YData = rmse(interval);
end