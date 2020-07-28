% paper: Distributed Kalman filter for cooperative localization with integrated measurements
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr
% date: 2020-07-28

% system eq
% x_{i,k+1} = A * x_{i,k} + B * w_{i,k}
% y_{i,k} = C * x_{i,k} + v_{i,k}
% z_{i,j,k} = D(x_{i,k} - x_{j,k})

%%
clear all
clc
clf
%% add path
addpath('/../matlab/filters');
%% global variables
global app
%% robot init
app.agent_num = 8;
app.adjacency = zeros(app.agent_num, app.agent_num);
app.adjacency(1,[7 8]) = [0.1 0.9];
app.adjacency(2,[1 8]) = [0.3 0.7];
app.adjacency(3,[1 2]) = [0.7 0.3];
app.adjacency(4,[2 3]) = [0.5 0.5];
app.adjacency(5,[3 4]) = [0.8 0.2];
app.adjacency(6,[4 5]) = [0.2 0.8];
app.adjacency(7,[5 6]) = [0.7 0.3];
app.adjacency(8,[6 7]) = [0.3 0.3];

%% simulation init
app.dt = 1;
app.iteration = 30;
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
    app.agent(ct).trajectory.real(:,1) = normrnd([0 0 0 0]', [5 5 0 0]');
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
xlim([-20 20])
ylim([-20 20])
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
V = diag([4,4]);
R = diag([4,4]);
% R = app.adjacency * 2;
A = [1 0 app.dt 0;
    0 1 0 app.dt ;
    0 0 1 0;
    0 0 0 1];
B = [app.dt^2/2 0 0 0;
    0 app.dt 0 0;
    0 0 app.dt^2/2 0;
    0 0 0 app.dt];
C = [1 0 0 0;
    0 1 0 0];
D = [1 0 0 0;
    0 1 0 0];

for ag = 1:app.agent_num
    if(ag == 1 || ag == 5)
        C(2,2) = 0;
    end
    DKFCL_FILTER(ag) = DKFCL(P,Q,V,R,A,B,C,D,app.adjacency,app.agent(ag).trajectory.real(:,1),1,ag);
end
for ag = 1:app.agent_num
    if(ag == 1 || ag == 5)
        C(2,2) = 0;
    end
    DKFCL_FILTER(ag+app.agent_num) = DKFCL(P,Q,V,R,A,B,C,D,app.adjacency,app.agent(ag).trajectory.real(:,1),2,ag);
end
%% run simulation
% make real data
for ct = 2:app.iteration
    for ag = 1:app.agent_num
        app.agent(ag).trajectory.input(:,ct) = normrnd([1 1], 0.01)';
        app.agent(ag).trajectory.real(:,ct) = dynamics([app.agent(ag).trajectory.real(1:2,ct-1)' app.agent(ag).trajectory.input(:,ct)']',1);
    end
    %     update_plot(ct);
    %     pause(0.05);
end

% run filtering
for ct = 2:app.iteration
    for ag = 1:app.agent_num
        %prior_x update
        
    end
    for ag = 1:app.agent_num
        u_ = app.agent(ag).trajectory.input(:,ct);
        y_ = app.agent(ag).trajectory.real(1:2,ct) + normrnd([0 0]', 4);
        z_ = zeros(app.agent(1).data.nx, app.agent_num);
        for j = 1:app.agent_num
            z_(:,j) = app.agent(ag).trajectory.real(:,ct) - app.agent(j).trajectory.real(:,ct) + normrnd([0 0 0 0]', 4);
        end
        x_neighbor_ = zeros(app.agent(1).data.nx, app.agent_num);
        for j = 1:app.agent_num
            x_neighbor_(:,j) = A *DKFCL_FILTER(j).x_pre;
%             x_neighbor_(:,j) = app.agent(j).trajectory.real(:,ct);
        end
        app.agent(ag).trajectory.filtered.al1(:,ct) = DKFCL_run(DKFCL_FILTER(ag), u_, y_, z_, x_neighbor_);
        app.agent(ag).trajectory.filtered.al2(:,ct) = DKFCL_run(DKFCL_FILTER(ag+app.agent_num), u_, y_, z_, x_neighbor_);
    end
    update_plot(ct);
    
end
%% plot
figure(2);
clf;
x = zeros(1, app.iteration);
y = zeros(1, app.iteration);

for ag = 1:app.agent_num
   x(:) = app.agent(ag).trajectory.real(1,:); y(:) = app.agent(ag).trajectory.real(2,:);
   lege1 = ['real_ ' num2str(ag)]
   plot(x,y, '-d', 'DisplayName', lege1); hold on;
   x(:) = app.agent(ag).trajectory.filtered.al1(1,:); y(:) = app.agent(ag).trajectory.filtered.al1(2,:);
   lege2 = ['filtered_{al1} ' num2str(ag)]
   plot(x,y, '-*', 'DisplayName', lege2); hold on;
   x(:) = app.agent(ag).trajectory.filtered.al2(1,:); y(:) = app.agent(ag).trajectory.filtered.al2(2,:);
   lege3 = ['filtered_{al2} ' num2str(ag)]
   plot(x,y, '-*', 'DisplayName', lege3); hold on;
end
hold off;
legend;

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
end