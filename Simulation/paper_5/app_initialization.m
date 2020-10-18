function r = app_initialization()

global app
global estimator

app.initial_state = zeros(app.nx, app.agent_num);

app.initial_state(:,findnode(app.digraph, "tb3a")) = [0 0 0]';
app.initial_state(:,findnode(app.digraph, "tb3b")) = [1 3 0]';
app.initial_state(:,findnode(app.digraph, "tb3c")) = [2 0 0]';
app.initial_state(:,findnode(app.digraph, "tb3d")) = [3 2 0]';
app.initial_state(:,findnode(app.digraph, "tb3e")) = [4 0 0]';
app.initial_state(:,findnode(app.digraph, "tb3f")) = [5 3 0]';

app.anchor_position = zeros(2, app.anchor_num);
app.anchor_position(:,1) = [0 0]';
app.anchor_position(:,2) = [0 6]';
app.anchor_position(:,3) = [6 6]';
app.anchor_position(:,4) = [6 0]';

% estimator init
[app.F, app.jF] = dynamics_nonholonomic(app.dt);
estimator = cell(app.estimator_num, app.agent_num);

for i = 1:app.agent_num
    if(app.digraph.Nodes.Type{i} == "known")
        % for known robots 
        % only using FIR with PEFFME
        % horizon size
        nh_ = app.horizon_size.FIR;
        % state size
        nx_ = app.nx;
        % input size
        nu_ = app.nu;
        % measurement size
        nz_ = size(app.anchor_position, 2) + 1;
        % nonholonomic dynamics
        [f_, jf_] = dynamics_nonholonomic(app.dt);
        % measurement with 4 anchor
        [h_, jh_] = measurement_update(app.anchor_position);
        estimator{app.index_RDFIR, i}   = FIR(nh_, nx_, nz_, nu_, f_, jf_, h_, jh_, app.initial_state(:,i));
        estimator{app.index_EKF, i}     = FIR(nh_, nx_, nz_, nu_, f_, jf_, h_, jh_, app.initial_state(:,i));
    else
        % for unknwon robots
        % using relative measurement with neighbors (app.adj_full)
        % neighbors number of each agent are described in app.nh{} <- cell
        nh_ = app.horizon_size.RDFIR;
        % state size
        nx_ = app.nx;
        % input size
        nu_ = app.nu;
        % measurement size
        nz_ = size(app.nh{i},1)
        % neighbor number
        nn_ = size(find(app.adj_full(:,i)==1), 1)
        % nonholonomic dynamics
        [f_, jf_] = dynamics_nonholonomic(app.dt);
        % relative measurement function
        [h1_,h2_,h3_,jh1_,jh2_,jh3_] = h__();
        estimator{app.index_RDFIR, i} = RDFIR(nh_, nx_, nu_, nz_, f_,jf_,h1_, h2_,h3_,jh1_,jh2_,jh3_,app.initial_state(:,i),nn_);
    end
    
end

% result data init 
for ct = 1:app.agent_num
   app.result.agent(ct).input = zeros(app.nu, []);
   if(app.digraph.Nodes.Type{ct} == "known")
   app.result.agent(ct).measurement = zeros(size(app.anchor_position, 2) + 1, []);
   else
    app.result.agent(ct).measurement = zeros(size(app.nh{ct},1), []);
   end
   app.result.agent(ct).trajectory.real = zeros(app.nx, []);
   app.result.agent(ct).trajectory.estimated = zeros(app.nx, []);
   app.result.agent(ct).trajectory.se = zeros(app.nx, []);
   app.result.agnet(ct).trajectory.rmse = 0;
   app.result.agent(ct).trajectory.real(:,1) = app.initial_state(:,ct);
   app.result.agent(ct).trajectory.estimated(:,1) = app.initial_state(:,ct);
%    disp(app.result.agent(ct))
end

% plot initialization
agent_names = "agent";
figure('Name', 'real');
clf;
app.ax1 = axes;
app.ax1_plots = cell(app.agent_num, 1);

for ct = 1:app.agent_num
    app.ax1_plots{ct} = plot(app.ax1, app.initial_state(1,ct),app.initial_state(2,ct), '*'); hold on;
end
xlim([-10 10]);
ylim([-10 10]);
xlabel("x(m)", 'FontSize', 12);
ylabel("y(m)", 'FontSize', 12);
title("trajectory", 'FontSize', 13);
hold off;

figure('Name', 'estimate');
clf;
app.ax2 = axes;
app.ax2_plots = cell(app.agent_num, 1);

for ct = 1:app.agent_num
    agent_plot_name = strcat(agent_names, num2str(ct));
    if(app.digraph.Nodes.Type{ct} == "unknown")
    agent_plot_name = strcat(agent_plot_name, "(unknown)");
    else
    agent_plot_name = strcat(agent_plot_name, "(known)");
    end
    app.ax2_plots{ct} = plot(app.ax2, app.initial_state(1,ct),app.initial_state(2,ct), '*', 'DisplayName', agent_plot_name); hold on;
end
legend;
xlim([-10 10]);
ylim([-10 10]);
xlabel("x(m)", 'FontSize', 12);
ylabel("y(m)", 'FontSize', 12);
title("trajectory", 'FontSize', 13);
hold on;



r = "app_initialization ok";
end 