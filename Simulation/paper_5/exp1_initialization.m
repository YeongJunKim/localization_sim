function r = app_initialization()
persistent unknown_agent_init_flag known_agent_init_flag

global app
global estimator

app.initial_state = zeros(app.nx, app.agent_num);

app.initial_state(:,findnode(app.digraph, "tb3a")) = [0.6 1.8 0]';
app.initial_state(:,findnode(app.digraph, "tb3b")) = [1.8 1.2 0]';
app.initial_state(:,findnode(app.digraph, "tb3c")) = [1.2 1.8 0]';
app.initial_state(:,findnode(app.digraph, "tb3d")) = [1.8 1.8 0]';
app.initial_state(:,findnode(app.digraph, "tb3e")) = [0.6 1.35 0]';
app.initial_state(:,findnode(app.digraph, "tb3f")) = [1.2 1.23 0]';


app.anchor_position = zeros(2, app.anchor_num);
app.anchor_position(:,1) = [0 0]';
app.anchor_position(:,2) = [0 6]';
app.anchor_position(:,3) = [6 6]';
app.anchor_position(:,4) = [6 0]';

% result data init 
for ct = 1:app.agent_num
   app.result.agent(ct).input = zeros(app.nu, []);
   app.result.agent(ct).odom_input = zeros(app.nu, []);
   app.result.agent(ct).user_input = zeros(app.nu, []);
   if(app.digraph.Nodes.Type{ct} == "known")
   app.result.agent(ct).measurement = zeros(size(app.anchor_position, 2) + 1, []);
   else
    app.result.agent(ct).measurement = zeros(size(app.nh{ct},1), []);
   end
   app.result.agent(ct).ahrsv1 = zeros(1,[]);
   app.result.agent(ct).trajectory.real = zeros(app.nx, []);
   app.result.agent(ct).trajectory.only_input = zeros(app.nx, []);
   app.result.agent(ct).trajectory.estimated = zeros(app.nx, []);
   app.result.agent(ct).trajectory.se = zeros(app.nx, []);
   app.result.agent(ct).trajectory.rmse = 0;
   app.result.agent(ct).trajectory.real(:,1) = app.initial_state(:,ct);
   app.result.agent(ct).trajectory.only_odom_input(:,1) = app.initial_state(:,ct);
   app.result.agent(ct).trajectory.only_user_input(:,1) = app.initial_state(:,ct);
   app.result.agent(ct).trajectory.only_constant_input(:,1) = app.initial_state(:,ct);
   app.result.agent(ct).trajectory.estimated(:,1) = app.initial_state(:,ct);
%    disp(app.result.agent(ct))
end



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
        if isempty(known_agent_init_flag)
            % nonholonomic dynamics
            [f_, jf_] = dynamics_nonholonomic(app.dt);
            % measurement with 4 anchor
            [h_, jh_] = measurement_update(app.anchor_position);
            known_agent_init_flag = 1;
        end
        estimator{app.index_RDFIR, i}     = FIR(nh_, nx_, nz_, nu_, f_, jf_, h_, jh_, app.initial_state(:,i));
        estimator{app.index_RDEKF, i}     = FIR(nh_, nx_, nz_, nu_, f_, jf_, h_, jh_, app.initial_state(:,i));
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
        nz_ = size(app.nh{i},1);
        % neighbor number
        nn_ = size(find(app.adj_full(:,i)==1), 1);
        if isempty(unknown_agent_init_flag)
            % nonholonomic dynamics
            [f_, jf_] = dynamics_nonholonomic(app.dt);
            % relative measurement function
            if(app.relative_scenario == app.relative_scenario_distance_eta)
                [h1_,h2_,h3_,jh1_,jh2_,jh3_] = h__();
            elseif(app.relative_scenario == app.relative_scenario_poss_diff)
                [h1_,h2_,h3_,jh1_,jh2_,jh3_] = h___(); 
            end
            unknown_agent_init_flag = 1;
        end
        if app.initial_error_scenario == app.initial_error_scenario_error
%             if i == 5 || i == 8 || i == 12 || i == 19 || i == 24
                app.initial_state(:,i) = app.initial_state(:,i) + normrnd([0 0 0]', [2 3 0.01]');
%             end
        elseif app.initial_error_scenario == app.initial_error_scenario_normal
            
        end
        estimator{app.index_RDFIR, i} = RDFIR(nh_, nx_, nu_, nz_, f_,jf_,h1_, h2_,h3_,jh1_,jh2_,jh3_,app.initial_state(:,i),nn_);
        %function obj = RDEKF(P_, Q_, R_, function_f_, function_jf_, function_h1_, function_h2_, function_h3_, function_jh1_, function_jh2_, function_jh3_, init_, nn_)
        R =  zeros(nn_, nn_);
        for j = 1:nn_*2
            R(j,j) = 0.01;
        end
        R(nn_*2+1,nn_*2+1) = 0.1;
        if(app.relative_scenario == app.relative_scenario_distance_eta)
            estimator{app.index_RDEKF, i} = RDEKF(diag([0.01, 0.01, 0.01]), diag([0.01, 0.01, 0.01]), R, f_, jf_, h1_, h2_, h3_, jh1_, jh2_, jh3_, app.initial_state(:,i),nn_);
        elseif(app.relative_scenario == app.relative_scenario_poss_diff)
            R =  zeros(nn_, nn_);
            for j = 1:nn_*2
                R(j,j) = 0.1;
            end
            R(nn_*2+1,nn_*2+1) = 0.1;
            estimator{app.index_RDEKF, i} = RDEKF(diag([0.1, 0.1, 0.1]), diag([0.1, 0.1, 0.1]), R, f_, jf_, h1_, h2_, h3_, jh1_, jh2_, jh3_, app.initial_state(:,i),nn_);
        end
    end
    
end


% plot initialization
% agent_names = "agent";
% figure(2);
% clf;
% app.ax1 = axes;
% app.ax1_plots = cell(app.agent_num, 1);
% 
% for ct = 1:app.agent_num
%     agent_plot_name = app.digraph.Nodes.Name{ct};
%     app.ax1_plots{ct} = plot(app.ax1, app.initial_state(1,ct),app.initial_state(2,ct), '*', 'DisplayName', agent_plot_name); hold on;
% end
% legend('FontSize', 15, 'NumColumns', 3);
% xlim([0 6]);
% ylim([0 6]);
% xlabel("x(m)", 'FontSize', 12);
% ylabel("y(m)", 'FontSize', 12);
% title("trajectory", 'FontSize', 13);
% hold off;
% grid on;

% figure(3);
% clf;
% app.ax2 = axes;
% app.ax2_plots = cell(app.agent_num, 1);
% app.ax2_plot_shape = ['*', '*', 'd', 'd', 'd', 'd', 'd','d','d','d'];
% app.ax2_plot_shape2 = ["-*", "-*", "-d", "-+", "-*", "-x", "-s","-d","-p","-h"];
% for ct = 1:app.agent_num
%     agent_plot_name = app.digraph.Nodes.Name{ct};
%     if app.digraph.Nodes.Type{ct} == "known"
%     else
%         app.ax2_plots{ct} = plot(app.ax2, app.initial_state(1,ct),app.initial_state(2,ct), '*', 'DisplayName', agent_plot_name); hold on;
%     end
% end
% legend('FontSize', 15, 'NumColumns', 3);
% % xlim([-8 10]);
% % ylim([-8 10]);
% xlabel("x(m)", 'FontSize', 12);
% ylabel("y(m)", 'FontSize', 12);
% title("trajectory", 'FontSize', 13);
% hold on;



r = "app_initialization ok";
end 