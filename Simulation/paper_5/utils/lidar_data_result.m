function lidar_data_result(fig_on_off,weight)

global app
global LIDARS
global experiment_data

LIDARS = cell(1,app.agent_num);

%% init
for ag = 1:app.agent_num
    fprintf("ag = %d\n", ag);
    if(app.digraph.Nodes.Type{ag} == "known")
        %% set var
        % time interval_ 1.5 0.5 0.5
        % 
        %%
    else
        angle_interval_ = 1;
        time_interval_  = app.dt;
        find_neighbors = find(app.adj_full(:,ag)==1);
        robot_num_ = size(find_neighbors, 1);
        range_ = [0 3];
        LIDARS{ag} = lidaring(strcat("/",app.digraph.Nodes.Name{ag}), angle_interval_, time_interval_, robot_num_, range_);
        
        init = zeros(2,robot_num_,1);
        for j = 1:robot_num_
            init(:,j,1) = app.initial_state(1:2,find_neighbors(j))-app.initial_state(1:2,ag);
        end
        
        LIDARS{ag}.lidaring_position_init(init,app.iteration);
        LIDARS{ag}.option("figure", fig_on_off);
        LIDARS{ag}.option("weight", weight);
        LIDARS{ag}.option("NLOS_random_noise", 0);
    end
end
%% clustering
for ct = 1:app.iteration
    if ct > 93
    end
    fprintf("ct = %d\n", ct);
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
        else
            LIDARS{ag}.lidaring_run(experiment_data(ag).lidar(:,ct));
        end
    end
end
%% save to app.result.measurement
plot_colors = ["r", "g", "c", "b", "m", "k"];
plot_shape = ["-*","-h","-+", "-o", "-x", "-d"];
for ag = 1:app.agent_num
    if app.digraph.Nodes.Type{ag} ~= "known"
        figure(100 + ag);
        clf;
        for ct = 1:app.iteration
            for i = 1:LIDARS{ag}.robot_num
                offset = LIDARS{ag}.robot_num;
                de = -experiment_data(ag).ahrsv1(ct);
                pos = [cos(de) sin(de); -sin(de) cos(de)] * (LIDARS{ag}.result_data(:,i,ct));
                experiment_data(ag).measurement(i,ct) = pos(1);
                experiment_data(ag).measurement(offset + i,ct) = pos(2);
            end
            experiment_data(ag).measurement(offset * 2 + 1,ct) = experiment_data(ag).ahrsv1(ct);
        end
        for i = 1:LIDARS{ag}.robot_num
            offset = LIDARS{ag}.robot_num;
            x = experiment_data(ag).measurement(i,:);
            y = experiment_data(ag).measurement(i+offset,:);
            find_neighbors = find(app.adj_full(:,ag) == 1);
            str_target = num2str(find_neighbors(i));
            legend_ = strcat("neighbor: ", str_target);
            plot(x,y, plot_shape(find_neighbors(i)), 'DisplayName', legend_); hold on;
        end
    strnum = num2str(ag);
    title_ = strcat("robot: ", strnum);
    title_ = strcat(title_, " relative measurement");
    title(title_, 'FontSize', 13);
    xlabel('x(m)', 'FontSize', 13);
    ylabel('y(m)', 'FontSize', 13);
    xlim([-1.5 1.5]);
    ylim([-1.5 1.5]);
    axis equal;
    legend('FontSize', 13);
    end
end



% 
% for ag = 1:app.agent_num
%     if app.digraph.Nodes.Type{ag} == "known"
%     else
%         rotations = zeros(LIDARS{ag}.robot_num * 2, 1);
%         %     experiment_data(ag).measurement = zeros(LIDARS{ag}.robot_num * 2, app.iteration);
%         for ct = 1:app.iteration
%             offset = LIDARS{ag}.robot_num;
%             for i = 1:LIDARS{ag}.robot_num
%                 app.result.agent(ag).measurement(i,ct)                         = LIDARS{ag}.result_data_distance_angle(1,i,ct);
%                 app.result.agent(ag).measurement(LIDARS{ag}.robot_num + i,ct)  = LIDARS{ag}.result_data_distance_angle(2,i,ct);
%                 experiment_data(ag).measurement(i,ct)                          = LIDARS{ag}.result_data_distance_angle(1,i,ct);
%                 experiment_data(ag).measurement(LIDARS{ag}.robot_num + i,ct)   = LIDARS{ag}.result_data_distance_angle(2,i,ct);
%                 
%                 if ct == 1
%                     
%                 elseif experiment_data(ag).measurement(offset+i,ct) < -1.5 && experiment_data(ag).measurement(offset+i,ct-1) > 1.5
%                     fprintf("ag: %d, index: %d, rotation + 1\n", ag, offset+i);
%                     rotations(offset+i) = rotations(offset+i) + 1;
%                 elseif experiment_data(ag).measurement(offset+i,ct) > 1.5 && experiment_data(ag).measurement(offset+i,ct-1) < -1.5
%                     fprintf("ag: %d, index: %d, rotation - 1\n", ag, offset+i);
%                     rotations(offset+i) = rotations(offset+i) - 1;
%                 end
%                 experiment_data(ag).measurement_filtered(offset+i,ct) = experiment_data(ag).measurement(offset+i,ct) + rotations(offset+i) * deg2rad(360);
%             end
%         end
%         disp(rotations);
%         experiment_data(ag).measurement(offset+1:offset*2,:) = experiment_data(ag).measurement_filtered(offset+1:offset*2,:);
%     end
% end