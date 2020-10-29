

global app
global LIDARS
% experiment_data


LIDARS = cell(1,app.agent_num);

%% init
for ag = 1:app.agent_num
    fprintf("ag = %d\n", ag);
    if(app.digraph.Nodes.Type{ag} == "known")
        
    else
        angle_interval_ = 1;
        time_interval_  = 1.5;
        find_neighbors = find(app.adj_full(:,ag)==1);
        robot_num_ = size(find_neighbors, 1);
        range_ = [0 2];
        LIDARS{ag} = lidaring(strcat("/",app.digraph.Nodes.Name{ag}), angle_interval_, time_interval_, robot_num_, range_);
        
        init = zeros(2,robot_num_,1);
        for j = 1:robot_num_
            init(:,j,1) = app.initial_state(1:2,find_neighbors(j))-app.initial_state(1:2,ag);
        end
        
        LIDARS{ag}.lidaring_position_init(init,app.iteration);
        LIDARS{ag}.option("figure", 0);
    end
end
%% clustering
for ct = 1:app.iteration
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
        else
            LIDARS{ag}.lidaring_run(experiment_data(ag).lidar(:,ct));
        end
    end
end
%% save to app.result.measurement
for ag = 1:app.agent_num
%     experiment_data(ag).measurement = zeros(LIDARS{ag}.robot_num * 2, app.iteration);
    for ct = 1:app.iteration
        if(app.digraph.Nodes.Type{ag} == "known")
            %     app.result.agent(ag).measurement = zeros(
        else
            for i = 1:LIDARS{ag}.robot_num
                app.result.agent(ag).measurement(i,ct)                         = LIDARS{ag}.result_data_distance_angle(1,i,ct);
                app.result.agent(ag).measurement(LIDARS{ag}.robot_num + i,ct)  = LIDARS{ag}.result_data_distance_angle(2,i,ct);
                experiment_data(ag).measurement(i,ct)                          = LIDARS{ag}.result_data_distance_angle(1,i,ct);
                experiment_data(ag).measurement(LIDARS{ag}.robot_num + i,ct)   = LIDARS{ag}.result_data_distance_angle(2,i,ct);
            end
        end
    end
end




