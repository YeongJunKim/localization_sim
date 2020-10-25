function app_ros_get_data_timer(obj, event)
global app ROBOTS
persistent first_run
persistent step
if isempty(first_run)
    step = 1;
    first_run = 1;
    app.stop_flag = 0;
    for i = 1:app.agent_num
        %if there is no moving
        ROBOTS{i}.sub_data(2).data = [0 0]';
        
        % attribute : name
        app.experiment_data(i).name = app.digraph.Nodes.Name{i};
        % input = measured input user_input = control input (keyboard)
        app.experiment_data(i).input = zeros(app.nu, []);
        app.experiment_data(i).user_input = zeros(app.nu, []);
        % lidar data
        app.experiment_data(i).lidar = zeros(360+1, []);
        % imu data from ahrsv1
        app.experiment_data(i).ahrsv1 = zeros(1, []);
        % tocs
        app.experiment_data(i).tocs = zeros(1, []);
        
        if(app.digraph.Nodes.Type{i} == "known")
            topic_name = strcat(app.digraph.Nodes.Name{i}, "/read");
            turtlebot3_addSubscriver(ROBOTS{i}, topic_name, @measurement);
            app.experiment_data(i).measurement = zeros(4+1, []);
        else
              nn_ = size(find(app.adj_full(:,i)==1), 1);
            app.experiment_data(i).measurement = zeros(2*nn_+1,[]);
        end
        topic_name = strcat(app.digraph.Nodes.Name{i}, "/cmd_vel");
        turtlebot3_addSubscriver(ROBOTS{i}, topic_name, @cmd_vel);
    end
    % wait for comming topics
    pause(3);
    tic;
    pause(1);
end

fprintf("step = %d\n", step);
% fprintf("distance measurement1 = [%d, %d, %d, %d] \r\n", ...
%     ROBOTS{1}.sub_data(1).data(1), ROBOTS{1}.sub_data(1).data(2), ...
%     ROBOTS{1}.sub_data(1).data(3), ROBOTS{1}.sub_data(1).data(4));
% fprintf("distance measurement2 = [%d, %d, %d, %d] \r\n", ...
%     ROBOTS{2}.sub_data(1).data(1), ROBOTS{2}.sub_data(1).data(2), ...
%     ROBOTS{2}.sub_data(1).data(3), ROBOTS{2}.sub_data(1).data(4));

%% IMPORTANT %%
%   SAVE DATA   %

for i = 1:app.agent_num
%     a = toc;
    app.experiment_data(i).input(:,step) = [ROBOTS{i}.v_l ROBOTS{i}.v_a]';
    app.experiment_data(i).user_input(:,step) = ROBOTS{i}.sub_data(2).data(:);
    app.experiment_data(i).ahrsv1(:,step) = ROBOTS{i}.ahrsv1(:);
    if(app.digraph.Nodes.Type{i} == "known")
        app.experiment_data(i).measurement(:, step) = ROBOTS{i}.sub_data(1).data(:);
        % toc
        app.experiment_data(i).tocs(1,step)  = ROBOTS{i}.sub_data(1).data(end);
    else
%         disp(i);
%         utilize with lidar scan data
        app.experiment_data(i).lidar(:,step) = ROBOTS{i}.lidar_data(:);
        app.experiment_data(i).tocs(1,step)  = ROBOTS{i}.lidar_data(end);
    end
end

if(1 == app.stop_flag)
    app.tocs = [app.experiment_data(1).tocs(:)'; app.experiment_data(2).tocs(:)'; app.experiment_data(3).tocs(:)'; app.experiment_data(4).tocs(:)'; app.experiment_data(5).tocs(:)'; app.experiment_data(6).tocs(:)' ];
    rosshutdown();
    delete_timer();
    dbquit();
 end
step = step + 1;
end


function cmd_vel(src, msg, obj)
sub_cmd_vel = msg;
v_l = msg.Linear.X;
v_a = msg.Angular.Z;
temp = [v_l v_a]';
obj.sub_data(2).data = temp;
end

function measurement(src, msg, obj)
sub_measurement_data = msg.Data;
sub_measurement_raw = extractBetween(sub_measurement_data,"a 1 "," b");
if size(sub_measurement_raw, 1) == 0
    sub_measurement_raw = cell(1,1);
    sub_measurement_raw(1) = {'0 0 0 0'};
end
sub_measurement_now = cell2mat(sub_measurement_raw(1));
if ischar(sub_measurement_now)
    sub_measurement_now = str2num(sub_measurement_now);
    sub_measurement_past = sub_measurement_now;
else
    sub_measurement_now = sub_measurement_past;
end
if size(sub_measurement_now) ~= 4
    sub_measurement_now = [0 0 0 0];
end
sub_measurement_now = (0.001)*sub_measurement_now;
%     disp(sub_measurement_now);
obj.sub_data(1).data = sub_measurement_now;
obj.sub_data(1).data(end + 1) = toc;
end