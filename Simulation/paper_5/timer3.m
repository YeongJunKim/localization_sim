function app_ros_get_data_timer(obj, event, i)
global app ROBOTS
persistent first_run
persistent step
if isempty(first_run)
    step = 1;
    first_run = 1;
    app.stop_flag = 0;
        %if there is no moving
    ROBOTS{i}.sub_data(2).data = [0 0]';

    % attribute : name
    app.experiment_data(i).name = app.digraph.Nodes.Name{i};
    % input = measured input user_input = control input (keyboard)
    app.experiment_data(i).input = zeros(app.nu, []);
    app.experiment_data(i).user_input = zeros(app.nu, []);
    % lidar data
    app.experiment_data(i).lidar = zeros(360, []);
    % imu data from ahrsv1
    app.experiment_data(i).ahrsv1 = zeros(1, []);
    % tocs
    app.experiment_data(i).tocs = zeros(1, []);

    if(app.digraph.Nodes.Type{i} == "known")
        topic_name = strcat(app.digraph.Nodes.Name{i}, "/read");
        turtlebot3_addSubscriver(ROBOTS{i}, topic_name, 0);
        app.experiment_data(i).measurement = zeros(4, []);
    else
          nn_ = size(find(app.adj_full(:,i)==1), 1);
        app.experiment_data(i).measurement = zeros(2*nn_+1,[]);
    end
    topic_name = strcat(app.digraph.Nodes.Name{i}, "/cmd_vel");
    turtlebot3_addSubscriver(ROBOTS{i}, topic_name, 0);

    % wait for comming topics
    tic;
end

if app.start_flag == 1
fprintf("[%d] step = %d\n",i ,step);
try
%%  IMPORTANT  %%
%   GET  DATA   %
ROBOTS{i}.sub_imu_data    = receive(ROBOTS{i}.sub_imu, 1);
ROBOTS{i}.sub_odom_data   = receive(ROBOTS{i}.sub_odom, 1);
ROBOTS{i}.sub_ahrsv1_data = receive(ROBOTS{i}.sub_ahrsv1, 1);
% ROBOTS{i}.sub_input_data  = receive(ROBOTS{i}.sub_input, 1);

if(app.digraph.Nodes.Type{i} == "known")
   sub1 = rossubscriber(strcat(app.digraph.Nodes.Name{i}, "/read"));
   rev = receive(sub1,1);
   app.experiment_data(i).measurement(:,step) = measurement_process(rev);
else
   ROBOTS{i}.sub_scan_data = receive(ROBOTS{i}.sub_scan, 1);
   app.experiment_data(i).lidar(:,step) = ROBOTS{i}.sub_scan_data.Ranges(:);
end
app.experiment_data(i).imu_data(:,step)  = [ROBOTS{i}.sub_imu_data.Orientation.X, ...
                                            ROBOTS{i}.sub_imu_data.Orientation.Y, ...
                                            ROBOTS{i}.sub_imu_data.Orientation.Z, ...
                                            ROBOTS{i}.sub_imu_data.Orientation.W]';
app.experiment_data(i).odom_data(:,step) = [ROBOTS{i}.sub_odom_data.Twist.Twist.Linear.X, ...
                                            ROBOTS{i}.sub_odom_data.Twist.Twist.Angular.Z]';


app.experiment_data(i).ahrsv1(1,step) = ROBOTS{i}.sub_ahrsv1_data.Orientation.Z;
app.experiment_data(i).user_input(:,step) = ROBOTS{i}.cmd_vel(:);
app.experiment_data(i).tocs(1,step) = toc;
catch ME
    disp('[3] Error Message:');
    disp(ME.message);
end
%%  IMPORTANT  %%
%   SAVE DATA   %
% for i = 1:app.agent_num
%     
%     app.experiment_data(i).input(:,step) = [ROBOTS{i}.v_l ROBOTS{i}.v_a]';
%     app.experiment_data(i).user_input(:,step) = ROBOTS{i}.sub_data(2).data(:);
%     app.experiment_data(i).ahrsv1(:,step) = ROBOTS{i}.ahrsv1(:);
%     if(app.digraph.Nodes.Type{i} == "known")
%         app.experiment_data(i).measurement(:, step) = ROBOTS{i}.sub_data(1).data(:);
%         app.experiment_data(i).tocs(1,step)  = ROBOTS{i}.sub_data(1).data(end);
%     else
%         app.experiment_data(i).lidar(:,step) = ROBOTS{i}.lidar_data(:);
%         app.experiment_data(i).tocs(1,step)  = ROBOTS{i}.lidar_data(end);
%     end
% end
if(1 == app.stop_flag)
    app.tocs = [app.experiment_data(1).tocs(:)'; app.experiment_data(2).tocs(:)'; app.experiment_data(3).tocs(:)'; app.experiment_data(4).tocs(:)'; app.experiment_data(5).tocs(:)'; app.experiment_data(6).tocs(:)' ];
    rosshutdown();
    delete_timer();
    dbquit();
 end
step = step + 1;


end
end


function r = measurement_process(msg)
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
r = sub_measurement_now;
end
