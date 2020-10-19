clear all
close all

addpath('./../../../matlab_utils');
addpath('./../../../turtlebot3_matlab/turtlebot3');

global ROBOTS
global app


app_settings();pause(1);

app.agent_name = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
app.agent_num = 6;

delete_timer();pause(1);
ros_deinit();pause(3);
rosinit('192.168.0.5');


app.experiment_data = [];

%%
ROBOTS = cell(3,1);
for i = 1:app.agent_num
   ROBOTS{i} = turtlebot3_default(app.agent_name(i));
   ROBOTS{i}.run_time_info = 0;
end


% turtlebot3_names = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
% 
% ROBOT = turtlebot3;
% turtlebot3_init(ROBOT, "tb3d");


tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 0.5, 'TimerFcn', {@app_ros_get_data_timer});
start(tm);