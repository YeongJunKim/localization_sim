
delete_timer();pause(1);
ros_deinit();pause(3);

clear all
close all

rosinit('192.168.0.5');

addpath('./../../../matlab_utils');
addpath('./../../../turtlebot3_matlab/turtlebot3');

global ROBOTS
global app


app_settings();pause(1);

app.agent_name = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
app.agent_num = 6;



app.experiment_data = [];

%%
ROBOTS = cell(3,1);
for i = 1:app.agent_num
   ROBOTS{i} = turtlebot3(app.agent_name(i));
end


% turtlebot3_names = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
% 
% ROBOT = turtlebot3;
% turtlebot3_init(ROBOT, "tb3d");


tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 0.5, 'TimerFcn', {@app_gazebo_get_data_timer});
start(tm);