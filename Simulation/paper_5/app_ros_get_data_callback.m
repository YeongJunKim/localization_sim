clear all
close all

addpath('./../../../matlab_utils');
addpath('./../../../turtlebot3_matlab/turtlebot3');

global ROBOTS
global app

global data

global data1
global data2
global data3
global data4
global data5
global data6

data = [];
data1 = [];
data2 = [];
data3 = [];
data4 = [];
data5 = [];
data6 = [];


exp2_settings();pause(1);

app.agent_name = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
app.agent_num = 6;
app.start_flag = 0;
delete_timer();pause(1);
ros_deinit();pause(3);
rosinit('192.168.0.4');


app.experiment_data = [];

%%
ROBOTS = cell(app.agent_num,1);
for i = 1:app.agent_num
   isknown = 0;
   if strcmp(app.digraph.Nodes.Type{i},"known")
      isknown = 1; 
   end
   ROBOTS{i} = turtlebot3_default(app.agent_name(i), 1, isknown);
   ROBOTS{i}.run_time_info = 0;
end

%%
tm = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 0.5, 'TimerFcn', @app_ros_get_data_timer);
start(tm);