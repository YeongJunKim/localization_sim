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


app_settings();pause(1);

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
   ROBOTS{i} = turtlebot3_default(app.agent_name(i), 0);
   ROBOTS{i}.run_time_info = 0;
end

%%
tm = cell(app.agent_num, 1);
% for i = 1:app.agent_num
% disp("timer i");
% disp(i);
% tm{i} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@app_ros_get_data_timer2, i});
% % start(tm{i});
% start([tm{1} tm{2} tm{3} tm{4} tm{5} tm{6}]); 
% end
%%
disp("why");
tm{1} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@timer1, 1});
tm{2} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@timer2, 2});
tm{3} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@timer3, 3});
tm{4} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@timer4, 4});
tm{5} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@timer5, 5});
tm{6} = timer('BusyMode', 'drop', 'ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', {@timer6, 6});
start([tm{1} tm{2} tm{3} tm{4} tm{5} tm{6}]); 

% start(tm);