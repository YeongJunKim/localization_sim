% paper: Distributed Localization Using Finite Impulse Response Filter.
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-09-08
% EKFDCL class is in
% https://www.github.com/YeongJunKim/matlab/filters/

clear all;

addpath('./../../matlab/filters/');
addpath('./../../matlab/filters/DFIR');

global app;

make_video = 1;
absolute_on = 1;

%% invironment setup
app.agent_num = 3;
app.nx = 3;
app.nu = 2;
app.nh = 6;
app.nz = app.agent_num * 2;



app.initial_state = zeros(app.nx * app.agent_num, 1);
app.adjacency = zeros(app.agent_num, app.agent_num);

app.initial_state(1:3,1) = [0 0 1]';
app.initial_state(4:6,1) = [0 1 1]';
app.initial_state(7:9,1) = [1 0 1.1]';

app.adjacency(1,2) = 1;
app.adjacency(2,3) = 1;
app.adjacency(3,1) = 1;

app.anchor_pos = zeros(2,4);
app.anchor_pos(:,1) = [0 0]';
app.anchor_pos(:,2) = [0 5]';
app.anchor_pos(:,3) = [5 5]';
app.anchor_pos(:,4) = [5 0]';

%% simulation init
app.dt = 0.1;
app.iteration = 140;

[app.F, app.J_F] = dynamics_nonholonomic(app.dt);
[app.H, app.J_H] = measurement_update(app.anchor_pos);
[app.R, app.J_R] = relative_measurement_update();


input = zeros(app.nu, app.agent_num, app.iteration);

app.cp_targets = 1;
estimator = cell(app.cp_targets,app.agent_num);

for ct = 1:app.agent_num
   estimator{1,ct} = DFIR(app.nh,app.nx,app.nz,app.nu,app.F,app.J_F,app.H,app.J_H,app.R,app.J_R,0);  
end



for ct = 1:app.iteration
    
end


for ct = 1:app.iteration
    
end























































































