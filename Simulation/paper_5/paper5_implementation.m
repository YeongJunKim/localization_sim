% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs

% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-10-18
% 
% https://www.github.com/YeongJunKim/matlab/filters/
%
% Important thing to achieve the sim & ex.


clear all;
close all;


addpath('./../../../matlab/filters/');
addpath('./../../../matlab/filters/FIR');
addpath('./../../../matlab/filters/RDFIR');
addpath('./../../../matlab/filters/EKF');

global app;
global estimator;


% app basic setting
% robot num, input, state, measurement setting,
% adjacency matrix are defined
% matlab functions
r = app_settings();disp(r);
r = app_initialization();disp(r);

%% make real
for ct = 1:app.iteration
    
    if ct < 1
        app.result.agent(1).input(:,ct) = normrnd([0.2 -0.2]', 0.1);
        app.result.agent(2).input(:,ct) = normrnd([0.1 0.15]', 0.1);
        app.result.agent(3).input(:,ct) = normrnd([0.1 0.2]', 0.1);
        app.result.agent(4).input(:,ct) = normrnd([0.1 0.16]', 0.1);
        app.result.agent(5).input(:,ct) = normrnd([0.1 -0.21]', 0.1);
        app.result.agent(6).input(:,ct) = normrnd([0.1 -0.21]', 0.1);
    elseif ct < 1
        app.result.agent(1).input(:,ct) = normrnd([0.1 0.1]', 0.1);
        app.result.agent(2).input(:,ct) = normrnd([0.15 -0.3]', 0.1);
        app.result.agent(3).input(:,ct) = normrnd([0.2 -0.3]', 0.1);
        app.result.agent(4).input(:,ct) = normrnd([0.4 -0.36]', 0.1);
        app.result.agent(5).input(:,ct) = normrnd([0.3 0.13]', 0.1);
        app.result.agent(6).input(:,ct) = normrnd([0.3 0.13]', 0.1);
    else
        app.result.agent(1).input(:,ct) = normrnd([0.1 0.1]', 0.000001);
        app.result.agent(2).input(:,ct) = normrnd([0.25 0.3]', 0.000001);
        app.result.agent(3).input(:,ct) = normrnd([0.2 -0.35]', 0.00001);
        app.result.agent(4).input(:,ct) = normrnd([0.1 -0.3]', 0.0000001);
        app.result.agent(5).input(:,ct) = normrnd([0.15 0.4]', 0.000001);
        app.result.agent(6).input(:,ct) = normrnd([0.15 0.4]', 0.000001);
    end
%     disp(ct)
%     app.result.agent(i)
    for i = 1:app.agent_num
        x = app.result.agent(i).trajectory.real(1,ct);
        y = app.result.agent(i).trajectory.real(2,ct);
        theta = app.result.agent(i).trajectory.real(3,ct);
        ul = app.result.agent(i).input(1,ct);
        ua = app.result.agent(i).input(2,ct);
        app.result.agent(i).trajectory.real(:,ct+1) = app.F(x, y, theta, ul, ua);
%         app.result.agent(i).trajectory.real(3,ct+1) = wrapTo2Pi(app.result.agent(i).trajectory.real(3,ct+1));
               
    end
end

for i = 1:app.agent_num
        app.ax2_plots{i}.XData = app.result.agent(i).trajectory.real(1,1:ct+1);
        app.ax2_plots{i}.YData = app.result.agent(i).trajectory.real(2,1:ct+1); 
end

%% estimate
for i = 1:app.agent_num
   estimator{app.index_RDFIR, i}.count = 1; 
end
for ct = 1:app.iteration
    %     disp(ct);
    pj_ = zeros(2,app.agent_num);
    for i = 1:app.agent_num
        pj_(:,i) = app.result.agent(i).trajectory.real(1:2,ct);
    end
    
    for i = 1:app.agent_num
        if(app.digraph.Nodes.Type{i} == "known")
            
        else
            % make measurment
            find_neighbors = find(app.adj_full(:,i)==1);
            nn = size(find_neighbors, 1);
            z = zeros(nn * 2 + 1, 1);
            for j = 1:nn
                x1 = pj_(:,i);
                x2 = pj_(:,find_neighbors(j));
                z(j) = norm(x1 - x2);
                z(nn+j) = (atan2(x2(2)-x1(2), x2(1)-x1(1)))- wrapTo2Pi(app.result.agent(i).trajectory.real(3,ct));
%                 z(nn+j) = wrapTo2Pi(z(nn+j));
            end
            z(nn*2+1) = app.result.agent(i).trajectory.real(1,ct);
%             z(nn*2+1) = wrapTo2Pi(z(nn*2+1));
            
%             if(i == 3)
%                disp(z) 
%             end
            
            estimator{app.index_RDFIR, i}.estimate2(i,app.result.agent(i).input(:,ct), z, app.adj_full, pj_);
           
            
        end
    end
    
    
end

x = zeros(1,app.iteration);
y = zeros(1,app.iteration);
interval = 1:app.iteration;
for i = 1:app.agent_num
        if(app.digraph.Nodes.Type{i} == "known")
        else
           x(:) = estimator{app.index_RDFIR, i}.x_appended(1,interval);
           y(:) = estimator{app.index_RDFIR, i}.x_appended(2,interval);
           plot(app.ax2, x,y); hold on;
        end
end




















































