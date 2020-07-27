clear all
%% add path
addpath('./../matlab/filters');
%% global variables
global app
%% robot init
app.agent_num = 8;
app.adjacency = zeros(app.agent_num, app.agent_num);
app.adjacency(1,[2 6 7 3]) = 1;
app.adjacency(2,[1 8]) = 1;
app.adjacency(3,[1 8 4]) = 1;
app.adjacency(4,[3 8 5]) = 1;
app.adjacency(5,[6 7 8 4]) = 1;
app.adjacency(6,[1 7 5]) = 1;
app.adjacency(7,[1 6 5 8]) = 1;
app.adjacency(8,[2 3 4 5 7]) = 1;

%% simulation init
app.iteration = 100;
for ct = 1:app.agent_num
   app.agent(ct).data.nx = 4;
   app.agent(ct).data.ny = 4;
   app.agent(ct).data.nz = 4;
   app.agent(ct).trajectory.real = zeros(app.agent(ct).data.nx, app.iteration);
   app.agent(ct).trajectory.filtered = zeros(app.agent(ct).data.nx, app.iteration);
   app.agent(ct).trajectory.measurement = zeros(app.agent(ct).data.ny, app.iteration);
   app.agent(ct).trajectory.time = zeros(1, app.iteration);
   app.agent(ct).trajectory.input = zeros(1, app.iteration);   
   app.agent(ct).trajectory.real(:,1) = normrnd([0 0 0 0]', [5 5 0 0]');
end
%% plotting init
figure(1);
clf;
ax = axes;
app.plot_agent = cell(1,app.agent_num);
app.plot_edge = cell(app.agent_num, app.agent_num);
for ct = 1:app.agent_num
   app.plot_agent{ct} = plot(ax, app.agent(ct).trajectory.real(1,1), app.agent(ct).trajectory.real(2,1), 'r*'); hold on; grid on;
end
for i = 1:app.agent_num-1
    for j = i+1:app.agent_num
        if i == j
        else
            if(app.adjacency(i,j) == 1)
            x = zeros(1,2);
            y = zeros(1,2);
            x(1) = app.agent(i).trajectory.real(1,1); x(2) = app.agent(j).trajectory.real(1,1);
            y(1) = app.agent(i).trajectory.real(2,1); y(2) = app.agent(j).trajectory.real(2,1);
            app.plot_edge{i,j} = plot(ax, x,y, 'b-'); hold on;
            end
        end
    end
end
xlim([-20 20])
ylim([-20 20])
%% filtering class init
% examples
% global filters
% for ct = 1:app.agent_num
%    filters.FIR(ct).filter = FIR();
%    filters.EKF(ct).filter = EKF();
%    filters.PF(ct).filter = PF();
% end
%% run simulation
% make real data
for ct = 1:app.iteration
    for ag = 1:app.agent_num
        
    end
end

% run filtering
for ct = 1:app.iteration
    for ag = 1:app.agent_num
       
        
        update_plot(ag); 
    end
end

%% local functions
function update_plot(index)
global app
for ct = 1:app.agent_num
   app.plot_agent{ct}.XData = app.agent(ct).trajectory.real(1,index);
   app.plot_agent{ct}.YData = app.agent(ct).trajectory.real(2,index);
end
end