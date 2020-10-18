clear all;

addpath('./../../../matlab/filters/NCLA');
addpath('./..');
global app;

app.agent_num = 5;
app.nx = 4;
app.ns = 50;
app.history_size = 100;


app.initial_state = zeros(app.nx, app.agent_num);
app.adjacency = zeros(app.agent_num, app.agent_num);

app.initial_state(:,1) = [1 0 1 0]';
app.initial_state(:,2) = [3 0 4 0]';
app.initial_state(:,3) = [5 0 4 0]';
app.initial_state(:,4) = [1 1 1 1]';
app.initial_state(:,5) = [4 4 3 3]';

app.adjacency(1,[2 3 5]) = 1;
app.adjacency(2,[1 4 5]) = 1;
app.adjacency(3,[1 5 4]) = 1;
app.adjacency(4,[2 3 5]) = 1;
app.adjacency(5,[1 2 3 4]) = 1;

app.A = [1 1 0 0;
        0 1 0 0;
        0 0 1 1;
        0 0 0 1];
app.B = [0.5 0;
        1 0;
        0 0.5;
        0 1];
app.Q = [0.5 0;
        0 0.5];
app.P = diag([1 0.1 1 0.1]);

for ct = 1:app.agent_num
   app.agent(ct).filter = NCLA(app.ns, app.initial_state(:,ct), app.A, app.B, app.Q, app.P, app.adjacency, ct);
end

for it = 1:app.history_size
    
    
end





function r =  get_distance(node1, node2)
    distance = norm(node1 - node2);
    r = distance;
end