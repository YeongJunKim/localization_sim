% args
% x = state
% adjacency = adjacency matrix
% name = "****" <- tb3a, tb3b, ... tb3*
function r = h_(x, adjacency, name, agents_positions)
global app;

node_num = findnode(app.digraph, name)
if(app.digraph.Nodes.Type{node_num} == "known")
    %        no
else
    find_neighbors = find(app.adj_full(:,node_num)==1);
    nn_ = size(find_neighbors, 1);
    r = zeros(2*nn_ + 1, 1);
end

for i = 1:nn_
    % distance
    r(i) = norm(agents_positions(1:2, find_neighbors(i)) - x(1:2));
    % eta
    xdiff = agents_positions(1,find_neighbors(i)) - x(1);
    ydiff = agents_positions(2,find_neighbors(i)) - x(2);
    rad = atan2(ydiff, xdiff);
    rad = wrapTo2Pi(rad);
    r(nn_ * 1 + i) = rad;
    r(nn_ * 2 + 1) = x(3);
end

% ap = zeros(3, app.agent_num);
% for i = 1:app.agent_num
%    ap(:,i) = normrnd([1 i 0]', [0.5 0.3, 0.01]');
% end
end
