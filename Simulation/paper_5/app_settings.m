function r = app_settings()
global app

app.make_video = 1;


app.agent_name = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
app.agent_type = ["","","","","",""];

app.agent_num = 6;
app.nx = 3;
app.nu = 2;
app.nh = cell(app.agent_num, 1);
app.horizon_size.RDFIR = 5;
app.horizon_size.FIR = 6;
app.anchor_num = 4;

app.iteration = 140;

app.dt = 0.1;

app.estimator_num = 2;
app.index_RDFIR = 1;
app.index_EKF = 2;
app.index_PF  = 3;

% tb3a, tb3b are known robot,
% start(give position) -> end(receive position)
st     = [1 1 1 2 2 2 3  5 5 6 6 ];
ed     = [3 4 5 3 5 6 6  6 4 4 3 ];
weight = [1 1 1 1 1 1 1  1 1 1 1 ]';


EdgeTable = table([st' ed'], weight, ...
    'VariableNames', {'EndNodes' 'Weight'});
NodeTable = table(app.agent_name', app.agent_type', ...
    'VariableNames', {'Name' 'Type'});
app.digraph = digraph(EdgeTable, NodeTable);
% app.digraph = digraph(st, ed, weight, app.agent_name);
app.adj = adjacency(app.digraph);
app.adj_full = full(app.adj);

    


% initialization nh, 
% app.nh are defined with directed graph.
for i = 1:app.agent_num 
    neigh = find(app.adj_full(:,i) == 1);
    neigh_num = size(neigh, 1);
    if neigh_num > 0
       app.nh{i} = zeros(neigh_num*2+1, 1);
       app.digraph.Nodes.Type{i} = 'unknown';
    else
       app.digraph.Nodes.Type{i} = 'known';
    end
end


figure('Name', 'Adjacency');
ax = axes;
plot(ax, app.digraph, 'Layout', ...
    'force', 'EdgeLabel', app.digraph.Edges.Weight, 'LineWidth', 2, 'MarkerSize', 15, ...
    'ArrowSize', 10, 'NodeColor', [0.85 0.33 0.10], 'NodeFontSize', 10, ...
    'EdgeAlpha', 0.6, 'EdgeColor', [0.5 0.25 0.8]);
title("Adjacency graph");

% get(ax)
% get(ax.Children)




r = "app_settings ok";
end