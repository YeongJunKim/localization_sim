function r = app_settings()
global app

app.relative_scenario_distance_eta = 1;
app.relative_scenario_poss_diff = 2;

app.initial_error_scenario_error = 1;
app.initial_error_scenario_normal = 2;

app.relative_scenario = app.relative_scenario_distance_eta;

app.initial_error_scenario = app.initial_error_scenario_error;

app.make_video = 1;

% abcdefghij klmnopqrst uvxyz
app.agent_name = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f", "tb3g", "tb3h", "tb3i", "tb3j", ...
                  "tb3k", "tb3l", "tb3m", "tb3n", "tb3o", "tb3p", "tb3q", "tb3r", "tb3s", "tb3t", ...
                  "tb3u", "tb3v", "tb3x", "tb3y", "tb3z"];
app.agent_type = ["","","","","","","","","","", ...
                  "","","","","","","","","","", ...
                  "","","","",""];

app.agent_name = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", ...
                  "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", ...
                  "21", "22", "23", "24", "25"];
app.agent_num = 25;
app.nx = 3;
app.nu = 2;
app.nh = cell(app.agent_num, 1);
app.horizon_size.RDFIR = 10;
app.horizon_size.FIR = 6;
app.anchor_num = 4;

app.iteration = 500;

app.dt = 0.1;

app.estimator_num = 2;
app.index_RDFIR = 1;
app.index_RDEKF = 2;
app.index_PF  = 3;

% tb3a, tb3b are known robot,
% start(give position) -> end(receive position)
st     = [1 1 1 1 2 2 2 2 3 3 4 4 5 6 6 6 ...
                                            7 7 7 7  8 8  9 10 10 3 5 11 11 6  8  12 12 13 14 14 14 15 15 16 17 17 18 18 18 19 19 19 20 21 21 22 22 23 24 24 25 25 25];
ed     = [3 5 4 6 3 5 6 4 6 4 3 6 4 3 5 10 ...
                                            5 8 9 10 9 10 7 8  9  7 8 12 13 11 11 15 16 17 13 18 20 16 9 20 18 19 14 15 19 20 13 12 4  20 24 23 21 25 22 20 16 17 18];
weight = ones(size(st,2), 1);


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


figure(1);
clf;
ax = axes;
app.plot_diagraph = plot(ax, app.digraph, 'Layout', ...
    'force', 'LineWidth', 2, 'MarkerSize', 8, ...
    'ArrowSize', 10, 'NodeColor', [0.85 0.33 0.10], 'NodeFontSize', 10, ...
    'EdgeAlpha', 0.6, 'EdgeColor', [0.5 0.25 0.8], 'ArrowPosition', 0.9);
% title("Adjacency graph");

for i = 1:app.agent_num
    if app.digraph.Nodes.Type{i} == "known"
        
    end
end

% get(ax)
% get(ax.Children)




r = "app_settings ok";
end