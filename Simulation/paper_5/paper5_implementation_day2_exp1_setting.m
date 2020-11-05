function r = app_settings()
global app
%% function defines
app.relative_scenario_distance_eta      = 1;
app.relative_scenario_poss_diff         = 2;
app.initial_error_scenario_error        = 1;
app.initial_error_scenario_normal       = 2;

app.relative_scenario                   = app.relative_scenario_poss_diff;
app.initial_error_scenario              = app.initial_error_scenario_normal;

app.make_video = 1;

%% change variables
app.dt = 1.5;
app.iteration = 100;

% tb3a, tb3b are known robot,
% start(give position) -> end(receive position)
st     = [1 3 3 4 6 2 2 5 2 4 6];
ed     = [5 5 6 6 5 4 6 3 3 3 3];
% st     = [1 1 2 2];
% ed     = [5 3 4 6];
weight = ones(size(st,2), 1);

app.agent_name              = ["tb3a", "tb3b", "tb3c", "tb3d", "tb3e", "tb3f"];
app.agent_type              = ["","","","","",""];
app.agent_name_by_number    = ["1", "2", "3", "4", "5", "6"];
app.agent_num               = 6;
app.nx                      = 3;
app.nu                      = 2;
app.nh                      = cell(app.agent_num, 1);
app.horizon_size.RDFIR      = 7;
app.horizon_size.FIR        = 5;
app.anchor_num              = 4;


app.estimator_num           = 2;
app.index_RDFIR             = 1;
app.index_RDEKF             = 2;
app.index_PF                = 3;
%% general settings

app.plot_colors = ["r", "g", "c", "b", "m", "k"];
app.plot_shapes = ["-*","-h","-+", "-o", "-x", "-d"];

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

app.agent_name_disp_type_robot_name = 1;
app.agent_name_disp_type_number = 2;

app.agent_name_disp_type = app.agent_name_disp_type_number;

if(app.agent_name_disp_type == app.agent_name_disp_type_robot_name)
EdgeTable = table([st' ed'], weight, ...
    'VariableNames', {'EndNodes' 'Weight'});
NodeTable = table(app.agent_name', app.agent_type', ...
    'VariableNames', {'Name' 'Type'});
app.disp_digraph = digraph(EdgeTable, NodeTable);
    
app.plot_diagraph = plot(ax, app.disp_digraph, 'Layout', ...
    'force', 'LineWidth', 2, 'MarkerSize', 8, ...
    'ArrowSize', 10, 'NodeColor', [0.85 0.33 0.10], 'NodeFontSize', 10, ...
    'EdgeAlpha', 0.6, 'EdgeColor', [0.5 0.25 0.8], 'ArrowPosition', 0.9);

elseif(app.agent_name_disp_type == app.agent_name_disp_type_number)
EdgeTable = table([st' ed'], weight, ...
    'VariableNames', {'EndNodes' 'Weight'});
NodeTable = table(app.agent_name_by_number', app.agent_type', ...
    'VariableNames', {'Name' 'Type'});
app.disp_digraph = digraph(EdgeTable, NodeTable);
    
    
app.plot_diagraph = plot(ax, app.disp_digraph, 'Layout', ...
    'force', 'LineWidth', 2, 'MarkerSize', 8, ...
    'ArrowSize', 10, 'NodeColor', [0.85 0.33 0.10], 'NodeFontSize', 14, ...
    'EdgeAlpha', 0.6, 'EdgeColor', [0.5 0.25 0.8], 'ArrowPosition', 0.9);
end

box off;
ax = gca;
ax.XColor = 'white';
ax.YColor = 'white';
for i = 1:app.agent_num
    if app.digraph.Nodes.Type{i} == "known"
        
    end
end

% get(ax)
% get(ax.Children)




r = "app_settings ok";
end