
plot(1)
clf

xi1 = [1 2 deg2rad(180)]';
xj1 = [1 4 0]';


l = norm(xi1(1:2)' - xj1(1:2)');

disp(l);

plot(xi1(1), xi1(2), '*'); hold on;
plot(xj1(1), xj1(2), 'd');

xlim([0 6]);
ylim([0 6]);


diff = xj1(1:2) - xi1(1:2);

eta = deg2rad(270);
theta = deg2rad(180);

x = l*cos(eta - theta);
y = l*sin(eta - theta);

fprintf("state substraction \n");
fprintf("diff x : %f    diff y: %f \r\n", x, y);
fprintf("with lidar information \n");
fprintf("diff x : %f    diff y: %f \r\n", x, y);


%%
    for i = 1:app.agent_num
        if(app.digraph.Nodes.Type{i} == "known")
            
        else
            % make measurment
            find_neighbors = find(app.adj_full(:,i)==1);
            j = size(find_neighbors,1);
            fprintf("%d th robot's neighbors : %d \r\n", i, j);
            find_neighbors = find(app.adj_full(:,i)==1);
            for j = 1:size(find_neighbors,1)
%                fprintf("%d,", find_neighbors(j)); 
            end
%             fprintf("\n\n\n");
            
        end
        
    end



