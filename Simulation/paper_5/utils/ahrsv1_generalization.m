function ahrsv1_generalization(fig_on_off, offset)

global experiment_data
global app

%% init ahrsv1 data && determine rotations
plot_shape = ["-h","-*","-+", "-o", "-x", "-d"];
plot_colors = ["r", "g", "c", "b", "m", "k"];
for ag = 1:app.agent_num
   for ct = 1:app.iteration
       if ag == 1
          continue 
       else
           if experiment_data(ag).ahrsv1(1,ct) == 0
               experiment_data(ag).ahrsv1(1,ct) = experiment_data(ag).ahrsv1(1,ct-1);
           end
       end
   end
end
app.rotations = zeros(app.agent_num, 1);
for ag = 1:app.agent_num
    experiment_data(ag).ahrsv1 = deg2rad(experiment_data(ag).ahrsv1);
    experiment_data(ag).ahrsv1_filtered = experiment_data(ag).ahrsv1;
    for ct = 1:app.iteration
        if ct == 1
            continue
        elseif experiment_data(ag).ahrsv1(ct) < -1.5 && experiment_data(ag).ahrsv1(ct-1) > 1.5
           app.rotations(ag) = app.rotations(ag) + 1;
           fprintf("ag: %d, index: %d, rotation + 1\n", ag, ct);
        elseif experiment_data(ag).ahrsv1(ct) > 1.5 && experiment_data(ag).ahrsv1(ct-1) < -1.5
           app.rotations(ag) = app.rotations(ag) - 1;
           fprintf("ag: %d, index: %d, rotation - 1\n", ag, ct);
        end
        experiment_data(ag).ahrsv1_filtered(ct) = experiment_data(ag).ahrsv1(ct) + app.rotations(ag) * deg2rad(360);
        
    end
    experiment_data(ag).ahrsv1_filtered(:) = experiment_data(ag).ahrsv1_filtered(:) - experiment_data(ag).ahrsv1_filtered(1);
end
legend();
if(fig_on_off)
fig_ahrsv1 = figure();
fig_ahrsv1.Name = 'ahrsv1';
clf;
end
for ag = 1:app.agent_num
    experiment_data(ag).ahrsv1 = experiment_data(ag).ahrsv1_filtered + offset(ag);
    if(fig_on_off)
    plot(app.interval, experiment_data(ag).ahrsv1(app.interval),plot_shape(ag), 'DisplayName', num2str(ag)); hold on;
    end
end