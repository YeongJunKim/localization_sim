function timer_input_generation(obj, event)
global app ROBOTS
persistent first_run
persistent step
persistent u
if isempty(first_run)
    u = zeros(2,6,1000);
    iteration = 60;
    for ct = 1:iteration
        if ct < 10
            u(:,1,ct) = [0.01 0.01]';
            u(:,2,ct) = [0.01 0.01]';
            u(:,3,ct) = [0.03 0.02]';
            u(:,4,ct) = [-0.01 0.0]';
            u(:,5,ct) = [0.01 0.01]';
            u(:,6,ct) = [0.005 -0.03]';
        elseif ct < 20
            u(:,1,ct) = [0.01 0.0]';
            u(:,2,ct) = [0.01 -0.01]';
            u(:,3,ct) = [0.01 0.0]';
            u(:,4,ct) = [0.01 -0.01]';
            u(:,5,ct) = [0.01 0.05]';
            u(:,6,ct) = [0.01 0.05]';
        elseif ct < 30
            u(:,1,ct) = [0.01 0.0]';
            u(:,2,ct) = [0.01 0.02]';
            u(:,3,ct) = [0.01 -0.01]';
            u(:,4,ct) = [0.01 0.02]';
            u(:,5,ct) = [0.02 -0.05]';
            u(:,6,ct) = [0.02 -0.04]';
        elseif ct < 40
            u(:,1,ct) = [0.01 0.01]';
            u(:,2,ct) = [0.01 -0.024]';
            u(:,3,ct) = [0.01 -0.03]';
            u(:,4,ct) = [0.025 0.01]';
            u(:,5,ct) = [0.02 -0.03]';
            u(:,6,ct) = [0.01 0.02]';
        elseif ct < 50
            u(:,1,ct) = [0.01 0.01]';
            u(:,2,ct) = [0.01 0.01]';
            u(:,3,ct) = [0.02 0.06]';
            u(:,4,ct) = [0.025 -0.08]';
            u(:,5,ct) = [0.02 0.08]';
            u(:,6,ct) = [0.01 0.03]';
        end
   
        u(:,1,ct) = [0.03 0.008]';
        u(:,2,ct) = [0.03 0.008]';
        u(:,3,ct) = [0.03 0.008]';
        u(:,4,ct) = [0.03 0.008]';
        u(:,5,ct) = [0.03 0.008]';
        u(:,6,ct) = [0.03 0.008]';
        
    end
    
    step = 1;
    first_run = 1;
    app.stop_flag = 0;
    tic;
end

if app.start_flag == 1
    fprintf("[%d] inputtimer = %d\n",step);
    
    for i = 1:6
        %% publish
        ROBOTS{i}.pub_control_msg.Linear.X = u(1,i,step);
        ROBOTS{i}.pub_control_msg.Angular.Z = u(2,i,step);
        ROBOTS{i}.pub_control.send(ROBOTS{i}.pub_control_msg);
    end
    
    if(1 == app.stop_flag)
        app.tocs = [app.experiment_data(1).tocs(:)'; app.experiment_data(2).tocs(:)'; app.experiment_data(3).tocs(:)'; app.experiment_data(4).tocs(:)'; app.experiment_data(5).tocs(:)'; app.experiment_data(6).tocs(:)' ];
        rosshutdown();
        delete_timer();
        dbquit();
    end
    step = step + 1;
end
end
