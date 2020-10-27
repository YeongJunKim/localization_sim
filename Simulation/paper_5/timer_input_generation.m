function timer_input_generation(obj, event, i)
global app ROBOTS
persistent first_run
persistent step
if isempty(first_run)
    step = 1;
    first_run = 1;
    app.stop_flag = 0;
    tic;
end

if app.start_flag == 1
fprintf("[%d] step = %d\n",i ,step);

for i = 1:app.agent_num
    
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
