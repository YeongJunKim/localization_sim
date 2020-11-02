%%

global ROBOTS;
initial_state = zeros(3,6);


exp1_settings()

initial_state(:,1) = [0.6 1.8 deg2rad(90)]';
initial_state(:,2) = [1.8 1.2 deg2rad(90)]';
initial_state(:,3) = [1.2 1.8 deg2rad(90)]';
initial_state(:,4) = [1.8 1.8 deg2rad(90)]';
initial_state(:,5) = [0.6 1.2 deg2rad(90)]';
initial_state(:,6) = [1.2 1.2 deg2rad(90)]';



figure(1);
clf;
ax = axes;
plots = cell(1,6);

for i = 1:6
    plots{i} = plot(ax, initial_state(1,i), initial_state(2,i), '-*', 'DisplayName', "aa"); hold on;
end
legend();
grid on;
xlim([0 6]);
ylim([0 6]);

iteration = 100;
state = zeros(3,6,iteration);
u = zeros(2,6,iteration);

u(1,:,:) = 0.01;
u(2,:,:) = 0.01;

nowtick = toc;
pasttick = nowtick;

state(:,:,1) = initial_state(:,:);

for ct = 1:iteration
   if ct < 40
       u(:,1,ct) = [0.01 0.0]';
       u(:,2,ct) = [0.01 0.0]';
       u(:,3,ct) = [0.01 0.0]';
       u(:,4,ct) = [0.01 0.0]';
       u(:,5,ct) = [0.01 0.0]';
       u(:,6,ct) = [0.01 0.0]';
   elseif ct < 90
       u(:,1,ct) = [0.0 -0.01]';
       u(:,2,ct) = [0.0 -0.01]';
       u(:,3,ct) = [0.0 -0.01]';
       u(:,4,ct) = [0.0 -0.01]';
       u(:,5,ct) = [0.0 -0.01]';
       u(:,6,ct) = [0.0 -0.01]';
   elseif ct < 80
       u(:,1,ct) = [0.01 0.0]';
       u(:,2,ct) = [0.01 0.0]';
       u(:,3,ct) = [0.01 0.0]';
       u(:,4,ct) = [0.01 0.0]';
       u(:,5,ct) = [0.02 0.0]';
       u(:,6,ct) = [0.02 0.0]';
   elseif ct < 99
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
       u(:,9,ct) = [0.01 0.03]';
   end
%         u(:,1,ct) = [0.03 0.008]';
%         u(:,2,ct) = [0.03 0.008]';
%         u(:,3,ct) = [0.03 0.008]';
%         u(:,4,ct) = [0.03 0.008]';
%         u(:,5,ct) = [0.03 0.008]';
%         u(:,6,ct) = [0.03 0.008]';
end

steps = 1;
for ct = 1:iteration
    
    for i = 1:6
        state(:,i,ct+1) = dynamics_nonholonomic(state(:,i,ct), u(:,i,ct), 5);
        plots{i}.XData = state(1,i,1:ct);
        plots{i}.YData = state(2,i,1:ct);
    end
    steps = steps +1;
end

%%
nowstep = 1;
for ct = 1:iteration
    nowtick = toc;
    if(nowtick - pasttick > 5)
        fprintf("nowstep : %d\r\n", nowstep);
        for i = 1:6
            %% publish
            ROBOTS{i}.pub_control_msg.Linear.X = 0;
            ROBOTS{i}.pub_control_msg.Angular.Z = 0.001;
            %        msg = rosmessage('std_msgs/cmd_vel');
            topicname = strcat(ROBOTS{i}.namespace, "/cmd_vel");
            disp(topicname);
            ROBOTS{i}.pub_control.send(ROBOTS{i}.pub_control_msg);
            pasttick = nowtick;
            nowstep = nowstep + 1;
        end
    end
    pause(1);
end


%%


function r = dynamics_nonholonomic(x, u, dt)
r = zeros(3,1);

r(1) = x(1) + u(1) * cos(x(3)) * dt;
r(2) = x(2) + u(1) * sin(x(3)) * dt;
r(3) = x(3) + u(2) * dt;
end