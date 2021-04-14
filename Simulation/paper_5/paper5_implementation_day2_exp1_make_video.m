% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2021-04-14
% First, run code paper5_implementation_day2_exp1.m


videoSave = 0;
figure(10000);
clf;
set(gcf,'Position',[300 100 700 650]);
error_kf_subplot = cell(1, app.nx);
error_kf_data = cell(app.nx, app.agent_num);
error_kf_xlim = zeros(app.nx, 2);
error_kf_xlim(1,:) = [0 app.iteration];
error_kf_xlim(2,:) = error_kf_xlim(1,:);
error_kf_xlim(3,:) = error_kf_xlim(1,:);
error_kf_ylim = zeros(app.nx, 2);
error_kf_ylim(1,:) = [0, 1.5];
error_kf_ylim(2,:) = [0, 0.5];
error_kf_ylim(3,:) = [0, 0.2];

figure(20000);
clf;
set(gcf,'Position',[400 100 700 650]);
error_DFMERM_subplot = cell(1, app.nx);
error_DFMERM_data = cell(app.nx, app.agent_num);
error_DFMERM_xlim = zeros(app.nx, 2);
error_DFMERM_xlim(1,:) = [0 app.iteration];
error_DFMERM_xlim(2,:) = error_DFMERM_xlim(1,:);
error_DFMERM_xlim(3,:) = error_DFMERM_xlim(1,:);
error_DFMERM_ylim = zeros(app.nx, 2);
error_DFMERM_ylim(1,:) = [0, 1.5];
error_DFMERM_ylim(2,:) = [0, 0.5];
error_DFMERM_ylim(3,:) = [0, 0.2];

plot_shape = ["-","-","-h", "-o", "-x", "-d"];
interval = app.horizon_size.RDFIR:app.iteration-1;
markersize = 7;
for nx = 1:app.nx
    
    figure(10000);
    error_kf_subplot{nx} = subplot(3,1,nx); xlim([1,100]); ylim([0,2]);
    for ag = 1:app.agent_num
        if(app.digraph.Nodes.Type{ag} == "known")
            
        else
            error_kf_data{nx, ag} = plot(0, 0,plot_shape(ag),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(ag)); hold on;
            xlim(error_kf_xlim(nx,:));
            ylim(error_kf_ylim(nx,:));
        end
    end
    legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
    xlabel("(a)", 'FontSize', 13);
    ylabel("estimation error", 'FontSize', 13);
    
    figure(20000);
    error_DFMERM_subplot{nx} = subplot(3,1,nx); xlim([1,100]); ylim([0,2]);
    for ag = 1:app.agent_num
       if(app.digraph.Nodes.Type{ag} == "known")
           
       else
            error_DFMERM_data{nx, ag} = plot(0, 0,plot_shape(ag),'LineWidth',1.2,'MarkerSize',markersize, 'DisplayName', num2str(ag)); hold on;
            xlim(error_DFMERM_xlim(nx,:));
            ylim(error_DFMERM_ylim(nx,:));
       end
    end
    legend('FontSize', 15, 'NumColumns',3, 'Location', 'northwest','LineWidth',1.2);
    xlabel("(a)", 'FontSize', 13);
    ylabel("estimation error", 'FontSize', 13);
end



for ct = 1:size(interval,2)
    disp(ct);
    if ct > app.iteration
       break; 
    end
    for ag = 1:app.agent_num
        disp(ag);
        if(app.digraph.Nodes.Type{ag} == "known")
            
        else
            error_kf_data{1, ag}.XData = interval(1:ct);
            error_kf_data{1, ag}.YData = result.agent(ag).RDEKF.error(1,1:ct);
            error_kf_data{2, ag}.XData = interval(1:ct);
            error_kf_data{2, ag}.YData = result.agent(ag).RDEKF.error(2,1:ct);
            error_kf_data{3, ag}.XData = interval(1:ct);
            error_kf_data{3, ag}.YData = result.agent(ag).RDEKF.error(3,1:ct);
            
            error_DFMERM_data{1, ag}.XData = interval(1:ct);
            error_DFMERM_data{1, ag}.YData = result.agent(ag).RDFIR.error(1,1:ct);
            error_DFMERM_data{2, ag}.XData = interval(1:ct);
            error_DFMERM_data{2, ag}.YData = result.agent(ag).RDFIR.error(2,1:ct);
            error_DFMERM_data{3, ag}.XData = interval(1:ct);
            error_DFMERM_data{3, ag}.YData = result.agent(ag).RDFIR.error(3,1:ct);
        end
        F1(ct) = getframe(10000);
        F2(ct) = getframe(20000);
    end
    
end
%% save video
if videoSave == 1
    video_name = sprintf('day2_exp1');
    video = VideoWriter(video_name, 'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.07;
    open(video);
    writeVideo(video,F1);
    close(video);
end

% if app.make_video == 1
%     video_name = sprintf('day3_exp2_kidnap_%s_%s',datestr(now,'yymmdd'),datestr(now,'HHMMSS'));
%     video = VideoWriter(video_name,'MPEG-4');
%     video.Quality = 100;
%     video.FrameRate = 1/0.05;   % 영상의 FPS, 값이 클수록 영상이 빨라짐
%     open(video);
%     writeVideo(video,F);
%     close(video);
% end
