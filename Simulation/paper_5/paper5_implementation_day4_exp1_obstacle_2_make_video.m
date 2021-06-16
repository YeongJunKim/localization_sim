% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2021-04-14
% First, run code paper5_implementation_day2_exp1.m


videoSave = 1;

%%
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
error_kf_ylim(2,:) = [0, 1.5];
error_kf_ylim(3,:) = [0, 2];

%%
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
error_DFMERM_ylim(2,:) = [0, 1.5];
error_DFMERM_ylim(3,:) = [0, 2];

%%
figure(30000);
clf;
set(gcf, 'Position', [500 100 700 650]);
errorSum_subplot = cell(1, app.nx);
errorSum_data = cell(app.nx, app.estimator_num);
errorSum_xlim = zeros(app.nx, 2);
errorSum_xlim(1,:) = [0 app.iteration];
errorSum_xlim(2,:) = errorSum_xlim(1,:);
errorSum_xlim(3,:) = errorSum_xlim(1,:);
errorSum_ylim = zeros(app.nx, 2);
errorSum_ylim(1,:) = [0, 2.8];
errorSum_ylim(2,:) = [0 2.7];
errorSum_ylim(3,:) = [0 2.5];
errorSum_FilterName = [sprintf('Distributed \nIIR filter'), "DFMERM"];
errorSum_Colors = zeros(app.estimator_num , 3);
errorSum_Colors(1, :) = [0.00,0.45,0.74];
errorSum_Colors(2, :) = [0.85,0.33,0.10];

% annotation('doublearrow',[0.751428571428571 0.79],[0.888230769230769 0.888230769230769],'Head2Width',5,'Head2Length',5,'Head1Width',5,'Head1Length',5);
% annotation('doublearrow',[0.702857142857143 0.741428571428572],...
%     [0.888230769230769 0.888230769230769],'Head2Width',5,'Head2Length',5,'Head1Width',5,'Head1Length',5);
% annotation('doublearrow',[0.347142857142857 0.385714285714286],[0.888230769230769 0.888230769230769],'Head2Width',5,'Head2Length',5,'Head1Width',5,'Head1Length',5);
% annotation('textbox',[0.381428571428571 0.900538461538462 0.001 0.001],'String',{'A'},'FontSize',13,'FitBoxToText','off');
% annotation('textbox',[0.66 0.900538461538462 0.001 0.001],'String','B','FontSize',13,'FitBoxToText','off');
% annotation('textbox',[0.787428571428571 0.900538461538462 0.00099999999999989 0.001],'String','C','FontSize',13,'FitBoxToText','off');
% annotation('line',[0.34714 0.34714],[0.920538461538462 0.123076923076923],'LineWidth', 1, 'Color', [0 0 0]);
% annotation('line',[0.789997142857143 0.789997142857143],[0.922076923076923 0.124615384615384],'Color',[0 0 0],'LineWidth',1);
% annotation('line',[0.742854285714286 0.742854285714286],[0.920538461538462 0.123076923076923],'Color',[0 0 0],'LineWidth',1);
% annotation('line',[0.704282857142857 0.704282857142857],[0.923615384615385 0.126153846153846],'Color',[0 0 0],'LineWidth',1);
% annotation('line',[0.385711428571429 0.385711428571429],[0.922076923076923 0.124615384615384],'Color',[0 0 0],'LineWidth',1);


%%
disp_name = ["(a)", "(b)", "(c)"];
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
    
    figure(30000);
    errorSum_subplot{nx} = subplot(3,1,nx);
    for fn = 1:app.estimator_num
       errorSum_data{nx, fn} = plot(0,0, '-x','LineWidth',1.2, 'DisplayName', errorSum_FilterName(fn), 'Color', errorSum_Colors(fn,:)); hold on; 
    end
    xlabel(disp_name(nx), 'FontSize', 13);
    ylabel("estimation error", 'FontSize', 13);
    legend([errorSum_data{nx,:}], 'FontSize', 13, 'Location', 'northwest');
    xlim(errorSum_xlim(nx,:));
    ylim(errorSum_ylim(nx,:));
    
end


%%
for ct = 1:size(interval,2)
    disp(ct);
    if ct > app.iteration - 5
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
    end
    for fn = 1:app.estimator_num
       disp(fn);
       errorSum_data{1, 1}.XData = interval(1:ct);
       errorSum_data{1, 1}.YData = error_sum_RDEKF(1,1:ct);
       errorSum_data{2, 1}.XData = interval(1:ct);
       errorSum_data{2, 1}.YData = error_sum_RDEKF(2,1:ct);
       errorSum_data{3, 1}.XData = interval(1:ct);
       errorSum_data{3, 1}.YData = error_sum_RDEKF(3,1:ct);
       
       
       errorSum_data{1, 2}.XData = interval(1:ct);
       errorSum_data{1, 2}.YData = error_sum_RDFIR(1,1:ct);
       errorSum_data{2, 2}.XData = interval(1:ct);
       errorSum_data{2, 2}.YData = error_sum_RDFIR(2,1:ct);
       errorSum_data{3, 2}.XData = interval(1:ct);
       errorSum_data{3, 2}.YData = error_sum_RDFIR(3,1:ct);
       
    end
    
    F1(ct) = getframe(10000);
    F2(ct) = getframe(20000);
    F3(ct) = getframe(30000);
    
end
%% save video
if videoSave == 1
    video_name = sprintf('obstacle_error_KF');
    video = VideoWriter(video_name, 'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.09;
    open(video);
    writeVideo(video,F1);
    close(video);
    
    video_name = sprintf('obstacle_error_DFMERM');
    video = VideoWriter(video_name, 'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.09;
    open(video);
    writeVideo(video,F2);
    close(video);
    
    video_name = sprintf('obstacle_error_sum');
    video = VideoWriter(video_name, 'MPEG-4');
    video.Quality = 100;
    video.FrameRate = 1/0.09;
    open(video);
    writeVideo(video,F3);
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
