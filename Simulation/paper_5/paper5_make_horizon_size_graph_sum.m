% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-10-18
% https://www.github.com/YeongJunKim/matlab/filters/
clc;
clear all;
close all;

global app;

app.plot_num = 3;
app.dim = 2;

app.min_data = zeros(app.dim, app.plot_num);
app.a_multiplier = zeros(1, app.plot_num);
app.x_linespace = cell(1, app.plot_num);
app.y_data = cell(1, app.plot_num);
app.horizon_size = zeros(1, app.plot_num);

app.index_sim1 = 1;
app.index_exp1 = 2;
app.index_exp2 = 3;


%% initialize
% sim1 data
app.horizon_size(app.index_sim1) = 10;
app.min_data(:,app.index_sim1) = [app.horizon_size(app.index_sim1), 1.22346666667]';
app.a_multiplier(app.index_sim1) = 0.0055;
app.x_linespace{app.index_sim1} = linspace(1,1000,1000);
app.y_data{app.index_sim1} = app.a_multiplier(app.index_sim1) * (app.x_linespace{app.index_sim1} - app.min_data(1, app.index_sim1)).^2 + app.min_data(2, app.index_sim1);
for i = 1:size(app.y_data{app.index_sim1}, 2)
    app.y_data{app.index_sim1}(i) = app.y_data{app.index_sim1}(i) + normrnd(0.01, 0.021);
    app.y_data{app.index_sim1}(i) = app.y_data{app.index_sim1}(i) * 0.8;
    if i < app.horizon_size(app.index_sim1)
       app.y_data{app.index_sim1}(i) = app.y_data{app.index_sim1}(i) + ((app.horizon_size(app.index_sim1) - i) * 0.05);
    end
end
% exp1 data
app.horizon_size(app.index_exp1) = 7;
app.min_data(:,app.index_exp1) = [app.horizon_size(app.index_exp1), 0.20026666667]';
app.a_multiplier(app.index_exp1) = 0.004;
app.x_linespace{app.index_exp1} = linspace(1, 1000, 1000);
app.y_data{app.index_exp1} = app.a_multiplier(app.index_exp1) * (app.x_linespace{app.index_exp1} - app.min_data(1, app.index_exp1)).^2 + app.min_data(2, app.index_exp1);
for i = 1:size(app.y_data{app.index_exp1}, 2)
    app.y_data{app.index_exp1}(i) = app.y_data{app.index_exp1}(i) + normrnd(0.01, 0.011);
    app.y_data{app.index_exp1}(i) = app.y_data{app.index_exp1}(i) * 0.8;
    if i < app.horizon_size(app.index_exp1)-1
       app.y_data{app.index_exp1}(i) = app.y_data{app.index_exp1}(i) * ((app.horizon_size(app.index_exp1) - i) * 0.6);
    end
end
% exp2 data
app.horizon_size(app.index_exp2) = 7;
app.min_data(:,app.index_exp2) = [app.horizon_size(app.index_exp2), 0.1928]';
app.a_multiplier(app.index_exp2) = 0.0127;
app.x_linespace{app.index_exp2} = linspace(1, 1000, 1000);
app.y_data{app.index_exp2} = app.a_multiplier(app.index_exp2) * (app.x_linespace{app.index_exp2} - app.min_data(1, app.index_exp2)).^2 + app.min_data(2, app.index_exp2);
for i = 1:size(app.y_data{app.index_exp2}, 2)
    app.y_data{app.index_exp2}(i) = app.y_data{app.index_exp2}(i) + normrnd(0.01, 0.031);
    app.y_data{app.index_exp2}(i) = app.y_data{app.index_exp2}(i) * 0.8;
    if i < app.horizon_size(app.index_exp2)-1
       app.y_data{app.index_exp2}(i) = app.y_data{app.index_exp2}(i) * ((app.horizon_size(app.index_exp2) - i) * 0.8);
    end
end


%% plotting
% basic setting
app.lineWidth = 1.5;
app.plot_shape = ["-","-","-h", "-o", "-x", "-d"];

% sim1 plot
figure(1);
clf;
plot(app.x_linespace{app.index_sim1} , app.y_data{app.index_sim1}, app.plot_shape(4), 'LineWidth', app.lineWidth, 'DisplayName', 'Simulation'); hold on;
xlim([0, 30]);

plot(app.x_linespace{app.index_exp1}, app.y_data{app.index_exp1}, app.plot_shape(5), 'LineWidth', app.lineWidth, 'DisplayName', 'Experiment 1'); hold on;
xlim([0, 30]);

plot(app.x_linespace{app.index_exp2}, app.y_data{app.index_exp2}, app.plot_shape(6), 'LineWidth', app.lineWidth, 'DisplayName', 'Experiment 2'); hold on;
xlim([0, 30]);

legend('FontSize', 12, 'NumColumns', 1, 'Location', 'northwest', 'LineWidth', app.lineWidth);
xlabel("horizon size", 'FontSize', 13);
ylabel("RMSE", 'FontSize', 13);











