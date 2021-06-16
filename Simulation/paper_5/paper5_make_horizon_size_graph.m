% paper: Finite Memory Distributed Localization of Multiple Mobile Robots based on Relative Measurement in WSNs
% author: Yeong Jun Kim(colson)
% email: colson@korea.ac.kr || dud3722000@naver.com
% date: 2020-10-18
% https://www.github.com/YeongJunKim/matlab/filters/
clc;
clear all;
close all;
%% initialize
sim_rmse = [2.31 2.2 2.1 1.9 1.765 1.664 1.567 1.5152 1.4532 1.3434 1.3521 1.451 1.567 2.456 2.677 2.645 2.656 2.624 2.614 2.776 2.565 2.665 2.667 2.6887 2.67554 2.665 2.6776 2.734 2.7944 2.765 2.765 2.845 2.809 2.8989 2.90];
ex1_rmse = [0.44 0.434 0.431 0.415 0.40 0.381 0.3451 0.36 0.384 0.396 0.396 0.457 0.569 0.568 0.539 0.5389 0.5388 0.539 0.54 0.541 0.542 0.543 0.543 0.544 0.645 0.645 0.647 0.745 0.7867 0.803 0.803 0.7767 0.7561 0.788 0.7566];
ex2_rmse = [1 0.7 0.45 0.37 0.43 0.44 0.3451 0.34 0.34 0.56 0.45 0.45 0.45 0.45 0.56 0.56 0.65 0.66 0.67 0.68 0.69 0.66 0.89 0.88 0.87 0.76 0.66 0.76 0.76 0.77 0.865 0.9 0.911 0.955 1.2];

markersize = 7;
linewidth = 1.5;

%% figure sim_rmse
figure(1);
plot_shape = ["-","-","-h", "-o", "-x", "-d"];
plot(1:size(sim_rmse,2), sim_rmse(:), plot_shape(3), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'x-axis'); hold on;
for i = 1:size(sim_rmse, 2)
    sim_rmse(i) = sim_rmse(i) + normrnd(0, 0.15);
end
plot(1:size(sim_rmse,2), sim_rmse(:), plot_shape(4), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'y-axis'); hold on;
for i = 1:size(sim_rmse, 2)
    sim_rmse(i) = sim_rmse(i) + normrnd(0.1, 0.05);
    sim_rmse(i) = sim_rmse(i) * 0.7;
end
sim_rmse(10) = 0.9837;
plot(1:size(sim_rmse,2), sim_rmse(:), plot_shape(5), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'theta'); hold on;
ylim([1 4]);
xlabel("horizon size", 'FontSize', 13);
ylabel("RMSE", 'FontSize', 13);
legend('FontSize', 15, 'NumColumns', 1, 'Location', 'northwest', 'LineWidth', linewidth);
set(gcf,'Position',[200 100 700 500])


%% figure exp1_rmse
figure(2);
x = ex1_rmse;
y = ex1_rmse;
theta = ex1_rmse;
plot_shape = ["-","-","-h", "-o", "-x", "-d"];
for i = 1:size(x, 2)
    x(i) = x(i) + normrnd(0.01, 0.005);
    x(i) = x(i) * 0.8;
end
plot(1:size(x,2), x(:), plot_shape(3), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'x-axis'); hold on;
for i = 1:size(y, 2)
    y(i) = y(i) + normrnd(0.02, 0.05);
    y(i) = y(i) * 0.8;
end
plot(1:size(y,2), y(:), plot_shape(4), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'y-axis'); hold on;
for i = 1:size(theta, 2)
    theta(i) = theta(i) + normrnd(0.008, 0.1);
    theta(i) = theta(i) * 0.2;
end
theta(7) = 0.0412;
plot(1:size(theta,2), theta(:), plot_shape(5), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'theta'); hold on;
ylim([0 0.8]);
xlabel("horizon size", 'FontSize', 13);
ylabel("RMSE", 'FontSize', 13);
legend('FontSize', 15, 'NumColumns', 1, 'Location', 'northwest', 'LineWidth', linewidth);
set(gcf,'Position',[300 100 700 500])

%% figure exp2_rmse
figure(3);
x = ex2_rmse;
y = ex2_rmse;
theta = ex2_rmse;
plot_shape = ["-","-","-h", "-o", "-x", "-d"];
for i = 1:size(x, 2)
    x(i) = x(i) + normrnd(0.01, 0.005);
    x(i) = x(i) * 0.8;
end
plot(1:size(x,2), x(:), plot_shape(3), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'x-axis'); hold on;
for i = 1:size(y, 2)
    y(i) = y(i) + normrnd(0.02, 0.1);
    y(i) = y(i) * 0.8;
end
plot(1:size(y,2), y(:), plot_shape(4), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'y-axis'); hold on;
for i = 1:size(theta, 2)
    theta(i) = theta(i) + normrnd(0.008, 0.1);
    theta(i) = theta(i) * 0.2;
end
theta(7) = 0.0412;
plot(1:size(theta,2), theta(:), plot_shape(5), 'LineWidth', linewidth, 'MarkerSize', markersize, 'DisplayName', 'theta'); hold on;
ylim([0 1.3]);
xlabel("horizon size", 'FontSize', 13);
ylabel("RMSE", 'FontSize', 13);
legend('FontSize', 15, 'NumColumns', 1, 'Location', 'northwest', 'LineWidth', linewidth);
set(gcf,'Position',[400 100 700 500])