clc;
clear;

simTime = 999;
data_path = 'data.mat';
load(data_path)

demo_index = 6;
ref_x = demos_3D{demo_index}.pos(1,:);
ref_y = demos_3D{demo_index}.pos(2,:);
ref_z = demos_3D{demo_index}.pos(3,:);
ref_vx = demos_3D{demo_index}.vel(1,:);
ref_vy = demos_3D{demo_index}.vel(2,:);
ref_vz = demos_3D{demo_index}.vel(3,:);
dt = demos_3D{demo_index}.dt;
t = demos_3D{demo_index}.t;

x0 = 20;
y0 = 5;
z0 = 2;

%% PID控制
tsim = t(end);
ts = dt;
ref_trajectory_x = timeseries(ref_x,t);
ref_trajectory_y = timeseries(ref_y,t);
ref_trajectory_z = timeseries(ref_z,t);
sim('pid_simu_3D.slx')
pid_x = actual_position_x';
pid_y = actual_position_y';
pid_z = actual_position_z';
pid_vx = actual_velocity_x';
pid_vy = actual_velocity_y';
pid_vz = actual_velocity_z';

pid_bias = sqrt((pid_x - ref_x).^2 + (pid_y - ref_y).^2 + (pid_z - ref_z).^2);

%% SMC
% M = 1;
% B = 0;
% K_x = 300;
% lamda_x = 10;
% K_y = 300;
% lamda_y = 10;
% sim('smc_lasa.slx')
% smc_x = position_x';
% smc_y = position_y';
% smc_vx = velocity_x';
% smc_vy = velocity_y';
% smc_bias = sqrt((smc_x - ref_x).^2 + (smc_y -ref_y).^2);

%% LNN 
[LNN_x, LNN_y, LNN_z, LNN_vx, LNN_vy, LNN_vz] = LNN_comparison_3D(ref_x, ref_y, ref_z, ref_vx, ref_vy, ref_vz, x0, y0, z0, dt);
LNN_bias = sqrt((LNN_x - ref_x).^2 + (LNN_y - ref_y).^2 + (LNN_z - ref_z).^2);

%% RNN
% RNN_csv = readtable('D:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\RNN_track_data\Angle_1.csv');
% RNN_x = RNN_csv.RNN_x';
% RNN_y = RNN_csv.RNN_y';
% RNN_vx = RNN_csv.RNN_vx';
% RNN_vy = RNN_csv.RNN_vy';
[RNN_x, RNN_y, RNN_z, RNN_vx, RNN_vy, RNN_vz] = RNN_comparison_3D(ref_x, ref_y, ref_z, ref_vx, ref_vy, ref_vz, x0, y0, z0, dt);
RNN_bias = sqrt((RNN_x - ref_x).^2 + (RNN_y - ref_y).^2 + (RNN_z - ref_z).^2);

%% ELM 
[ELM_x,ELM_y,ELM_z ,ELM_vx, ELM_vy, ELM_vz] = ELM_comparison_3D(ref_x, ref_y, ref_z, ref_vx, ref_vy, ref_vz, x0, y0, z0, dt);
ELM_bias = sqrt((ELM_x - ref_x).^2 + (ELM_y - ref_y).^2 + (ELM_z - ref_z).^2);

%% BLS 
% [BLS_x,BLS_y,BLS_vx,BLS_vy] = BLS_comparison(ref_x, ref_y, ref_vx, ref_vy, x0, y0, dt);
% BLS_bias = sqrt((BLS_x - ref_x).^2 + (BLS_y - ref_y).^2);

%% 
set(0, 'DefaultFigurePosition', [100, 100, 800,600 ])
set(groot, 'DefaultAxesFontName', 'Times New Roman','DefaultTextFontSize', 14);
set(groot, 'DefaultTextFontName', 'Times New Roman','DefaultTextFontSize', 14); 
figure;
view(3)
hold on;
plot3(LNN_x, LNN_y, LNN_z, 'LineWidth', 1.0, 'color', [0.8824 0.3412 0.3490]);
plot3(pid_x, pid_y, pid_z, 'LineWidth', 1, 'LineStyle', '--', 'color', [0.9490 0.5569 0.1686]);
% plot3(smc_x, smc_y, smc_z, 'LineWidth', 1, 'color', [1.0000 0.7647 0.6275]);
plot3(ELM_x, ELM_y, ELM_z, 'LineWidth', 1, 'LineStyle', ':', 'color', [0.3059 0.4745 0.6549]);
plot3(RNN_x, RNN_y, RNN_z, 'LineWidth', 1, 'LineStyle', '-.', 'color', [0.3490 0.6314 0.3098]);
% plot3(BLS_x, BLS_y, BLS_z, 'LineWidth', 1, 'color', [0.6588 0.5608 0.7294]);
plot3(ref_x, ref_y, ref_z, 'LineWidth', 1.5, 'LineStyle', '--', 'color', 'k');
plot3(ref_x(1), ref_y(1), ref_z(1), '*', 'color', 'k');
plot3(x0, y0, z0, 'p', 'color', 'b', 'MarkerFaceColor', 'b');
grid on;
box on;
legend('Trajectory of LNN method', 'Trajectory of PID method', 'Trajectory of ELM method', 'Trajectory of RNN method', 'Desired Trajectory', 'Desired Start Point', 'Actual Start Point');
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');


figure;
hold on;
grid on;
box on;
plot(t, LNN_bias, 'LineWidth', 1, 'color', [0.8824 0.3412 0.3490]);
plot(t, pid_bias, 'LineWidth', 1, 'LineStyle', '--', 'color', [0.9490 0.5569 0.1686]);
plot(t, ELM_bias, 'LineWidth', 1, 'LineStyle', ':', 'color', [0.3059 0.4745 0.6549]);
% plot(t, smc_bias, 'LineWidth', 1.5, 'color', [1.0000 0.7647 0.6275]);
plot(t, RNN_bias, 'LineWidth', 1, 'LineStyle', '-.', 'color', [0.3490 0.6314 0.3098]);
legend('LNN method', 'PID method', 'ELM method', 'RNN method');
xlabel('Time (s)');
ylabel('Position bias (mm)');