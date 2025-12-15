%% 

clc;
clear;
%load('G:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\xonly\LNN_test_xonly_8.mat')
% load('G:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\xonly\LNN_test_xonly_9.mat')
load('D:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\xonly\LNN_test_xonly_con_7.mat')

beta_x = beta;
As_x = As;
tau_x = tau;
% results_x =results;
%f1 = fun_LNN_xonly(beta_x, Number, simTime, interval, dt, tau_x, As_x, position_error, num_node, results_x, draw);  % 这里f = cal_val记得改函数

%load('G:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\yonly\LNN_test_yonly_2.mat')
load('D:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\yonly\LNN_test_yonly_con_7.mat')
beta_y = beta;
As_y = As;
tau_y = tau;
% results_y = results;
%f2 = fun_LNN_yonly(beta_y, Number, simTime, interval, dt, tau_y, As_y, position_error, num_node, results_y, draw);

draw = 1;
%% 
% plot([1:simTime], f(1,:,1));
% hold on;
% plot([1:simTime], results(1,:,1));
% xlabel('Time t');
% ylabel('velocity_error');
% grid on;
% hold on;

demo = 3; 
% plot(0:0.002:1.996, f1(demo,:));
% hold on;
% plot(0:0.002:1.996, results_x(demo,:));
% xlabel('Time t');
% ylabel('velocity_error_x');
% grid on;
% 
% figure
% plot(0:0.002:1.996, f2(demo,:));
% hold on;
% plot(0:0.002:1.996, results_y(demo,:));
% xlabel('Time t');
% ylabel('velocity_error_y');
% grid on;

%% 
% load('New_demos_S.mat')
%load('D:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code/LASA_track/Sshape.mat')
% load('D:\Program
% Files\Polyspace\R2021a\bin\tracking\Prof_Fei_Gao_Control\BLS_learning_code\LASA_V2\Zshape.mat')
load('D:\Program Files\Polyspace\R2021a\bin\tracking\LNN_learning_code\LASA_SMC\ZShape.mat')
%t_step = 0.002;
t_step = New_demos{1,demo}.dt;

x = zeros(1,simTime);
y = zeros(1,simTime);

h_x = zeros(num_node, simTime);
h_y = zeros(num_node, simTime);

control_law_x = zeros(1,simTime);
control_law_y = zeros(1,simTime);


ref_x = New_demos{1,demo}.ref_position_x(:)';
ref_y = New_demos{1,demo}.ref_position_y(:)';
SMC_x = New_demos{1,demo}.position_x(:)';
SMC_y = New_demos{1,demo}.position_y(:)';


ref_vel_x = New_demos{1,demo}.ref_velocity_x(:)';
ref_vel_y = New_demos{1,demo}.ref_velocity_y(:)';


x0 = New_demos{1,demo}.position_x(1);
y0 = New_demos{1,demo}.position_y(1);

x(1) = x0;
y(1) = y0;


h_x(:,1) = beta_x(4,:)'* (x(1)-ref_x(1)) + beta_x(end,:)';
h_y(:,1) = beta_y(4,:)'* (y(1)-ref_y(1)) + beta_y(end,:)';

%% 
for j = 1:simTime
   x_error = x(j) - ref_x(j);
   y_error = y(j) - ref_y(j);
   [h_x(:,j+1), control_law_x] = cal_control_xonly(beta_x, tau_x, As_x, x_error,num_node, h_x(:,j), t_step);
   [h_y(:,j+1), control_law_y] = cal_control_yonly(beta_y, tau_y, As_y, y_error,num_node, h_y(:,j), t_step);
   x(j + 1) = x(j) +  (control_law_x + ref_vel_x(j)) * t_step;
   y(j + 1) = y(j) + (control_law_y + ref_vel_y(j)) * t_step;
end
figure;
plot(ref_x, ref_y,'k','LineWidth',1.5);
hold on;
plot(ref_x(1),ref_y(1),'k*')
plot(x,y,'Color',[0,0.4470,0.7410],'LineWidth',1.5);
hold on;
plot(x(1),y(1),'*','Color',[0,0.4470,0.7410]);

grid on
legend('Reference Trajectory','Reference Start Point','Actural Trajectory','Actural Start Point');
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');

% hold on;
% plot(learn_x, learn_y)
% figure;
% plot(1:simTime, h(1,1:999))
% figure;
% plot(1:simTime, control_law_x);


