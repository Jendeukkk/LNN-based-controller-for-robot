
clc;
clear;
load('LNN_2D_0321_09.mat')
w = reshape(beta(1:num_node*dimension), dimension, []);
rau = -0.9*w;
lambda = reshape(beta(num_node*dimension+1:num_node*dimension*2),dimension,[]);
kk = beta(num_node*(2*dimension)+1:num_node*(dimension*2+1));
As = beta(num_node*(2*dimension+1)+1:num_node*(dimension*2+2));
tau = beta(num_node*(2*dimension+2)+1:num_node*(dimension*2+3));
theta = beta(num_node*(2*dimension+3)+1:end);

draw = 1;
f = fun_LNN_2D(beta, Number, simTime, dimension, position_error,num_node, results, draw);  



% figure()
% plot([1:simTime], f(1,:,1),'color','r');
% hold on;
% plot([1:simTime], results(1,:,1),'color','b');
% xlabel('Time t');
% ylabel('velocity_error');
% grid on;
% hold on;
% 

% figure()
% plot([1:simTime], f(1,:,2),'color','r');
% hold on;
% plot([1:simTime], results(1,:,2),'color','b');
% xlabel('Time t');
% ylabel('velocity_error');
% grid on;
% hold on;


%% 
load('ZShape.mat')
demo = 2;
t_step = New_demos{1,demo}.dt;

x = zeros(1,simTime);
y = zeros(1,simTime);

h = zeros(num_node, simTime);

control_law_x = zeros(1,simTime);
control_law_y = zeros(1,simTime);

ref_x = New_demos{1,demo}.ref_position_x(:)';
ref_y = New_demos{1,demo}.ref_position_y(:)';

SMC_x = New_demos{1,demo}.position_x(:)';
SMC_y = New_demos{1,demo}.position_y(:)';


ref_vel_x = New_demos{1,demo}.ref_velocity_x(:)';
ref_vel_y = New_demos{1,demo}.ref_velocity_y(:)';


x0 = -30;
y0 = 30;

x(1) = x0;
y(1) = y0;

xy_error_0 = [x(1) - ref_x(1), y(1) - ref_y(1)]';
h(:,1) = lambda' * xy_error_0 + kk';
%% 
for j = 1:simTime
   xy_error = [x(j) - ref_x(j), y(j) - ref_y(j)]';
   
   [h(:,j+1), control_law] = cal_control_2D(beta, dimension, xy_error,num_node, h(:,j),t_step);
   x(j + 1) = x(j) +  (control_law(1) + ref_vel_x(j)) * t_step;
   y(j + 1) = y(j) + (control_law(2) + ref_vel_y(j)) * t_step;
   
   control_law_x(j) = control_law(1);
   control_law_y(j) = control_law(2);
end
figure;
plot(ref_x, ref_y);
hold on;
plot(x,y);

figure;
plot(ref_y);
hold on;
plot(y);
% figure;
% plot(1:simTime, h(1,1:999))
% figure;
% plot(1:simTime, control_law_x);


