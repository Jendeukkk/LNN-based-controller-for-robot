

clc;
clear all;

simTime = 999;
Number = 7; 
dimension = 2; 

%xy方向误差
position_error = zeros(Number,simTime,dimension);
velocity_error = zeros(Number,simTime,dimension);
% velocity = zeros(Number,simTime,dimension);
% ref_velocity = zeros(Number,simTime,dimension);
% position = zeros(Number,simTime,dimension);
% ref_position = zeros(Number,simTime,dimension);
%% data
load('demos.mat'); 
for j = 1:Number % demos group

for t = 1:simTime % data point in each demo group
position_error(j,t,1) = BLS_demos{1, j}.position_error_x(t);
position_error(j,t,2) = BLS_demos{1, j}.position_error_y(t);

velocity_error(j,t,1) = BLS_demos{1, j}.cal_x(t);
velocity_error(j,t,2) = BLS_demos{1, j}.cal_y(t);

end

end



results = velocity_error;
% figure;
%plot([1:simTime], ref_velocity(1,:,1));
% hold on;
% plot([1:simTime], velocity(1,:,1));
% hold on;
% plot([1:simTime], results(1,:,1));
% xlabel('Time t');
% ylabel('velocity_error');
% plot([1:simTime], position(1,:,2));
% hold on;
% plot([1:simTime], ref_position(1,:,2));
% grid on;

%% 

num_node = 30; 

beta0 = randn(1,num_node*(2*dimension+3) + dimension);
A = []; 
b = [];
Aeq = []; 
beq = [];
vlb = -Inf*ones(1,num_node*(2*dimension+3) + dimension); 
vlb(num_node*(2*dimension+1)+1:end) = zeros(1, 2*num_node+dimension);
vlb(num_node*dimension+1:num_node*(2*dimension+1)) = -10*ones(1,num_node*(dimension+1));
vlb(num_node*(2*dimension+2)+1:num_node*(2*dimension+3)) = 0.04*ones(1, num_node);
vub = Inf*ones(1,num_node*(2*dimension+3) + dimension);
vub(num_node*(2*dimension+1)+1:num_node*(2*dimension+2)) = 30 * ones(1, num_node);
vub(num_node*dimension+1:num_node*(2*dimension+1)) = 10*ones(1,num_node*(dimension+1));
draw = 0;


%fun_LNN_2(beta0, Number, simTime, dimension, tau, As, position_error, num_node, results, draw)
options = optimoptions('fmincon','Algorithm', 'interior-point','MaxIter', 2000, 'MaxFunctionEvaluations', 5e+05, 'display', 'iter-detailed');
[beta, fval] = fmincon(@(beta) fun_LNN_2D(beta, Number, simTime, dimension, position_error, num_node, results, draw), beta0, A, b, Aeq, beq, vlb, vub,  @(beta) mycon_LNN_2D(beta, num_node, dimension), options);


save('LNN_2D.mat')


