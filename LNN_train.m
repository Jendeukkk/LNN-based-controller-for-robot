clc;
clear all;

simTime = 999;
Number = 7; 



dimension = 2; 

n = 1; 

position_error = zeros(Number,simTime,dimension);
velocity_error = zeros(Number,simTime,dimension);
% velocity = zeros(Number,simTime,dimension);
% ref_velocity = zeros(Number,simTime,dimension);
% position = zeros(Number,simTime,dimension);
% ref_position = zeros(Number,simTime,dimension);
%% data
% Once demontion, X-axis

load('New_demos_S.mat'); 

for j = 1:Number % demos group

for t = 1:simTime % data point in each demo group
position_error(j,t,1) = New_demos{1, j}.position_x(t) - New_demos{1, j}.ref_position_x(t);
position_error(j,t,2) = New_demos{1, j}.position_y(t) - New_demos{1, j}.ref_position_y(t);  
% position(j,t,2) = New_demos{1, j}.position_y(t);
% ref_position(j,t,2) = New_demos{1, j}.ref_position_y(t);
velocity_error(j,t,1) = New_demos{1, j}.velocity_x(t) - New_demos{1, j}.ref_velocity_x(t); 
velocity_error(j,t,2) = New_demos{1, j}.velocity_y(t) - New_demos{1, j}.ref_velocity_y(t); 
% velocity(j,t,1) = New_demos{1, j}.velocity_x(t);
% ref_velocity(j,t,1) = New_demos{1, j}.ref_velocity_x(t);
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
% h = 2 * rand(1,num_node);
% b = zeros(dimension,1); 
% theta0 = ones(1,num_node); 
% rau = zeros(demension,1);
% w = rand(dimension,1);
beta0 = randn(2 * dimension + 3,num_node); %beta0 = [w;rau;h0;theta0]
nvars = (2 * dimension + 1) * num_node;

tau = 0.3;
As = 4;
A = []; 
b = [];
Aeq = []; 
beq = [];
vlb = -Inf * ones(2 * dimension + 3,num_node); 
vub = Inf * ones(2 * dimension + 3,num_node);
draw = 0;

% options = optimoptions('fmincon', 'Algorithm', 'active-set','MaxIter', 2000, 'MaxFunctionEvaluations', 5e+05,  'display', 'iter-detailed');
% [beta, fval] = fmincon(@(beta) fun_LNN(beta, Number, simTime, dimension, tau, As, position_error, num_node, results, draw), beta0, A, b, Aeq, beq, vlb, vub, [], options);

% adam
% sOpt = optimset('fmin_adam');
% sOpt.GradObj = 'off';
% sOpt.MaxFunEvals = 1e4;
% phiHat = fmin_adam(@(beta) fun_LNN(beta, Number, simTime, dimension, tau, As, position_error,num_node, results, draw), beta0, 0.1, [], [], [], [], sOpt)

%SQP
options = optimoptions('fmincon', 'Algorithm', 'SQP','MaxIter', 2000, 'MaxFunctionEvaluations', 5e+05,  'display', 'iter-detailed');
[beta, fval] = fmincon(@(beta) fun_LNN(beta, Number, simTime, dimension, tau, As, position_error, num_node, results, draw), beta0, A, b, Aeq, beq, vlb, vub, [], options);

%
% options = optimset('Algorithm', 'interior-point','MaxIter', 1000, 'display', 'iter-detailed');
% [beta, fval] = fmincon(@(beta) fun_LNN(beta, Number, simTime, dimension, h, tau, theta, As, position_error,num_node, results), beta0, A, b, Aeq, beq, vlb, vub, [], options);

%
% options = optimoptions('simulannealbnd','PlotFcns', {@saplotbestx,@saplotbestf,@saplotx,@saplotf}, 'MaxIter', 10000, 'Display', 'iter','TemperatureFcn', @customTempSchedule);
% [beta, fval] = simulannealbnd(@(beta) fun_LNN(beta, Number, simTime, dimension, h, tau, theta, As, position_error,num_node, results), beta0, vlb, vub, options);

%ga
% options = optimoptions('ga', 'Display', 'iter');
% [beta, fval] = ga(@(beta) fun_LNN(beta, Number, simTime, dimension, h, tau, theta, As, position_error,num_node, results), nvars, A, b, Aeq, beq, vlb, vub, [], options);
save LNN_test_3
disp(beta);
% draw = 1;
% f = fun_LNN(beta, Number, simTime, dimension, tau, theta, As, position_error,num_node, results, draw);
