function [LNN_x, LNN_y, LNN_vx, LNN_vy] = LNN_comparison_2D(ref_x, ref_y, ref_vx, ref_vy, x0, y0, dt,demo)
%% 


if demo == 7
% load('LNN_2D_1201.mat','beta')
load('LNN_2D_1125.mat','beta')
elseif demo ==3
load('LNN_2D_1202_3demo.mat','beta')        
elseif demo ==5
load('LNN_2D_1202_5demo.mat','beta')  
elseif demo ==9
load('LNN_2D_1202_9demo.mat','beta')  
end       
     
num_node = 10;
simTime = 999;
dimension = 2;
%% 

LNN_x = zeros(1,simTime);
LNN_y = zeros(1,simTime);
LNN_vx = zeros(1,simTime);
LNN_vy = zeros(1,simTime);

h = zeros(num_node, simTime);

LNN_x(1) = x0;
LNN_y(1) = y0;



%% 
for j = 1:simTime
   xy_error = [LNN_x(j) - ref_x(j), LNN_y(j) - ref_y(j)]';
   [h(:,j+1), control_law] = cal_control_2D(beta, dimension, xy_error,num_node, h(:,j),dt);
   LNN_x(j + 1) = LNN_x(j) +  (control_law(1) + ref_vx(j)) * dt;
   LNN_y(j + 1) = LNN_y(j) + (control_law(2) + ref_vy(j)) * dt;
   LNN_vx(j) =  control_law(1) + ref_vx(j);
   LNN_vy(j) =  control_law(2) + ref_vy(j);
end