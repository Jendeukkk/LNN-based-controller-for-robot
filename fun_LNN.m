function f = fun_LNN(beta, Number, simTime, dimension, tau, As, position_error,num_node, results,draw)
    t_step = 0.002;
    w = beta(1:dimension,:);
    rau = beta(dimension+1:2*dimension,:); 
    theta = beta(end-1:end,:);
    cal_val = zeros(Number,simTime,dimension);
%     disp(['origin',num2str(h0)]);
    for j = 1:Number
        h = beta(end-1,:);
%         disp([j,h]);
        for i = 1:simTime
            tspan = [0,t_step];
            u = zeros(dimension,1);
            for k = 1:num_node    
                % 计算每个隐含层状态
                error = [position_error(j,i,1),position_error(j,i,2)]';
                %[~, h(k)] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
                %[t,y] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
                %[t,y] = ode23(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
                h(k) = h(k) + ((-1/tau + w(:,k)' * tanh(theta(:,k) .* error)) * h(k) + As * w(:,k)' * tanh(theta(:,k) .* error))* t_step;
         
%                 if draw == 1 && k == 5 && j == 1
%                     h(k);
%                     plot((i-1) * t_step, h(k), '.');
%                     hold on
%                     plot((i-1) * t_step, As * w(:,k)' * tanh(theta * error), '*');
%                     xlabel('Time t');
%                     ylabel('Solution h');
%                     title('Solution of the ODE ');
%                     grid on;
%                     hold on;
%                 end
           
%                 if k == 1
%                     subplot(1,2,1)
%                     plot(t + (i-1) * t_step, y);
%                     xlabel('Time t');
%                     ylabel('Solution h');
%                     title('Solution of the ODE ');
%                     grid on;
%                     hold on;
%                 end
%                 if k == 2
%                     subplot(1,2,2)
%                     plot(t + (i-1) * t_step, y);
%                     xlabel('Time t');
%                     ylabel('Solution h');
%                     title('Solution of the ODE ');
%                     grid on;
%                     hold on;
%                 end
%                 h(k) = y(end);

                u = u + rau(:,k) * h(k);
                
            end
 
            for d = 1:dimension
                cal_val(j,i,d) = u(d);
            end
        end
    end
    diffsquare = sqrt(sum((cal_val - results) .^2, 3));
%     diffsquare = (cal_val - results);
    %f = sum(diffsquare(:));
    f = cal_val;
end
