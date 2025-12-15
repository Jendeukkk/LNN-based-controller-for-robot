function f = fun_LNN_2D(beta, Number, simTime, dimension, position_error,num_node, results,draw)
    t_step = 0.0046;
    w = reshape(beta(1:num_node*dimension), dimension, []);
    rau = -1.1*w;
    lambda = reshape(beta(num_node*dimension+1:num_node*dimension*2),dimension,[]);
    kk = beta(num_node*(2*dimension)+1:num_node*(dimension*2+1));
    As = beta(num_node*(2*dimension+1)+1:num_node*(dimension*2+2));
    tau = beta(num_node*(2*dimension+2)+1:num_node*(dimension*2+3));
    theta = beta(num_node*(2*dimension+3)+1:end);
    cal_val = zeros(Number,simTime,dimension);

    for j = 1:Number
        % position_error:7*999*2
        h = lambda' * squeeze(position_error(j,1,:)) + kk';

        for i = 1:simTime
            tspan = [0,t_step];
            u = zeros(dimension,1);
            for k = 1:num_node    
                % 计算每个隐含层状态
                error = [position_error(j,i,1),position_error(j,i,2)]';
                %[~, h(k)] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
                %[t,y] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
                %[t,y] = ode23(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
                h(k) = h(k) + ((-1/tau(k) + w(:,k)' * tanh(theta' .* error)) * h(k) + As(k) * w(:,k)' * tanh(theta' .* error))* t_step;
                %用自己写的欧拉法画图
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
                  %ode画图
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
    f = sum(diffsquare(:));
%     f = cal_val;
end
