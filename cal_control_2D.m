function [h, control_law] = cal_control_2D(beta, dimension, error, num_node, h_value,t_step)
    
    w = reshape(beta(1:num_node*dimension), dimension, []);
    rau = -0.9 * w;
    As = beta(num_node*(2*dimension+1)+1:num_node*(dimension*2+2));
    tau = beta(num_node*(2*dimension+2)+1:num_node*(dimension*2+3));
    theta = beta(num_node*(2*dimension+3)+1:end);
    u = zeros(dimension,1);

    for k = 1:num_node
        % choose an ode solution
        [~, h_value(k)] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tstep, h_value(k));
        %[t,y] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
        %[t,y] = ode23(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));

        h_value(k) = h_value(k) + ((-1/tau(k) + w(:,k)' * tanh(theta' .* error)) * h_value(k) + As(k) * w(:,k)' * tanh(theta' .* error))* t_step;
        
        u = u + rau(:,k) * h_value(k);
            
    end
%     disp(u)
    h = h_value;
    control_law = u;
end