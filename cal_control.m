function [h, control_law] = cal_control(beta, dimension, tau, As, xy_error, num_node, h_value)
    t_step = 0.002;
    w = beta(1:dimension,:);
    rau = beta(dimension+1:2*dimension,:);
    u = [0;0];
    theta = beta(end-1:end,:);
    disp('error');
    disp(xy_error);
    for k = 1:num_node    
        % choose an ode solution
        [~, h_value(k)] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tstep, h_value(k));
        %[~, h(k)] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
        %[t,y] = ode45(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));
        %[t,y] = ode23(@(t,h) LNN_ODE(h, tau, w(:,k), theta, As, error), tspan, h(k));

        h_value(k) = h_value(k) + ((-1/tau + w(:,k)' * tanh(theta(:,k) .* xy_error)) * h_value(k) + As * w(:,k)' * tanh(theta(:,k) .* xy_error))* t_step;
        u = u + rau(:,k) * h_value(k);
            
    end
%     disp(u)
    h = h_value;
    control_law = u;
end