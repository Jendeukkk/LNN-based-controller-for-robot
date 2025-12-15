function [c,ceq] = mycon_LNN_2D(beta,num_node,dimension)
%     lambda = beta(4,:);
%     kk = beta(end,:);
%     h = lambda' * position_error + kk'
    w = reshape(beta(1:num_node*dimension), dimension, []);
    tau = beta(num_node*(2*dimension+2)+1:num_node*(dimension*2+3));
    
    cw = sum(abs(w), 1) - 1./tau;
    c = [cw];
%     c = [cw];
    ceq =[];