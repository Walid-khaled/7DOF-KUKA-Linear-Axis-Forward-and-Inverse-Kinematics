function [q, q_dot] = PseudoInverse(q_0, link_lengths, p_global, flag)
deltaT = 1;
k = 1000;

weights = [1, 1, 7500, 1, 1, 1, 1];
W = diag(weights);
%% Calculating the jacobian
J = Jacobian(q_0, link_lengths);

%% Calculating the forward kinematics
[~, ~, ~, ~, ~, ~, ~, cur_pos] =  FK(q_0, link_lengths);

%% Getting the r vector
r = p_global - cur_pos;

%% Numerical differentiation
r_dot = r./k; % To decrease the step that is taken by the velocity, k is some large constant

if flag == 1
    J_inv_ps = pinv(J);
else %Weighted pseudo inverse
   J_inv_ps =  W\J'/(J/W*J');
end
%% Caculating the q_dot
q_dot = J_inv_ps * r_dot;

q = q_0+ (q_dot .* deltaT)';
end

