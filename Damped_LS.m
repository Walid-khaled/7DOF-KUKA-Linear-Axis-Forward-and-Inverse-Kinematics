function [q, q_dot] = Damped_LS(q_0, link_lengths, p_global)
deltaT = 1;
k = 1000;

u = sqrt(0.001);
%% Calculating the jacobian
J = Jacobian(q_0, link_lengths);


%% Calculating the forward kinematics
[~, ~, ~, ~, ~, ~, ~, cur_pos] =  FK(q_0, link_lengths);

%% Getting the r vector
r = p_global - cur_pos;

%% Numerical differentiation
r_dot = r./k; % To decrease the step that is taken by the velocity, k is some large constant

%% Calculate the J_inverse
J_DLS= J'/(J*J' + u^2*eye(6));

%% Calculating the q_dot
q_dot = J_DLS * r_dot;

%% Integration
q = q_0+ (q_dot .* deltaT)';
end

