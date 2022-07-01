function [q, q_dot] = TaskAugmentation(q_0, link_lengths, p_global)
deltaT = 1;
k = 100;
u = sqrt(0.001);
%% Calculating the jacobian
J = Jacobian(q_0, link_lengths);

J1 = J(1:3, :);

J2 = J(4:6, :);

%% Calculating the forward kinematics
[~, ~, ~, ~, ~, ~, ~, cur_pos] =  FK(q_0, link_lengths);

%% Getting the r vector
r1 = p_global(1:3) - cur_pos(1:3);

r2 = p_global(4:6) - cur_pos(4:6);
%% Numerical differentiation
r_dot1 = r1./k; % To decrease the step that is taken by the velocity, k is some large constant

r_dot2 = r2./k;
%% Calculate the J_inverse
J1_hash = (J1'/(J1*J1' + u^2*eye(3)));

I = eye(7);

P1 = (I - J1_hash*J1);

J2_hash = ((J2 * P1)'/((J2 * P1)*(J2 * P1)' + u^2*eye(3)));

v1 = J2_hash * (r_dot2 - J2*J1_hash*r_dot1);

%% Caculating the q_dot
q_dot = (J1_hash * r_dot1) + (P1 * v1);

q = q_0+ (q_dot .* deltaT)';


end

