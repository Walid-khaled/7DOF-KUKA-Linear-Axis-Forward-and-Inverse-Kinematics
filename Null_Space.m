function [q, q_dot] = Null_Space(q_0, link_lengths, p_global, flag)
%% Joint ranges
qmin_range = [0 deg2rad(-120) deg2rad(-170)  deg2rad(-120) deg2rad(-170) deg2rad(-120) deg2rad(-175)];
qmax_range = [1.5 deg2rad(120) deg2rad(170) deg2rad(120) deg2rad(170) deg2rad(120) deg2rad(175)];
q_mean = (qmin_range + qmax_range)./2;

N = length(q_0);
H_range = sum(((q_0 - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);


%% Extract the angles from the vector
q1 = q_0(1);
q2 = q_0(2);
q3 = q_0(3);
q4 = q_0(4);
q5 = q_0(5);
q6 = q_0(6);
q7 = q_0(7);

deltaT = 1;

deltaq = 1e-02;

k = 1000;

k_0 = -1;
%% Calculating the jacobian
J = Jacobian(q_0, link_lengths);

w = sqrt(det(J*J'));
%% Calculate the new jacobians
new_q = [ q1+deltaq, q2, q3, q4, q5, q6, q7];

J1 = Jacobian(new_q, link_lengths);

w1 = sqrt(det(J1*J1'));

H_range1 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

new_q = [ q1, q2+deltaq, q3, q4, q5, q6, q7];

J2 = Jacobian(new_q, link_lengths);

w2 = sqrt(abs(det(J2*J2')));

H_range2 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

new_q = [ q1, q2, q3+deltaq, q4, q5, q6, q7];

J3 = Jacobian(new_q, link_lengths);

w3 = sqrt(det(J3*J3'));

H_range3 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

new_q = [ q1, q2, q3, q4+deltaq, q5, q6, q7];

J4 = Jacobian(new_q, link_lengths);

w4 = sqrt(det(J4*J4'));

H_range4 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

new_q = [ q1, q2, q3, q4, q5+deltaq, q6, q7];

J5 = Jacobian(new_q, link_lengths);

w5 = sqrt(det(J5*J5'));

H_range5 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

new_q = [ q1, q2, q3, q4, q5, q6+deltaq, q7];

J6 = Jacobian(new_q, link_lengths);

w6 = sqrt(det(J6*J6'));

H_range6 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

new_q = [ q1, q2, q3, q4, q5, q6, q7+deltaq];

J7 = Jacobian(new_q, link_lengths);

w7 = sqrt(det(J7*J7'));

H_range7 = sum(((new_q - q_mean)./(qmax_range - qmin_range)).^2)/(2*N);

%% Get the q_0_dot vector

if flag == 1
    q_0_dot_1 = -k_0 * (w1 - w)/deltaq;

    q_0_dot_2 = -k_0 * (w2 - w)/deltaq;

    q_0_dot_3 = -k_0 * (w3 - w)/deltaq;

    q_0_dot_4 = -k_0 * (w4 - w)/deltaq;
    
    q_0_dot_5 = -k_0 * (w5 - w)/deltaq;
    
    q_0_dot_6 = -k_0 * (w6 - w)/deltaq;
    
    q_0_dot_7 = -k_0 * (w7 - w)/deltaq;
else
    q_0_dot_1 = k_0 * (H_range1 - H_range)/deltaq;

    q_0_dot_2 = k_0 * (H_range2 - H_range)/deltaq;

    q_0_dot_3 = k_0 * (H_range3 - H_range)/deltaq;

    q_0_dot_4 = k_0 * (H_range4 - H_range)/deltaq;
    
    q_0_dot_5 = k_0 * (H_range5 - H_range)/deltaq;
    
    q_0_dot_6 = k_0 * (H_range6 - H_range)/deltaq;
    
    q_0_dot_7 = k_0 * (H_range7 - H_range)/deltaq;
end

q_0_dot = [q_0_dot_1; q_0_dot_2; q_0_dot_3; q_0_dot_4; q_0_dot_5; q_0_dot_6; q_0_dot_7];
%% Calculating the forward kinematics
[~, ~, ~, ~, ~, ~, ~, cur_pos] =  FK(q_0, link_lengths);

%% Getting the r vector
r = p_global - cur_pos;

%% Numerical differentiation
r_dot = r./k; % To decrease the step that is taken by the velocity, k is some large constant

%% Calculate the J_inverse
J_hash = pinv(J);
I = eye(7);
%% Caculating the q_dot
q_dot = J_hash * r_dot + (I - J_hash*J)*q_0_dot;

q = q_0+ (q_dot .* deltaT)';
end

