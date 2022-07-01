function [T, T1, T2, T3, T4, T5, T6, Pos] =  FK(q, link_lengths) 
%% Extracting the link lengths
d0 = link_lengths(1);
d1 = link_lengths(2);
d2 = link_lengths(3);
d3 = link_lengths(4);
d4 = link_lengths(5);
d6 = link_lengths(6);

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
q4 = q(5);
q5 = q(6);
q6 = q(7);

%% Transformations
T1 = Ty(q0);

T2 = T1 * Tz(d0);

T3 = T2 * Rz(q1) * Tx(d1);

T4 = T3 * Ry(q2) * Tx(d2);

T5 = T4 * Ry(q3) * Tx(d3) * Tz(d4);

T6 = T5 * Rx(q4)* Ry(q5) * Rx(q6);

T = T6 * Tx(d6);

phi_x = atan2(T(3,1),T(3,2));
phi_z = atan2(T(1,3),-T(2,3));
phi_y = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));

Pos = [T(1:3,4);phi_x;phi_y;phi_z];
end
