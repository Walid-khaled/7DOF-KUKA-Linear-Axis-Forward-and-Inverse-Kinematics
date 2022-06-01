function [J, J1, J2, J3, J4, J5, J6, J7] =  Jacobian(q, link_lengths)
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

%% Getting the forward kinematics
T =  FK(q, link_lengths);
T(1:3, 4) = 0;

%% Getting the jacobians
Td = Tyd(q0)*Tz(d0)*Rz(q1)*Tx(d1)*Ry(q2)*Tx(d2)*Ry(q3)*Tx(d3)*Tz(d4)*Rx(q4)*Ry(q5)*Rx(q6)*Tx(d6) / T;
 
J1 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Ty(q0)*Tz(d0)*Rzd(q1)*Tx(d1)*Ry(q2)*Tx(d2)*Ry(q3)*Tx(d3)*Tz(d4)*Rx(q4)*Ry(q5)*Rx(q6)*Tx(d6) / T;
 
J2 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Ty(q0)*Tz(d0)*Rz(q1)*Tx(d1)*Ryd(q2)*Tx(d2)*Ry(q3)*Tx(d3)*Tz(d4)*Rx(q4)*Ry(q5)*Rx(q6)*Tx(d6) / T;
 
J3 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Ty(q0)*Tz(d0)*Rz(q1)*Tx(d1)*Ry(q2)*Tx(d2)*Ryd(q3)*Tx(d3)*Tz(d4)*Rx(q4)*Ry(q5)*Rx(q6)*Tx(d6) / T;
 
J4 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Ty(q0)*Tz(d0)*Rz(q1)*Tx(d1)*Ry(q2)*Tx(d2)*Ry(q3)*Tx(d3)*Tz(d4)*Rxd(q4)*Ry(q5)*Rx(q6)*Tx(d6) / T;
 
J5 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Ty(q0)*Tz(d0)*Rz(q1)*Tx(d1)*Ry(q2)*Tx(d2)*Ry(q3)*Tx(d3)*Tz(d4)*Rx(q4)*Ryd(q5)*Rx(q6)*Tx(d6) / T;
 
J6 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

Td = Ty(q0)*Tz(d0)*Rz(q1)*Tx(d1)*Ry(q2)*Tx(d2)*Ry(q3)*Tx(d3)*Tz(d4)*Rx(q4)*Ry(q5)*Rxd(q6)*Tx(d6) / T;
 
J7 = [ Td(1,4); Td(2,4); Td(3,4); Td(3,2); Td(1,3); Td(2,1)];

J = [J1, J2, J3, J4, J5, J6, J7];
end

