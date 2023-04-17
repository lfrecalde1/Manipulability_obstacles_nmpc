function [J] = jacobian_manipulador(h, L)
%% GET INTERNAL VALUES
x_0 = h(1);
y_0 = h(2);
z_0 = h(3);
psi = h(4);

q_1 = h(5);
q_2 = h(6);
q_3 = h(7);

l_1 = L(1);
l_2 = L(2);
l_3 = L(3);

%% GET JACOBIAN MATRIX


%% CREATE MATRIX
 
J = [[ -sin(q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2)), -cos(q_1)*(l_3*sin(q_2 + q_3) + l_2*sin(q_2)), -l_3*sin(q_2 + q_3)*cos(q_1)];...
     [  cos(q_1)*(l_3*cos(q_2 + q_3) + l_2*cos(q_2)), -sin(q_1)*(l_3*sin(q_2 + q_3) + l_2*sin(q_2)), -l_3*sin(q_2 + q_3)*sin(q_1)];...
     [                                             0,           - l_3*cos(q_2 + q_3) - l_2*cos(q_2),          -l_3*cos(q_2 + q_3)]];
 
end
