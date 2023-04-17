function [u] = control_system(hd, hdp, qd, H, h, L, k1, k2, k3)
%% JACOBIAN MATRIX
[J] = jacobian_control(h, L);

%% CONTROL GAINS
K1 = k1*eye(length(hd));
K2 = k2*eye(length(hd));
K3 = k3*eye(length(h));

%% ITERNAL STATES
q1 = h(5);
q2 = h(6);
q3 = h(7);

%% DESIRED INTERNAL STATES
q1_d = qd(1);
q2_d = qd(2);
q3_d = qd(3);

%% CONTROL ERROR
H = H(1:3);
he = hd - H;



%% Auxuliar 
I = eye(7,7);
nul = [0;...
       0;...
       0;...
       0;...
       q1_d-q1;...
       q2_d-q2;...
       q3_d-q3];
%% CONTROL LAW
u = pinv(J)*(hdp+K2*tanh(inv(K2)*K1*he)) + (I-pinv(J)*J)*K3*tanh(nul);

end

