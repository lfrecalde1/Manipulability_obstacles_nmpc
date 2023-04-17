function [J] = jacobian_general(h, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%% SPLIT VECTOR VALUES
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

J11 = cos(psi);
J12 = -sin(psi);
J13 = 0;
J14 = 0;
J15 = 0;
J16 = 0;
J17 = 0;

J21 = sin(psi);
J22 = cos(psi);
J23 = 0;
J24 = 0;
J25 = 0;
J26 = 0;
J27 = 0;


J31 = 0;
J32 = 0;
J33 = 1;
J34 = 0;
J35 = 0;
J36 = 0;
J37 = 0;

J41 = 0;
J42 = 0;
J43 = 0;
J44 = 1;
J45 = 0;
J46 = 0;
J47 = 0;

J51 = 0;
J52 = 0;
J53 = 0;
J54 = 0;
J55 = 1;
J56 = 0;
J57 = 0;

J61 = 0;
J62 = 0;
J63 = 0;
J64 = 0;
J65 = 0;
J66 = 1;
J67 = 0;

J71 = 0;
J72 = 0;
J73 = 0;
J74 = 0;
J75 = 0;
J76 = 0;
J77 = 1;

J = [J11, J12, J13, J14, J15, J16, J17;...
     J21, J22, J23, J24, J25, J26, J27;...
     J31, J32, J33, J34, J35, J36, J37;...
     J41, J42, J43, J44, J45, J46, J47;...
     J51, J52, J53, J54, J55, J56, J57;...
     J61, J62, J63, J64, J65, J66, J67;...
     J71, J72, J73, J74, J75, J76, J77];

end

