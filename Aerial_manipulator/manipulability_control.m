function [m] = manipulability_control(h, L)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
J = jacobian_control(h, L);
m = sqrt(det(J*J'));
a_m = 0.1;
aux_m = exp(-(m)^2/a_m);

end

