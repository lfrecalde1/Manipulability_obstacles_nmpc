function [m] = manipulability_manipulador(h, L)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
J = jacobian_manipulador(h, L);
m = sqrt(det(J*J'));
m = m/0.0534;
a_m = 0.1;
aux_m = exp(-(m)^2/a_m);
end