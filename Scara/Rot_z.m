function [R] = Rot_z(x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
R = [cos(x), -sin(x), 0, 0;...
     sin(x), cos(x), 0, 0;...
     0, 0, 1, 0;...
     0, 0, 0, 1];
 
end

