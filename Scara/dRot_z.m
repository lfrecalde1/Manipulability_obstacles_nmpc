function [dR] = dRot_z(x, flip)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Aux Variable
Rhz = [[0, -1, 0, 0];
       [1, 0, 0, 0];
       [0, 0, 0, 0];
       [0, 0, 0, 0]];

R = [cos(x), -sin(x), 0, 0;...
     sin(x), cos(x), 0, 0;...
     0, 0, 1, 0;...
     0, 0, 0, 1];
 
if flip
    dR = Rhz*R';
else
    dR = Rhz*R;
end

end

