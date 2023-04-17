function [V_sum] = potential_field(h, obs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
i = size(obs, 1);
j = size(obs, 2);
number = size(obs, 3);
V = [];
for k = 1:1:number
    aux = exp(distance(h,obs(:,1,k)));
    V = [V;aux];
end
V_sum = sum(V);
end

