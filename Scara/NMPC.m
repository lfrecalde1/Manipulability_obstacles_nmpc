function [H0, control] = NMPC(h, V, hd, k, H0, vc, args, solver ,N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
V_sum = sum(V);
H = [h(1:5);V_sum];
args.p(1:6) = H;

for i = 1:N
    args.p(6*i+1:6*i+6)=[hd(:,k+i);0;0;0;0];
end

args.x0 = [reshape(H0',6*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(6*(N+1)+1:end))',2,N)';
H0 = reshape(full(sol.x(1:6*(N+1)))',6,N+1)';
end
