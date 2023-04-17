function [H0, control] = NMPC(h, H, obs, hd, k, H0, vc, args, solver ,N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
i = size(obs, 1);
j = size(obs, 2);
number = size(obs, 3);
V = [];
for ii = 1:1:number
    aux = obs(:,1,ii);
    V = [V;aux];
end
V_f = reshape(V,1,3*number);
args.p(1:10) = [H(1:3);h(4:7);h(1:3)];

for i = 1:N
    args.p(10*i+1:10*i+10)=[hd(:,k+i);0;0;0;0;0;0;0];
end
args.p((10)+N*(10)+1:(10)+N*(10)+3*number) = V_f';

args.x0 = [reshape(H0',10*(N+1),1);reshape(vc',size(vc,2)*N,1)]; % initial value of the optimization variables

sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(10*(N+1)+1:end))',7,N)';
H0 = reshape(full(sol.x(1:10*(N+1)))',10,N+1)';
end
