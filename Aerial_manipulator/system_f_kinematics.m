function [x] = system_f_kinematics(x, u, L, ts)
% Sample Time
T_s = ts;

% General States System
k1 = f_model_kinematics(x, u, L);
k2 = f_model_kinematics(x + T_s/2*k1, u, L);
k3 = f_model_kinematics(x + T_s/2*k2, u, L);
k4 = f_model_kinematics(x + T_s*k3, u, L);
x = x +T_s/6*(k1 +2*k2 +2*k3 +k4);


end