function xp = f_model_kinematics(x, u, L)


% System matrices Kinematics
J = jacobian_general(x, L); 

xp = J*u;
end