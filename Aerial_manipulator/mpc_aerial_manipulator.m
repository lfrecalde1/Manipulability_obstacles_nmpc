function [f,solver,args] = mpc_aerial_manipulator(bounded, N, L, ts, obs)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% Definicion de las restricciones en las acciones de control
ul_max = bounded(1); 
ul_min = bounded(2);

um_max = bounded(3);
um_min = bounded(4);

un_max = bounded(5);
un_min = bounded(6);

w_max = bounded(7); 
w_min = bounded(8);

q_1_max = bounded(9); 
q_1_min = bounded(10);

q_2_max = bounded(11);
q_2_min = bounded(12);

q_3_max = bounded(13);
q_3_min = bounded(14);


%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
z = SX.sym('z');
psi = SX.sym('psi');
q1 = SX.sym('q1');
q2 = SX.sym('q2');
q3 = SX.sym('q3');
x_p = SX.sym('x_p');
y_p = SX.sym('y_p');
z_p = SX.sym('xz_p');



%% Definicion de cuantos estados en el sistema
states = [x;y;z;psi;q1;q2;q3;x_p;y_p;z_p];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
ul = SX.sym('ul');
um = SX.sym('um');
un = SX.sym('un');
w = SX.sym('w');
q1_p = SX.sym('q1_p');
q2_p = SX.sym('q2_p');
q3_p = SX.sym('q3_p');




%% Defincion de cuantas acciones del control tiene el sistema
controls = [ul;um;un;w;q1_p;q2_p;q3_p]; 
n_control = length(controls);


%% Obstacles definition
obs_num = size(obs,3);
%% ONSTANT VALUES OF THE FUNSTION THESE VALUES NEED TO BE POSITIVE >0
ax_1 = 0.2;
ay_1 = 0.2;
az_1 = 0.2;

ax_2= 0.3;
ay_2 = 0.3;
az_2 = 0.3;
 %% THIS VALUE NEEDS TO BE POSITIVE DEFINED
n = 2;


%% Definicion de los las constantes dl sistema
l_1 = L(1);
l_2 = L(2);
l_3 = L(3);


%% Matrix generation Parts
J_11 = cos(psi);
J_12 = -sin(psi);
J_13 = 0;
J_14 = -sin(psi + q1)*(l_3*cos(q2 + q3) + l_2*cos(q2));
J_15 = -sin(psi + q1)*(l_3*cos(q2 + q3) + l_2*cos(q2));
J_16 = -cos(psi + q1)*(l_3*sin(q2 + q3) + l_2*sin(q2));
J_17 = -l_3*cos(psi + q1)*sin(q2 + q3);

J_21 = sin(psi);
J_22 = cos(psi);
J_23 = 0;
J_24 = cos(psi + q1)*(l_3*cos(q2 + q3) + l_2*cos(q2));
J_25 = cos(psi + q1)*(l_3*cos(q2 + q3) + l_2*cos(q2));
J_26 = -sin(psi + q1)*(l_3*sin(q2 + q3) + l_2*sin(q2));
J_27 = -l_3*sin(psi + q1)*sin(q2 + q3);

J_31 = 0;
J_32 = 0;
J_33 = 1;
J_34 = 0;
J_35 = 0;
J_36 = - l_3*cos(q2 + q3) - l_2*cos(q2);
J_37 = -l_3*cos(q2 + q3);

J_t = [J_11, J_12, J_13, J_14, J_15, J_16, J_17;...
     J_21, J_22, J_23, J_24, J_25, J_26, J_27;...
     J_31, J_32, J_33, J_34, J_35, J_36, J_37;...
     0,    0,    0,    1,    0,     0,   0;...
     0,    0,    0,    0,    1,     0,   0;...
     0,    0,    0,    0,    0,     1,   0;...
     0,    0,    0,    0,    0,     0,   1];
  
%% Jacobian platform only 
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

J_p = [J11, J12, J13, J14, J15, J16, J17;...
       J21, J22, J23, J24, J25, J26, J27;...
       J31, J32, J33, J34, J35, J36, J37];

%% System Evolution    
A =[J_t;...
    J_p];
   
rhs = (A*controls);

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states) + 3*obs_num);
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion
% 

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state

g = [g;X(:,1)-P(1:n_states)]; % initial condition constraints

%% Control gains
Q = 1*eye(3);
Q_p_e = 1*eye(2);
R = 1*[1/ul_max,0,0,0,0,0,0;...
       0,1/um_max,0,0,0,0,0;...
       0,0,1/un_max,0,0,0,0;...
       0,0,0,1/w_max,0,0,0;...
       0,0,0,0,1/q_1_max,0,0;...
       0,0,0,0,0,1/q_2_max,0;...
       0,0,0,0,0,0,1/q_3_max];


% FINAL COST
obj = 0;



%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar 
    he = ((X(1:3,k)-P(n_states*k+1:n_states*k+3)));
    u = con;
    he_p_e = tanh(X(1:2,k)-X(8:9,k));
    he_p_e_z = tanh(X(3,k)-X(10,k));
    
    %% OBSTACLES AVOINDANCE PART
    obstacles = P((n_states)+N*(n_states)+1:(n_states)+N*(n_states)+3*obs_num);
    
    obstacles_general = reshape(obstacles, 3, obs_num);
    
    
    V_vector_1 =  [];
    for i=1:1:obs_num
        aux_x_1 = ((X(1,k)-obstacles_general(1,i))^n)/ax_1;
        aux_y_1 = ((X(2,k)-obstacles_general(2,i))^n)/ay_1;
        aux_z_1 = ((X(3,k)-obstacles_general(3,i))^n)/az_1;
        
        value_1 = exp(-aux_x_1-aux_y_1-aux_z_1);
        
        V_vector_1 = [V_vector_1;value_1];
        
    end
    sum_field_1 = sum(V_vector_1);
    
    V_vector_2 =  [];
    for i=1:1:obs_num
        aux_x_2 = ((X(8,k)-obstacles_general(1,i))^n)/ax_2;
        aux_y_2 = ((X(9,k)-obstacles_general(2,i))^n)/ay_2;
        aux_z_2 = ((X(10,k)-obstacles_general(3,i))^n)/az_2;
        
        value_2 = exp(-aux_x_2-aux_y_2-aux_z_2);
        
        V_vector_2 = [V_vector_2;value_2];
        
    end
    sum_field_2 = sum(V_vector_2);
    
    %% Manipulability Section

    Jb = [[ -sin(X(5, k))*(l_3*cos(X(6, k) + X(7, k)) + l_2*cos(X(6, k))), -cos(X(5, k))*(l_3*sin(X(6, k) + X(7, k)) + l_2*sin(X(6, k))), -l_3*sin(X(6, k) + X(7, k))*cos(X(5, k))]
          [  cos(X(5, k))*(l_3*cos(X(6, k) + X(7, k)) + l_2*cos(X(6, k))), -sin(X(5, k))*(l_3*sin(X(6, k) + X(7, k)) + l_2*sin(X(6, k))), -l_3*sin(X(6, k) + X(7, k))*sin(X(5, k))]
          [                                             0,           - l_3*cos(X(6, k) + X(7, k)) - l_2*cos(X(6, k)),          -l_3*cos(X(6, k) + X(7, k))]];
    
    
    mani =  sqrt(det(Jb*Jb'))/0.0534;
  
    
    %% R manipulability
    R_mani = 1*[1/(mani+0.01),0,0,0,0,0,0;...
                    0,1/(mani+0.01),0,0,0,0,0;...
                    0,0,1/(mani+0.01),0,0,0,0;...
                    0,0,0,1/(mani+0.01),0,0,0;...
                    0,0,0,0,mani+0.01,0,0;...
                    0,0,0,0,0,mani+0.01,0;...
                    0,0,0,0,0,0,mani+0.01];
    
%     obj = obj + 1*(he'*Q*he)+(0.05)*(u'*R*R_mani*u) - (1-(mani))*mani^2  + 0.6*sum_field_1^2 + 1*(he_p_e'*Q_p_e*he_p_e) + 0.6*sum_field_2^2;
    obj = obj + 1*(he'*Q*he)+(0.05)*(u'*R*R_mani*u) - 0.8*mani^2  + (sum_field_1)*sum_field_1^2 + 0.7*mani*(he_p_e'*Q_p_e*he_p_e) + 1*(sum_field_2)*sum_field_2^2 - 0.5*mani*he_p_e_z^2;
    %% Actualizacion del sistema usando Euler runge kutta
    st_next = X(:,k+1);
    k1 = f(st, con);   % new 
    k2 = f(st + ts/2*k1, con); % new
    k3 = f(st + ts/2*k2, con); % new
    k4 = f(st + ts*k3, con); % new
    st_next_RK4 = st +ts/6*(k1 +2*k2 +2*k3 +k4); % new 
    
    %% Restricciones del sistema se =basan en el modelo del sistema
    g = [g;st_next-st_next_RK4]; 
end

% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_control*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 150;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-6;
opts.ipopt.acceptable_obj_change_tol = 1e-5;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:n_states*(N+1)) = -1e-10 ;  %-1e-20  %Equality constraints
args.ubg(1:n_states*(N+1)) = 1e-10;  %1e-20   %Equality constraints
% 

args.lbx(1:n_states:n_states*(N+1),1) = -inf; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = inf;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -inf; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = inf;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state z lower bound
args.ubx(3:n_states:n_states*(N+1),1) = inf;  %state z upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -inf; %state psi lower bound
args.ubx(4:n_states:n_states*(N+1),1) =  inf;  %state psi upper bound
% 
args.lbx(5:n_states:n_states*(N+1),1) =  -160*pi/180; %state q1 lower bound
args.ubx(5:n_states:n_states*(N+1),1) =  160*pi/180;  %state q1 upper bound

args.lbx(6:n_states:n_states*(N+1),1) =  45*pi/180; %state q2 lower bound
args.ubx(6:n_states:n_states*(N+1),1) =  165*pi/180;  %state q2 upper bound

args.lbx(7:n_states:n_states*(N+1),1) =  -140*pi/180; %state q3 lower bound
args.ubx(7:n_states:n_states*(N+1),1) =  140*pi/180;  %state q3 upper bound

args.lbx(8:n_states:n_states*(N+1),1) =  -inf; %state q1 lower bound
args.ubx(8:n_states:n_states*(N+1),1) =  inf;  %state q1 upper bound

args.lbx(9:n_states:n_states*(N+1),1) =  -inf; %state q2 lower bound
args.ubx(9:n_states:n_states*(N+1),1) =  inf;  %state q2 upper bound

args.lbx(10:n_states:n_states*(N+1),1) =  -inf; %state q3 lower bound
args.ubx(10:n_states:n_states*(N+1),1) =  inf;  %state q3 upper bound


%% Definicion de las restricciones de las acciones de control del sistema

args.lbx(n_states*(N+1)+1:n_control:n_states*(N+1)+n_control*N,1) = ul_min;  %
args.ubx(n_states*(N+1)+1:n_control:n_states*(N+1)+n_control*N,1) = ul_max;  %

args.lbx(n_states*(N+1)+2:n_control:n_states*(N+1)+n_control*N,1) = um_min;  %
args.ubx(n_states*(N+1)+2:n_control:n_states*(N+1)+n_control*N,1) = um_max;  % 

args.lbx(n_states*(N+1)+3:n_control:n_states*(N+1)+n_control*N,1) = un_min;  %
args.ubx(n_states*(N+1)+3:n_control:n_states*(N+1)+n_control*N,1) = un_max;  %

args.lbx(n_states*(N+1)+4:n_control:n_states*(N+1)+n_control*N,1) = w_min;  %
args.ubx(n_states*(N+1)+4:n_control:n_states*(N+1)+n_control*N,1) = w_max;  % 

args.lbx(n_states*(N+1)+5:n_control:n_states*(N+1)+n_control*N,1) = q_1_min;  %
args.ubx(n_states*(N+1)+5:n_control:n_states*(N+1)+n_control*N,1) = q_1_max;  % 

args.lbx(n_states*(N+1)+6:n_control:n_states*(N+1)+n_control*N,1) = q_2_min;  %
args.ubx(n_states*(N+1)+6:n_control:n_states*(N+1)+n_control*N,1) = q_2_max;  %

args.lbx(n_states*(N+1)+7:n_control:n_states*(N+1)+n_control*N,1) = q_3_min;  %
args.ubx(n_states*(N+1)+7:n_control:n_states*(N+1)+n_control*N,1) = q_3_max;  % 

end