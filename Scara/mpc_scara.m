function [f,solver,args] = mpc_scara(bounded, N, L1, L2, ts, obs)

addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;

%% Definicion de las restricciones en las acciones de control
q_1_max = bounded(1); 
q_1_min = bounded(2);

q_2_max = bounded(3);
q_2_min = bounded(4);



%% Generacion de las variables simbolicas de los estados del sistema
x = SX.sym('x'); 
y = SX.sym('y');
q1 = SX.sym('q1');
q2 = SX.sym('q2');
m = SX.sym('m');
V = SX.sym('V');

%% Definicion de cuantos estados en el sistema
states = [x;y;q1;q2;m;V];
n_states = length(states);

%% Generacion de las variables simbolicas de las acciones del control del sistema
q1_p = SX.sym('q1_p');
q2_p = SX.sym('q2_p');




%% Defincion de cuantas acciones del control tiene el sistema
controls = [q1_p;q2_p]; 
n_control = length(controls);


%% Definicion de los las constantes dl sistema
b1 = L1(1);
m1 = L1(2);
l1 = L1(3);
Iz1 = L1(4);

b2 = L2(1);
m2 = L2(2);
l2 = L2(3);
Iz2 = L2(4);

%% Obstacles Definition
obs_num = size(obs,3);
%% ONSTANT VALUES OF THE FUNSTION THESE VALUES NEED TO BE POSITIVE >0
ax = 0.1;
ay = 0.1;

 %% THIS VALUE NEEDS TO BE POSITIVE DEFINED
n = 2;
vi_aux_1 = [];
V_vector_1 = zeros(1,2);
for i=1:1:obs_num
    aux_x_1 = ((x-obs(1,1,i))^n)/ax;
    aux_y_1 = ((y-obs(2,1,i))^n)/ay;
    value_1 = exp(-aux_x_1-aux_y_1);
    aux_vector = value_1*[((x-obs(1,1,i))^(n-1))/ax,((y-obs(2,1,i))^(n-1))/ay];
    V_vector_1 = V_vector_1+aux_vector; 
    
end
%% Matrix generation Parts
J11 = l2*cos(q1 + q2) + l1*cos(q1);
J12 = l2*cos(q1 + q2);
J21 = l2*sin(q1 + q2) + l1*sin(q1);
J22 = l2*sin(q1 + q2);

J_t = [J11, J12;...
    J21, J22;...
     1, 0;...
     0, 1];

 %% Hessian Matrices
 %% Hessian Matrix q1
 H_1 = [[-l2*sin(q1 + q2) - l1*sin(q1), -l2*sin(q1 + q2)];...
     [l2*cos(q1 + q2) + l1*cos(q1),  l2*cos(q1 + q2)]];
 %% Hessian Matrix q2
 H_2 = [[-l2*sin(q1 + q2), -l2*sin(q1 + q2)]
       [l2*cos(q1 + q2),  l2*cos(q1 + q2)]];
 
   %% Jacobian Matrix Definition
   J = [J11, J12;...
       J21, J22];
   
   %% Aux Variables
   aux_hessian_1 = J*H_1';
   aux_hessian_2 = J*H_2';
   aux_J = inv(J*J');
   
   %% Vectorization of the matricex
   vec_h_1 = reshape(aux_hessian_1,4,1);
   vec_h_2 = reshape(aux_hessian_2,4,1);
   vec_J = reshape(aux_J,4,1);
   
   %% Manipulability
   mani =  sqrt(det(J*J'));
   a_m = 0.1;
   aux_m = exp(-(mani)^2/a_m);
   
   %% Create each value of the matrices
   J_11 = mani* vec_h_1'*vec_J;
   J_21 = mani* vec_h_2'*vec_J;
   
   %% Asign values to the matrix
   J_m = [J_11;...
       J_21];
   
   %% MAnipulability System
   aux = aux_m*-(2*(mani)/a_m);
   
   A =[J_t;...
      aux*J_m';...
      -n*V_vector_1*J];
   
rhs = (A*controls);

%% Definicion de kas funciones del sistema
f = Function('f',{states,controls},{rhs}); 
U = SX.sym('U',n_control,N);
P = SX.sym('P',n_states + N*(n_states));
%% vector que incluye el vector de estados y la referencia
X = SX.sym('X',n_states,(N+1));

%% Vector que representa el problema de optimizacion
g = [];  % restricciones de estados del problema  de optimizacion
% 

%% INITIAL CONDITION ERROR
st  = X(:,1); % initial state

g = [g;X(:,1)-P(1:n_states)]; % initial condition constraints

%% Control gains
Q = 1*eye(2);
R = 1*[1/q_1_max,0;...
       0, 1/q_2_max];


% FINAL COST
obj = 0;

%% Definicon del bucle para optimizacion a los largo del tiempo
for k = 1:N
    
    st = X(:,k);  con = U(:,k);

    %% Funcion costo a minimizar 
    he = [tanh((X(1:2,k)-P(n_states*k+1:n_states*k+2)))];
    u = [con];
    manipulability = X(5,k);
    h_obs1 = [X(6,k)];
    
    obj = obj + (1-abs(manipulability))*(1-abs(h_obs1))*(he'*Q*he)+(0.06)*(u'*R*u) + abs(1*manipulability)*(1-abs(h_obs1))*manipulability^2 + abs(h_obs1)*h_obs1^2;
%     obj = obj + (1-abs(h_obs1))*(he'*Q*he)+(0.05)*(u'*R*u) + abs(h_obs1)*h_obs1^2;

    %obj = obj + (0.9)^k*(he'*Q*he)+(0.05)^k*(u'*R*u);
    
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

% Cost final 


%% Control values constrains

% se crea el vector de desiscion solo de una columna
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_control*N,1)];

nlprob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlprob,opts);

args = struct;

args.lbg(1:n_states*(N+1)) = -1e-20 ;  %-1e-20  %Equality constraints
args.ubg(1:n_states*(N+1)) = 1e-20;  %1e-20   %Equality constraints
% 

args.lbx(1:n_states:n_states*(N+1),1) = -inf; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = inf;  %state x upper bound

args.lbx(2:n_states:n_states*(N+1),1) = -inf; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = inf;  %state y upper bound

args.lbx(3:n_states:n_states*(N+1),1) = -inf; %state q1 lower bound
args.ubx(3:n_states:n_states*(N+1),1) = inf;  %state q1 upper bound

args.lbx(4:n_states:n_states*(N+1),1) = -inf; %state q2 lower bound
args.ubx(4:n_states:n_states*(N+1),1) =  inf;  %state q2 upper bound

args.lbx(5:n_states:n_states*(N+1),1) =  -inf; %state m lower bound
args.ubx(5:n_states:n_states*(N+1),1) =  inf;  %state m upper bound

args.lbx(6:n_states:n_states*(N+1),1) =  -inf; %state m lower bound
args.ubx(6:n_states:n_states*(N+1),1) =  inf;  %state m upper bound


%% Definicion de las restricciones de las acciones de control del sistema
args.lbx(n_states*(N+1)+1:n_control:n_states*(N+1)+n_control*N,1) = q_1_min;  %
args.ubx(n_states*(N+1)+1:n_control:n_states*(N+1)+n_control*N,1) = q_1_max;  %

args.lbx(n_states*(N+1)+2:n_control:n_states*(N+1)+n_control*N,1) = q_2_min;  %
args.ubx(n_states*(N+1)+2:n_control:n_states*(N+1)+n_control*N,1) = q_2_max;  % 



end