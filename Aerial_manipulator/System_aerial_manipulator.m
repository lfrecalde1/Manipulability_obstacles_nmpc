%% Controller Aerial Manipulator Manipulabily %%

%% TIME PARAMETERS
clear all;
close all;
clc;
warning off;
ts = 0.1;
tfin = 100;
t = (0.1:ts:tfin);
N = 20; 
%% LOCATION OF THE MANIPULATOR
a1 = 0;
b1 = 0;
h_1 = 0.12;


%% LINKS LENGTH
l11 = 0.3;
l12 = 0.26;
L = [h_1, l11, l12];

%% DESIRED TRAJECTORY
[hxd1, hyd1, hzd1, hthd1] =  Trajectory(t,ts,3);

%% DESIRED TRAJECTORY VECTORS
hd = [hxd1; hyd1; hzd1];

%% UAV
x(1) = hxd1(1);
y(1) = hyd1(1);
z(1) = hzd1(1)+1;
psi(1)= 0*pi/180;

%% ARM 1
q11(1)= 0*pi/180;
q12(1)= 90*pi/180;
q13(1)= -10*pi/180;


h = zeros(7, length(t)+1-N);
h(:, 1) = [x(1);y(1);z(1);psi(1);q11(1);q12(1);q13(1)];



%% DESIRED INTERNAL ANGLES OF THJE SYSTEM
q1d = 10*pi/180*ones(1, length(t));
q2d = 45*pi/180*ones(1, length(t));
q3d = 10*pi/180*ones(1, length(t));
qd = [q1d;q2d;q3d];

%% FORWARD KINEMATICS
H(:, 1) = forward_kinematics(h(:, 1), L);


%% Control Gains
k1 = 1;
k2 = 1;
k3 = 10;
%% VARIABLES MANIPULABILITY
m_robot_control(1, 1) = manipulability_control(h(:, 1), L);
m_robot_manipulador(1, 1) = manipulability_manipulador(h(:, 1), L);

%% OBSTACLES DEFINTION
obs_1  = [hxd1(5)*ones(1, length(t));...
          hyd1(5)*ones(1, length(t));...
          hzd1(5)*ones(1, length(t))];
      
obs_2 = [hxd1(180)*ones(1, length(t));...
         hyd1(180)*ones(1, length(t));...
         hzd1(180)*ones(1, length(t))];
     
obs_3 = [hxd1(380)*ones(1, length(t));...
         hyd1(380)*ones(1, length(t));...
         hzd1(380)*ones(1, length(t))];     
     
obs_4 = [hxd1(580)*ones(1, length(t));...
         hyd1(580)*ones(1, length(t));...
         hzd1(580)*ones(1, length(t)) + 0.4];   
     
obs = zeros(3, length(t), 4);
obs(:, :, 1) = obs_1;
obs(:, :, 2) = obs_2;
obs(:, :, 3) = obs_3;
obs(:, :, 4) = obs_4;
%% NMPC Controller
H_complete = [H(1:3, 1);h(4:7,1);h(1:3,1)];

%% Boundaries of the control actions
bounded = [2.5; -2.5; 2.5; -2.5; 2.5; -2.5; 2.5; -2.5; 2.5; -2.5; 2.5; -2.5; 2.5; -2.5];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,7);
H0 = repmat(H_complete,1,N+1)';

[f, solver, args] = mpc_aerial_manipulator(bounded, N, L, ts, obs(:,1,:));

%% DISTANCE OBSTACLES FIELD
V_b = potential_field(H(:,1), obs(:,1,:));
V_p = potential_field(h(1:3,1), obs(:, 1, :));

%% DISTANCE OBTACLES
D_b = distance_objects(H(1:3,1), obs(:,1,:));
D_p = distance_objects(h(1:3,1), obs(:,1,:));

%% CONTROL LOOP SYSTEM
for k=1:length(t)-N
    %% DEFINITION CONTROL ERROR
    He(:, k) = hd(:, k) - H(1:3,k);
    
    %% Control  robot
    tic
    
    %% CONTROL USING NMPC FORMULATION
    %[H0, control_nmpc] = NMPC(h(:,k), H(:,k), obs(:,k,:), hd(:,:), k, H0, vc, args, solver ,N);
    delta_t(k) = toc;
    toc
    %% CONTROL VELOCITIES
    ul(k) = 0;
    um(k) = 0;
    un(k) = 0;
    w(k) = 0;
    q11_p(k) = 0;
    q12_p(k) = 0;
    q13_p(k) = 0;
    
    %% GENERAL CONTROL VECTOR
    control(:, k) = [ul(k);um(k);un(k);w(k);q11_p(k);q12_p(k);q13_p(k)];
    
    %% AERIAL ROBOT
    [h(:, k+1)] = system_f_kinematics(h(:, k), control(:, k), L, ts);
    m_robot_control(1, k+1) = manipulability_control(h(:, k+1), L);
    m_robot_manipulador(1, k+1) = manipulability_manipulador(h(:, k+1), L);

    %% FORWARD SYSTEM
    H(:, k+1) = forward_kinematics(h(:, k+1), L);
    
    %% DISTANCE FIELD
    V_b(:,k+1) = potential_field(H(:,k+1), obs(:, k+1, :));
    V_p(:,k+1) = potential_field(h(1:3,k+1), obs(:, k+1, :));
    
    D_b(:,k+1) = distance_objects(H(1:3,k), obs(:,k+1,:));
    D_p(:,k+1)= distance_objects(h(1:3,k), obs(:,k+1,:));


    %% GET NEW VALUES OF NMPC CONTROLLER
    %vc = [control_nmpc(2:end,:);control_nmpc(end,:)];
    %H0 = [H0(2:end,:);H0(end,:)];
end

%% PLOT SYSTEM
% ANIMATION

close all; paso =5; fig=figure;
set(gcf,'Position',[300 200 800 600])

view(-15,15) % Angulo de vista
title ("Simulacion Cinematica")
%% Configuracion del Manipulador Aereo
DimensionesManipulador(0,h_1,l11,l12-0.2,1);
Hexacoptero(0.04,[1 0 0]);
M_1=Manipulador3D(h(1,1),h(2,1),h(3,1),h(4,1),0,0,0,h(5,1),h(6,1),h(7,1),0); % GrÃ¡fica el brazo por encima el UAV
rotate(M_1,[1 0 0],180,[h(1,1),h(2,1),h(3,1)])
hold on
UAV= Hexacoptero(h(1,1),h(2,1),h(3,1),h(4,1));
%%
G1 = plot3(hxd1(1),hyd1(1),hzd1(1),'r','linewidth',2);hold on
G2 = plot3(H(1,1),H(2,1),H(3,1),'-k','linewidth',2);hold on
OBS_1 = plot3(obs(1,1,1),obs(2,1,1),obs(3,1,1),'x','Color',[0,171,217]/255,'linewidth',2);
OBS_2 = plot3(obs(1,1,2),obs(2,1,2),obs(3,1,2),'x','Color',[0,171,217]/255,'linewidth',2);
OBS_3 = plot3(obs(1,1,3),obs(2,1,3),obs(3,1,3),'x','Color',[0,171,217]/255,'linewidth',2);
OBS_4 = plot3(obs(1,1,4),obs(2,1,4),obs(3,1,4),'x','Color',[0,171,217]/255,'linewidth',2);




view([52 19])
for k = 1:paso:length(t)-N
    %% SYSTEM DRAW
    drawnow
    delete(M_1);
    delete(UAV);
    delete(G1);
    delete(G2);
    delete(OBS_1);
    delete(OBS_2);
    delete(OBS_3);
    delete(OBS_4)
    
    %% DRONE PLOT
    M_1=Manipulador3D(h(1,k),h(2,k),h(3,k),h(4,k),0,0,0,h(5,k),h(6,k),h(7,k),0); % GrÃ¡fica el brazo por encima el UAV    A2 = brazoPlot(h(1,k),h(2,k),h(3,k),a1,b1,h(4,k),h(5,k),h(6,k),h(7,k),0,scaleRobot);hold on
    rotate(M_1,[1 0 0],180,[h(1,k),h(2,k),h(3,k)]);
    UAV= Hexacoptero(h(1,k),h(2,k),h(3,k),h(4,k));
    G1 = plot3(hxd1(1:k),hyd1(1:k),hzd1(1:k),'r','linewidth',2);grid on
    G2 = plot3(H(1,1:k),H(2,1:k),H(3,1:k),'--k','linewidth',2);grid on
    OBS_1 = plot3(obs(1,k,1),obs(2,k,1),obs(3,k,1),'x','Color',[0,171,217]/255,'linewidth',2);
    OBS_2 = plot3(obs(1,k,2),obs(2,k,2),obs(3,k,2),'x','Color',[0,171,217]/255,'linewidth',2);
    OBS_3 = plot3(obs(1,k,3),obs(2,k,3),obs(3,k,3),'x','Color',[0,171,217]/255,'linewidth',2);
    OBS_4 = plot3(obs(1,k,4),obs(2,k,4),obs(3,k,4),'x','Color',[0,171,217]/255,'linewidth',2);

    xlabel('X[m]'), ylabel('Y[m]'), zlabel('Z[m]')
    
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(He)),He(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(He)),He(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(He)),He(3,:),'Color',[26,115,160]/255,'linewidth',1); hold on;
grid('minor')
grid on;
legend({'$\tilde{\eta}_{x}$','$\tilde{\eta}_{y}$','$\tilde{\eta}_{z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(h)),h(5,:),'--','Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),h(6,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),h(7,:),'--','Color',[26,115,160]/255,'linewidth',1); hold on;

plot(t(1,1:length(h)),q1d(1,1:length(h)),'Color',[108,105,105]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),q2d(1,1:length(h)),'Color',[108,105,105]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),q3d(1,1:length(h)),'Color',[108,105,105]/255,'linewidth',1); hold on;


grid('minor')
grid on;
legend({'$q_1$','$q_2$', '$q_3$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(m_robot_control)),m_robot_control,'Color',[108,105,105]/255,'linewidth',1); hold on
% plot(t(1,1:length(estimation)),estimation,'--','Color',[20,105,105]/255,'linewidth',1); hold on

grid('minor')
grid on;
legend({'$Manipulability_c$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);  

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(m_robot_manipulador)),m_robot_manipulador,'-','Color',[108,105,105]/255,'linewidth',1); hold on

grid('minor')
grid on;
legend({'$Manipulability_{m}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9); 

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul)),ul,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul)),um,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul)),un,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul)),w,'Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul)),q11_p,'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul)),q12_p,'--','Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul)),q13_p,'--','Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{q}_1$','$\dot{q}_2$','$\dot{q}_3$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(V_b)),V_b(:,:),'linewidth',1); hold on;
plot(t(1,1:length(V_p)),V_p(:,:),'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$V_b$','$V_p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
%ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(D_b)),D_b(:,:),'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$D_b$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(D_p)),D_p(:,:),'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$D_p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


save("System_configuration", "t", "ts", "N", "L", "h", "hd", "H", "obs", "V_p", "V_b", "D_b", "D_p", "He", "control", "m_robot_manipulador", "delta_t")