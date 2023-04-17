%% Code Manipulator 2DOF

% Clean variables
clc, clear all, close all;

% Time defintion variables
t_s = 0.1;
t_final = 120;
t = (0:t_s:t_final);

g = 9.8;
% System parameters L1
b1 = 1;
m1 = 0.8;
l1 = 1;
Iz1= 0.5;

% System parameters L2
b2 = 1;
m2 = 0.8;
l2 = 1;
Iz2= 0.5;


L1 = [b1 , m1, l1, Iz1];
L2 = [b2 , m2, l2, Iz2];

% Initial conditions system       
q = zeros(2, length(t)+1);
q(:, 1) = [150*pi/180;...
           -90*pi/180];
       
% Constant defintion
constans = [g, t_s];

% Robot definition
robot = manipulator_system(L1, L2, constans, q(:,1));

%h = zeros(5, length(t)+1-N);
h(:, 1) = [robot.get_general_position();...
           robot.get_positions();...
           robot.manipulability_filter()];



% Desired task space
xd = [2.0*sin(0.1*t);...
      0*ones(1, length(t))];
  
xdp = [(2.0*0.1)*cos(0.1*t);...
       0*ones(1, length(t))];
   
hd = [xd];

% Obtacles definition
obs_1  = [1 + 0.3*sin(0.5*t);...
          0*ones(1, length(t))];
      
obs_2 = [-1 + 0.3*cos(0.5*t);...
          0*ones(1, length(t))];
      
obs = zeros(2, length(t), 2);
obs(:, :, 1) = obs_1;
obs(:, :, 2) = obs_2;
%% POTENTIAL FIEL DEFINITION
V = potential_field(h(:,1), obs(:,1,:));

% General states NMPC
H = [h(1:5,1); sum(V)];
% Control Gains
K1 = 1*eye(2);
K2 = 1*eye(2);

control = controller(K1, K2, robot);

% Initialization control variable
u_cartesian = zeros(2, length(t));

% Manipulability
%m_robot = zeros(1, length(t)+1); 
m_robot(1, 1) = robot.manipulability();

%% NMPC Controller
N = 10; 

%% Boundaries of the control actions
bounded = [1.5; -1.5; 1.5; -1.5];
%% Definicion del vectro de control inicial del sistema
vc = zeros(N,2);
H0 = repmat(H,1,N+1)';

[f, solver, args] = mpc_scara(bounded, N, L1, L2, t_s, obs(:,1,:));

for k = 1:length(t)-N
    
    %% Control vector
    xe(:, k) = xd(:, k) - robot.get_general_position();
  
    %% Control  robot
    tic
    
    %% u_cartesian(:, k) = control.get_inverse_kinematics(h(:, k), xd(:, k), xdp(:, k));
    [H0, control] = NMPC(h(:,k), V(:, k), hd(:,:), k, H0, vc, args, solver, N);
    delta_t(k) = toc;
    toc
    optimal_control(:, k) = control(1, :)';
    estimation(:, k) = H0(1,end-1);
    
    %% System evolution
    h(:, k+1) = robot.system_f_kinematics(control(1, :)');
    m_robot(1, k+1) = robot.manipulability();
    
    %% Robot Obstacles
    V(:,k+1) = potential_field(h(:,k+1), obs(:, k+1, :));
    
    %% Update NMPC
    %% UPDATE OPTIMIZATION
    vc = [control(2:end,:);control(end,:)];
    H0 = [H0(2:end,:);H0(end,:)];
end
% figure
% set(gcf,'Position',[300 200 800 700])
% myVideo = VideoWriter('myVideoFile'); 
% myVideo.FrameRate = 2;  
% open(myVideo)
for k = 1:2:length(t)-N
    drawpend2(h(3:4, k), m1, m2, 0.3, l1, l2, xd(:,k), obs(:,k,:));
%     frame = getframe(gcf); %get frame
%     writeVideo(myVideo, frame);

end
% close(myVideo)
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(xe)),xe(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(xe)),xe(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$\tilde{\eta}_{x}$','$\tilde{\eta}_{y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(h)),h(3,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),h(4,:),'Color',[46,188,89]/255,'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$q_1$','$q_2$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(optimal_control)),optimal_control(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(optimal_control)),optimal_control(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$\dot{q}_1$','$\dot{q}_2$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(m_robot)),m_robot(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),h(5,:),'--','Color',[106,76,44]/255,'linewidth',1); hold on;
%plot(t(1,1:length(m_robot)),m_robot(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(estimation)),estimation(1,:),'.-','Color',[106,50,44]/255,'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$Manipulability $','$Manipulability_a $' },'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(delta_t)),delta_t(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$\Delta_t$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(h)),h(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1,1:length(h)),h(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1,1:length(V)),V(:,:),'linewidth',1); hold on;

grid('minor')
grid on;
legend({'$x$','$y$','$V$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);