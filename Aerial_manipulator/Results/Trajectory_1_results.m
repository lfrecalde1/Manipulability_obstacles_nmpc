%% Code to show the results of the Inverse kinematic controller

% Clean variables
clc, clear all, close all;
 
% Load variables of the system
load("Trajectory_1.mat")

% Change variable size
t = t(1, 1:length(He));
h = h(:, 1:length(t));
H = H(:, 1:length(t));
hd = hd(:,1:length(t));
obs(:,1:length(t),:);
V_b = V_b(1, 1:length(t));
V_p = V_p(1, 1:length(t));
D_b = D_b(:, 1:length(t));
D_p = D_p(:, 1:length(t));
m_robot_manipulador = m_robot_manipulador(1, 1:length(t));

% Parameters Robot
h_1 = L(1);
l11 = L(2);
l12 = L(3);
% Error definition
for k = 1:length(t)
    error_vector = hd(1:3, k)-H(1:3, k);
    error_NMPC(k) = norm(error_vector,2);
end

% error_NMPC = error_NMPC/max(error_NMPC);
% Figure propert%% Figures
lw = 1; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 700; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;

C18 = [0 0 0];
c5 = [130, 37, 37]/255;
c6 = [205, 167, 37]/255;
c7 = [81, 115, 180]/255;

C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;
C19 = [214, 157, 156]/255;
C20 = [160, 165, 172]/255;


C21 = [42 142 86]/255;
C22 = [139 177 221]/255;
C23 = [64 109 159]/255;
C24 = [233 230 129]/255;
C25 = [178 173 12]/255;
%% New color slection

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

% Figure 1
axes('Position',[0.041 0.06 .30 .25]);
ax_1.XLim = [min(H(1,:))-0.8 max(H(1,:))+0.8];
lim_x_1 = ax_1.XLim;
ax_1.YLim = [min(H(2,:))-1.5 max(H(2,:))+1.5];
lim_y_1 = ax_1.YLim;
%view([51 26])
%% Colorbar
hold on
Obstacle_1 = plot(obs(1,1,1),obs(2,1,1),'o','Color',[100,100,100]/255,'linewidth',5);
plot(obs(1,1,2),obs(2,1,2),'o','Color',[100,100,100]/255,'linewidth',4);
plot(obs(1,1,3),obs(2,1,3),'o','Color',[100,100,100]/255,'linewidth',4);
plot(obs(1,1,4),obs(2,1,4),'o','Color',[100,100,100]/255,'linewidth',4);

scatter(H(1, :),H(2, :),20,error_NMPC,'filled','MarkerFaceAlpha',0.5);
c = colorbar;
c.Label.Interpreter = 'latex';
c.Label.String = '$\textrm{Error}~||\tilde{\mathbf{\eta}}_a||$';

set(c,'TickLabelInterpreter','latex')



%% Desired Trajectory
hd_plot = line(hd(1,:),hd(2,:));
set(hd_plot, 'LineStyle', '-', 'Color', C18, 'LineWidth', 1.2*lw)

%% Real Trajectory
h_plot = line(H(1,:),H(2,:));
set(h_plot, 'LineStyle', '--', 'Color', C19, 'LineWidth', 1.2*lw)
h_plot_p = line(h(1,:),h(2,:));
set(h_plot_p, 'LineStyle', '-.', 'Color', C20, 'LineWidth', 1.2*lw)

%% Title of the image
hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',12,'interpreter','latex','Color',C18, 'position',[lim_x_1(1) lim_y_1(2)]);
hXLabel_1 = xlabel('$x~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);
hYLabel_1 = ylabel('$y~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([hd_plot, h_plot, h_plot_p, Obstacle_1 ],{'$^i\mathbf{r}^{d}_e$','$^i\mathbf{r}_e$','$^i\mathbf{r}_b$','$^i\mathbf{r}^{obs}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','northwest','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.0*fontsizeTicks)
% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.TickLength = [0.005;0.005];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';
%ax_2.MinorGridColor = '#8f8f8f';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;


% 
% Figure 2
axes('Position',[0.04 0.37 .30 .25]);
ax_2.XLim = [min(H(1,:))-0.1 max(H(1,:))+0.1];
ax_2.YLim = [min(H(3,:))-0.2 max(h(3,:))+0.2];
lim_x_2 = ax_2.XLim;
lim_y_2 = ax_2.YLim;
%% Colorbar
hold on
Obstacle_1 = plot(obs(1,1,1),obs(3,1,1),'o','Color',[100,100,100]/255,'linewidth',4);
plot(obs(1,1,2),obs(3,1,2),'o','Color',[100,100,100]/255,'linewidth',4);
plot(obs(1,1,3),obs(3,1,3),'o','Color',[100,100,100]/255,'linewidth',4);
plot(obs(1,1,4),obs(3,1,4),'o','Color',[100,100,100]/255,'linewidth',4);


scatter(H(1, :),H(3, :),20,error_NMPC,'filled','MarkerFaceAlpha',0.5);
c = colorbar;
c.Label.Interpreter = 'latex';
c.Label.String = '$\textrm{Error}~||\tilde{\mathbf{\eta}}_a||$';

set(c,'TickLabelInterpreter','latex')




%% Desired Trajectory
hd_plot_2 = line(hd(1,:),hd(3,:));
set(hd_plot_2, 'LineStyle', '-', 'Color', C18, 'LineWidth', 1.2*lw)

%% Real Trajectory
h_plot_2 = line(H(1,:),H(3,:));
set(h_plot_2, 'LineStyle', '--', 'Color', C19, 'LineWidth', 1.2*lw)

h_plot_p_2 = line(h(1,:), h(3, :));
set(h_plot_p_2, 'LineStyle', '-.', 'Color', C20, 'LineWidth', 1.2*lw)


%% Title of the image
hTitle_2 = title({'$\textrm{(b)}$'},'fontsize',12,'interpreter','latex','Color',C18, 'position',[lim_x_2(1) lim_y_2(2)]);
hXLabel_2 = xlabel('$x~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);
hYLabel_2 = ylabel('$z~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([hd_plot_2, h_plot_2, h_plot_p_2, Obstacle_1],{'$^i\mathbf{r}^{d}_e$','$^i\mathbf{r}_e$','$^i\mathbf{r}_b$','$^i\mathbf{r}^{obs}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','northwest','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.0*fontsizeTicks)
% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.TickLength = [0.005;0.005];
ax_2.TickDirMode = 'auto';
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
%ax_2.MinorGridColor = '#8f8f8f';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;
ax_2.XLim = [min(H(1,:))-0.2 max(H(1,:))+0.2];
ax_2.YLim = [min(H(3,:))-0.2 max(h(3,:))+0.2];
% 

axes('Position',[0.04 0.66 .35 .31])
%% Colorbar
scatter3(H(1, :),H(2, :), H(3, :),20,error_NMPC,'filled','MarkerFaceAlpha',0.1);
c = colorbar;
c.Label.Interpreter = 'latex';
c.Label.String = '$\textrm{Error}~||\tilde{\mathbf{\eta}}_a||$';

set(c,'TickLabelInterpreter','latex')

hold on
Obstacle_1 = plot3(obs(1,1,1),obs(2,1,1),obs(3,1,1),'o','Color',[100,100,100]/255,'linewidth',4);
plot3(obs(1,1,2),obs(2,1,2),obs(3,1,2),'o','Color',[100,100,100]/255,'linewidth',4);
plot3(obs(1,1,3),obs(2,1,3),obs(3,1,3),'o','Color',[100,100,100]/255,'linewidth',4);
plot3(obs(1,1,4),obs(2,1,4),obs(3,1,4),'o','Color',[100,100,100]/255,'linewidth',4);


%% Desired Trajectory
hd_plot_10 = line(hd(1,:),hd(2,:), hd(3, :));
set(hd_plot_10, 'LineStyle', '-', 'Color', C18, 'LineWidth', 1.2*lw)

% %% Real Trajectory
h_plot_10 = line(H(1,:),H(2,:), H(3, :));
set(h_plot_10, 'LineStyle', '--', 'Color', C19, 'LineWidth', 1.5*lw)

h_plot_p_10 = line(h(1,:),h(2,:), h(3, :));
set(h_plot_p_10, 'LineStyle', '-.', 'Color', C20, 'LineWidth', 1.2*lw)


%% PLot Aerial Manipulator
DimensionesManipulador(0,h_1,l11,l12-0.2,1);
Hexacoptero(0.05,[1 0 0]);
aux_plot = 5;
M_1=Manipulador3D(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot),0,0,0,h(5,aux_plot),h(6,aux_plot),h(7,aux_plot),0); 
rotate(M_1,[1 0 0],180,[h(1,aux_plot),h(2,aux_plot),h(3,aux_plot)])
UAV= Hexacoptero(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot));


aux_plot = 180;
M_1=Manipulador3D(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot),0,0,0,h(5,aux_plot),h(6,aux_plot),h(7,aux_plot),0); 
rotate(M_1,[1 0 0],180,[h(1,aux_plot),h(2,aux_plot),h(3,aux_plot)])
UAV= Hexacoptero(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot));

aux_plot = 380;
M_1=Manipulador3D(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot),0,0,0,h(5,aux_plot),h(6,aux_plot),h(7,aux_plot),0); 
rotate(M_1,[1 0 0],180,[h(1,aux_plot),h(2,aux_plot),h(3,aux_plot)])
UAV= Hexacoptero(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot));

aux_plot = 580;
M_1=Manipulador3D(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot),0,0,0,h(5,aux_plot),h(6,aux_plot),h(7,aux_plot),0); 
rotate(M_1,[1 0 0],180,[h(1,aux_plot),h(2,aux_plot),h(3,aux_plot)])
UAV= Hexacoptero(h(1,aux_plot),h(2,aux_plot),h(3,aux_plot),h(4,aux_plot));

%% Title of the image
hTitle_10 = title({'$\textrm{(a)}$'},'fontsize',12,'interpreter','latex','Color',C18);
hYLabel_10 = ylabel('$y~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);
hXLabel_10 = xlabel('$x~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);
hZLabel_10 = zlabel('$z~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_10 = legend([hd_plot_10, h_plot_10, h_plot_p_10, Obstacle_1],{'$^i\mathbf{r}^{d}_e$','$^i\mathbf{r}_e$','$^i\mathbf{r}_b$','$^i\mathbf{r}^{obs}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','northwest','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',1.0*fontsizeTicks)
    
% h_plot_10 = line(H(1,:),H(2,:), H(3, :));
% set(h_plot_10, 'LineStyle', '--', 'Color', C19, 'LineWidth', 1.2*lw)
%     
% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.005;0.005];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';
%ax_2.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;
ax_10.XLim = [min(H(1,:))-0.5 max(H(1,:))+0.5];
ax_10.YLim = [min(H(2,:))-0.5 max(H(2,:))+0.5];
ax_10.ZLim = [min(h(3,:))-0.5 max(h(3,:))+0.5];

axes('Position',[0.45 0.68 .5 .29]);
%% Data generation
error_x_plot = line(t(1:length(h)),hd(1,:)-H(1,:));
set(error_x_plot, 'LineStyle', '-', 'Color', C5, 'LineWidth', 1.2*lw);
error_y_plot = line(t(1:length(h)),hd(2,:)-H(2,:));
set(error_y_plot, 'LineStyle', '-', 'Color', C3, 'LineWidth', 1.2*lw);
error_z_plot = line(t(1:length(h)),hd(3,:)-H(3,:));
set(error_z_plot, 'LineStyle', '-', 'Color', c7, 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_3 = title({'$\textrm{(d)}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Control Error}~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([error_x_plot,error_y_plot, error_z_plot],{'$ ^i \tilde{r}_x$','$^i \tilde{r}_y$','$^i \tilde{r}_z$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.0*fontsizeTicks)
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;
ax_3.XLim = [0 t(end)];
% %% Zoom Plot
% axes('Position',[0.68 0.7 .2 .1]);
% index_zoom_1 = (t>=20) & (t<=30);
% error_x_plot_zoom = line(t(index_zoom_1),hd(1,(index_zoom_1))-H(1,(index_zoom_1)));
% error_y_plot_zoom = line(t(index_zoom_1),hd(2,(index_zoom_1))-H(2,(index_zoom_1)));
% error_z_plot_zoom = line(t(index_zoom_1),hd(3,(index_zoom_1))-H(3,(index_zoom_1)));
% 
% set(error_x_plot_zoom, 'LineStyle', '-', 'Color', C5, 'LineWidth', lw*0.8);
% set(error_y_plot_zoom, 'LineStyle', '-', 'Color', C3, 'LineWidth', lw*0.8);
% set(error_z_plot_zoom, 'LineStyle', '-', 'Color', c7, 'LineWidth', lw*0.8);
% 
% set(gca,'ticklabelinterpreter','latex',...
%          'fontsize',fontsizeTicks*0.5)
% ax_1_zoom = gca;
% ax_1_zoom.Box = 'on';
% ax_1_zoom.BoxStyle = 'full';
% ax_1_zoom.TickLength = [0.01;0.01];
% %ax_1_zoom.YLim = [-0.5 0.5];
% %ax_1_zoom.YTickLabel = [];
% %ax_1_zoom.XTickLabel = [];
% ax_1_zoom.TickDirMode = 'manual';
% ax_1_zoom.TickDir = 'in';
% ax_1_zoom.YMinorTick = 'on';
% ax_1_zoom.XMinorTick = 'on';
% ax_1_zoom.LineWidth = 0.5;
% ax_1_zoom.XMinorGrid = 'on';
% ax_1_zoom.YMinorGrid = 'on';
% 

%% Figure 5
axes('Position',[0.40 0.37 .55 .25]);
ax_4.XLim = [0 t(end)];
ax_4.YLim = [min(m_robot_manipulador(1,:))-0.5 max(m_robot_manipulador(1,:))+0.5];
lim_x_2 = ax_4.XLim;
lim_y_2 = ax_4.YLim;
m_manipulador_plot = line(t(1:length(h)),m_robot_manipulador(1,:));
set(m_manipulador_plot, 'LineStyle', '-', 'Color', C25, 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(e)}$'},'fontsize',12,'interpreter','latex','Color',C18,'position',[lim_x_2(1) lim_y_2(2)]);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Manipulability}$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([m_manipulador_plot],{'$m_a$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.0*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];
ax_4.YLim = [min(m_robot_manipulador(1,:))-0.5 max(m_robot_manipulador(1,:))+0.5];

%% Figure 6
axes('Position',[0.40 0.06 .55 .25]);

Obstacles_plot = line(t(1:length(h)),D_b(:,:));
set(Obstacles_plot, 'LineStyle', '-', 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(f)}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Obstacles Distance}~[m]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([Obstacles_plot],{'$d_{1}$','$d_{2}$','$d_{3}$','$d_{4}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.0*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;


% axes('Position',[0.45 0.1 .2 .1]);
% index_zoom_1 = (t>=0) & (t<=21);
% error_x_plot_zoom = line(t(index_zoom_1),D_b(1:2,index_zoom_1));
% 
% set(error_x_plot_zoom, 'LineStyle', '-', 'LineWidth', lw*0.8);
% 
% set(gca,'ticklabelinterpreter','latex',...
%          'fontsize',fontsizeTicks*0.5)
% ax_1_zoom = gca;
% ax_1_zoom.Box = 'on';
% ax_1_zoom.BoxStyle = 'full';
% ax_1_zoom.TickLength = [0.01;0.01];
% %ax_1_zoom.YLim = [-0.5 0.5];
% %ax_1_zoom.YTickLabel = [];
% %ax_1_zoom.XTickLabel = [];
% ax_1_zoom.TickDirMode = 'manual';
% ax_1_zoom.TickDir = 'in';
% ax_1_zoom.YMinorTick = 'on';
% ax_1_zoom.XMinorTick = 'on';
% ax_1_zoom.LineWidth = 0.5;
% ax_1_zoom.XMinorGrid = 'on';
% ax_1_zoom.YMinorGrid = 'on';
% ax_1_zoom.XLim = [0 max(t(index_zoom_1))];

% %% Data generation

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_trajectoria_1.pdf -q101
