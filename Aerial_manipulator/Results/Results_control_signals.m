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
control = control(:, 1:length(t));

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
lw = 1.2; % linewidth 1
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

C26 = [229 23 101]/255;
C27 = [42 142 86]/255;
C28 = [64 109 159]/255;

%% New color slection

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;


%% Figure 5
axes('Position',[0.04 0.37 .45 .25]);
ax_4.XLim = [0 t(end)];
platform_plot_l = line(t(1:length(h)),control(1,:));
set(platform_plot_l, 'LineStyle', '-','Color',[196,81,128]/255, 'LineWidth', 1.2*lw);

platform_plot_m = line(t(1:length(h)),control(2,:));
set(platform_plot_m, 'LineStyle', '-','Color',C13 ,'LineWidth', 1.2*lw);

platform_plot_n = line(t(1:length(h)),control(3,:));
set(platform_plot_n, 'LineStyle', '-','Color',C2, 'LineWidth', 1.2*lw);

platform_plot_w = line(t(1:length(h)),control(4,:));
set(platform_plot_w, 'LineStyle', '-','Color',C16, 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(a)}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Drone Control Values}[m/s][rad/s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([platform_plot_l, platform_plot_m, platform_plot_n, platform_plot_w],{'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.0*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.XTickLabel = [];
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];

%% Figure 5
axes('Position',[0.53 0.37 .45 .25]);
ax_4.XLim = [0 t(end)];
platform_plot_q1 = line(t(1:length(h)),control(5,:));
set(platform_plot_q1, 'LineStyle', '-','Color',C26, 'LineWidth', 1.2*lw);

platform_plot_q2 = line(t(1:length(h)),control(6,:));
set(platform_plot_q2, 'LineStyle', '-.','Color',C27 ,'LineWidth', 1.2*lw);

platform_plot_q3 = line(t(1:length(h)),control(7,:));
set(platform_plot_q3, 'LineStyle', '--','Color',C28, 'LineWidth', 1.2*lw);



% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(c)}$'},'fontsize',12,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Arm Control Values}[rad/s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([platform_plot_q1, platform_plot_q2, platform_plot_q3],{'$\dot{q}_{1c}$','$\dot{q}_{2c}$','$\dot{q}_{3c}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',1.0*fontsizeTicks)
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.XTickLabel = [];
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];


%% Sample Time


%% Figure 5
axes('Position',[0.04 0.06 .45 .25]);
ax_4.XLim = [0 t(end)];
sample_time_plot = line(t(1:length(h)),0.7*delta_t(1,:));
set(sample_time_plot, 'LineStyle', '-','Color',C24, 'LineWidth', 1.2*lw);

aux_sample = ts*ones(1, length(t));
sample_time_limit = line(t(1:length(h)), aux_sample);
set(sample_time_limit, 'LineStyle', '--','Color',C8, 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(b)}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Computational Time}[s]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([sample_time_plot, sample_time_limit],{'$\Delta t$','$t_s$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
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
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];

axes('Position',[0.53 0.06 .45 .25]);
ax_4.XLim = [0 t(end)];
platform_plot_q1 = line(t(1:length(h)),h(5,:));
set(platform_plot_q1, 'LineStyle', '-','Color',C26, 'LineWidth', 1.2*lw);

platform_plot_q2 = line(t(1:length(h)),h(6,:));
set(platform_plot_q2, 'LineStyle', '-.','Color',C27 ,'LineWidth', 1.2*lw);

platform_plot_q3 = line(t(1:length(h)),h(7,:));
set(platform_plot_q3, 'LineStyle', '--','Color',C28, 'LineWidth', 1.2*lw);



% fig1_comps.p1 = ul_plot;
%% Title of the image
hTitle_4 = title({'$\textrm{(d)}$'},'fontsize',12,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$\textrm{Arm Values}[rad]$','fontsize',8,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([platform_plot_q1, platform_plot_q2, platform_plot_q3],{'${q}_{1}$','${q}_{2}$','${q}_{3}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','southeast','NumColumns',1,'TextColor','black');
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
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;
ax_4.XLim = [0 t(end)];

% %% Data generation

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_Control_Values_1.pdf -q101