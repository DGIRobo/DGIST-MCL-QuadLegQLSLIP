clear; clc; close all;

% system('run_win.bat');

filename_PD = 'C:\myproject\mujoco\mujoco-2.2.1-windows-x86_64\myProject\quad_legFL_jump_release\data.csv';
% filename_DOB = 'result_RWDOB.csv';
% filename_ADM = 'result_Admittance.csv';

T_PD = readtable(filename_PD); %check T.Properties
% VariableNames = T_PD.Properties.VariableNames;
% T_DOB = readtable(filename_DOB); %check T.Properties
% T_ADM = readtable(filename_ADM); %check T.Properties

Arr_PD = table2array(T_PD);
% Arr_DOB = table2array(T_DOB);
% Arr_ADM = table2array(T_ADM);

[m,n] = size(Arr_PD);

% PD
% t = zeros(m,1);
t           = Arr_PD(:,1);

r_ref_PD       = Arr_PD(:,2);
qr_ref_PD      = Arr_PD(:,3);
r_act_PD       = Arr_PD(:,4);
qr_act_PD      = Arr_PD(:,5);

out_acc     = Arr_PD(:,6);
out_Acc_old   = Arr_PD(:,7);
out_Acc_old2  = Arr_PD(:,8);
ydR_act_PD     = Arr_PD(:,9);

fxR_hat_PD     = Arr_PD(:,10);
fyR_hat_PD     = Arr_PD(:,11);

grf_x_PD       = Arr_PD(:,12);
grf_y_PD       = Arr_PD(:,13);
grf_z_PD       = Arr_PD(:,14);

touch_PD     = Arr_PD(:,15);

trunk_vel_y_PD     = Arr_PD(:,16);



%%
% DOB
% t_DOB           = Arr_DOB(:,1);
% 
% r_ref_DOB       = Arr_DOB(:,2);
% qr_ref_DOB      = Arr_DOB(:,3);
% r_act_DOB       = Arr_DOB(:,4);
% qr_act_DOB      = Arr_DOB(:,5);
% 
% xdR_ref_DOB     = Arr_DOB(:,6);
% ydR_ref_DOB     = Arr_DOB(:,7);
% xdR_act_DOB     = Arr_DOB(:,8);
% ydR_act_DOB     = Arr_DOB(:,9);
% 
% fxR_hat_DOB     = Arr_DOB(:,10);
% 
% grf_x_DOB       = Arr_DOB(:,12);
% 
% touch_DOB     = Arr_DOB(:,15);
% 
% foot_vel_z_DOB  = Arr_DOB(:,18);
% 
% torso_pos_z_DOB = Arr_DOB(:,21);
% 
% torso_vel_z_DOB = Arr_DOB(:,24);
% 
% % Admittance
% t_ADM           = Arr_ADM(:,1);
% 
% r_ref_ADM       = Arr_ADM(:,2);
% qr_ref_ADM      = Arr_ADM(:,3);
% r_act_ADM       = Arr_ADM(:,4);
% qr_act_ADM      = Arr_ADM(:,5);
% 
% xdR_ref_ADM     = Arr_ADM(:,6);
% ydR_ref_ADM     = Arr_ADM(:,7);
% xdR_act_ADM     = Arr_ADM(:,8);
% ydR_act_ADM     = Arr_ADM(:,9);
% 
% fxR_hat_ADM     = Arr_ADM(:,10);
% 
% grf_x_ADM       = Arr_ADM(:,12);
% 
% touch_ADM     = Arr_ADM(:,15);
% 
% foot_vel_z_ADM  = Arr_ADM(:,18);
% 
% torso_pos_z_ADM = Arr_ADM(:,21);
% 
% torso_vel_z_ADM = Arr_ADM(:,24);

%% plotting
clc; close all;
lw = 1.2;
lw_gain = 2;
fs = 12;

simStartime = 0;
simEndtime = 20;


% RW tracking
figure()
fig1 = tiledlayout(2,2,"TileSpacing",'compact','Padding','compact');
nexttile;
hold on
plot(t, r_ref_PD, 'k--',   'LineWidth', lw_gain*lw);
plot(t, r_act_PD, 'r-.',     'LineWidth', lw);
% plot(t, r_act_DOB, 'b',   'LineWidth', lw);
xlim([simStartime simEndtime])
ylabel('$r$ (m)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
grid

nexttile;
hold on
plot(t, qr_ref_PD, 'k--',  'LineWidth', lw_gain*lw);
plot(t, qr_act_PD, 'r-.',    'LineWidth', lw);
% plot(t, qr_act_DOB, 'b',    'LineWidth', lw);
xlim([simStartime simEndtime])
ylabel('$\theta_r$ (rad)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
legend('Reference','w/o RWDOB', 'w/RWDOB','FontName','Times New Roman','location','northeast')
grid

nexttile;
hold on
plot(t, xdR_ref_PD,   'k--',  'LineWidth', lw_gain*lw);
plot(t, xdR_act_PD, 'r-.',    'LineWidth', lw);
% plot(t, xdR_act_DOB, 'b',    'LineWidth', lw);
xlim([simStartime simEndtime])
ylabel('$\dot{x}^R$ (m/s)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
grid

nexttile;
hold on
plot(t, ydR_ref_PD,   'k--',  'LineWidth', lw_gain*lw);
plot(t, ydR_act_PD, 'r-.',    'LineWidth', lw);
% plot(t, ydR_act_DOB, 'b',    'LineWidth', lw);
xlim([simStartime simEndtime])
xlabel(fig1,'Time (sec)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
ylabel('$\dot{y}^R$ (m/s)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
grid


% GRF Estimation
figure()
fig2 = tiledlayout(3,1,"TileSpacing",'compact','Padding','compact');
% nexttile;
% hold on
% plot(t, torso_pos_z_PD, 'r-.', 'LineWidth', lw);
% plot(t, torso_pos_z_DOB, 'b', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$p_z^{torso}$ (m)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
% legend('w/o RWDOB','w/ RWDOB', 'FontName','Times New Roman','location','northeast')

nexttile;
hold on
plot(t, -grf_x_PD,   'k--', 'LineWidth', lw_gain*lw);
plot(t, -fxR_hat_PD, 'r',   'LineWidth', lw);
% plot(t, -grf_x_DOB,   'b--', 'LineWidth', lw_gain*lw);
% plot(t, -fxR_hat_DOB, 'g',   'LineWidth', lw);
xlim([simStartime simEndtime])
% ylim([0 20])
grid
ylabel('$\hat{f}^R_x$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
legend('Force Sensor','RWFOB', 'FontName','Times New Roman','location','northeast')

nexttile;
hold on
plot(t, grf_y_PD,   'r-.', 'LineWidth', 1.5*lw);
plot(t, -fyR_hat_PD, 'b',   'LineWidth', lw);
xlim([simStartime simEndtime])
% ylim([0 20])
grid
xlabel(fig2,'Time (sec)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
ylabel('$\hat{f}^R_y$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

% nexttile;
% hold on
% plot(t, grf_z,   'r-.', 'LineWidth', 1.5*lw);
% % plot(t, fyR_hat, 'b',   'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$\hat{f}^R_z$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t, touch_PD,  'LineWidth', lw);
xlim([simStartime simEndtime])
grid
ylabel('$\sigma_x$','FontName','Times New Roman','interpreter','latex','FontSize',fs);
legend('Touch', 'FontName','Times New Roman','location','northeast')


%%
figure()
fig3 = tiledlayout(3,1,"TileSpacing",'compact','Padding','compact');
% nexttile;
% hold on
% plot(t, torso_pos_x, 'r', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$p_x^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

% nexttile;
% hold on
% plot(t, torso_pos_y, 'r', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$p_y^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t, torso_pos_z_PD, 'r-.', 'LineWidth', lw);
plot(t, torso_pos_z_DOB, 'b', 'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$p_z^{torso}$ (m)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

% nexttile;
% hold on
% plot(t, torso_vel_x, 'g', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$v_x^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

% nexttile;
% hold on
% plot(t, torso_vel_y, 'g', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$v_y^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t, torso_vel_z_PD, 'g', 'LineWidth', lw);
plot(t, foot_vel_z_PD, 'b', 'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$v_z^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);


% nexttile;
% hold on
% plot(t, foot_vel_x, 'b', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$v_x^{foot}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

% nexttile;
% hold on
% plot(t, foot_vel_y, 'b', 'LineWidth', lw);
% xlim([0 simEndtime])
% grid
% ylabel('$v_y^{foot}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t, foot_vel_z_PD, 'b', 'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$v_y^{foot}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);



%% plotting
clc; close all;
lw = 1.2;
lw_gain = 1.5;
fs = 10;

simEndtime = 10;

% RW tracking
figure()
fig1 = tiledlayout(2,2,"TileSpacing",'compact','Padding','compact');
nexttile;
hold on
plot(t_ADM, r_ref_ADM, 'r-.',   'LineWidth', lw_gain*lw);
plot(t_ADM, r_act_ADM, 'b',     'LineWidth', lw);
xlim([0 simEndtime])
ylabel('$r$ (m)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
grid

nexttile;
hold on
plot(t_ADM, qr_ref_ADM, 'r-.',  'LineWidth', lw_gain*lw);
plot(t_ADM, qr_act_ADM, 'b',    'LineWidth', lw);
xlim([0 simEndtime])
ylabel('$\theta_r$ (rad)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
legend('Reference','Actual','FontName','Times New Roman','location','northeast')
grid

nexttile;
hold on
plot(t_ADM, xdR_ref_ADM,   'r-.',  'LineWidth', lw_gain*lw);
plot(t_ADM, xdR_act_ADM, 'b',    'LineWidth', lw);
xlim([0 simEndtime])
ylabel('$\dot{x}^R$ (m/s)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
grid

nexttile;
hold on
plot(t_ADM, ydR_ref_ADM,   'r-.',  'LineWidth', lw_gain*lw);
plot(t_ADM, ydR_act_ADM, 'b',    'LineWidth', lw);
xlim([0 simEndtime])
xlabel(fig1,'Time (sec)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
ylabel('$\dot{y}^R$ (m/s)','FontName','Times New Roman','interpreter','latex','FontSize',fs);
grid


% GRF Estimation
figure()
fig2 = tiledlayout(2,1,"TileSpacing",'compact','Padding','compact');
nexttile;
hold on
plot(t_ADM, -grf_x_ADM,   'r-.', 'LineWidth', lw_gain*lw);
plot(t_ADM, -fxR_hat_ADM, 'b',   'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$\hat{f}^R_x$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t_ADM, touch_ADM,   'LineWidth', lw_gain*lw);
xlim([0 simEndtime])
grid
ylabel('$\sigma_x$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

figure()
fig3 = tiledlayout(3,1,"TileSpacing",'compact','Padding','compact');
nexttile;
hold on
plot(t_ADM, torso_pos_z_ADM, 'r', 'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$p_z^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t_ADM, torso_vel_z_ADM, 'g', 'LineWidth', lw);
plot(t_ADM, foot_vel_z_ADM, 'b', 'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$v_z^{torso}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);

nexttile;
hold on
plot(t_ADM, foot_vel_z_ADM, 'b', 'LineWidth', lw);
xlim([0 simEndtime])
grid
ylabel('$v_y^{foot}$ (N)','FontName','Times New Roman','interpreter','latex','FontSize',fs);