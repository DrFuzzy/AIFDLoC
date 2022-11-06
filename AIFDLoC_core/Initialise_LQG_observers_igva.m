% LQR_gen_500=load('LQR_gen_500.txt');
% [value indx]=min(LQR_gen_500(:,6));   %select the best ride quality
% LQR_gains=LQR_gen_500(indx,:);
% 
% load LQR_measurements.txt
% % figure(1)
% % plot(LQR_measurements(:,1),LQR_measurements(:,4),'*')
% 
% C_reg=[C_maglev(5,:) 0 %\ddot{z}
%       C_maglev(3,:) 0  %(z_t-z)
%       0 0 0 1];        %\int{z_t-z}
% 
% q1=1/(LQR_gains(1)^2);
% q2=1/(LQR_gains(2)^2);
% q3=1/(LQR_gains(3)^2);
% 
% Q_reg=[q1 q2 q3];
% 
% r_input=1/(LQR_gains(4)^2);
% 
% Q_lqr=[q1 0 0
%        0 q2 0
%        0 0 q3];
% 
% R_lqr=r_input;
% 
% %State and Input matrices including the extra state (integrator)
% [A_maglev,B_maglev,C_maglev,D_maglev]=ssdata(maglevssmodel);
% A_with_i=[A_maglev zeros(3,1);
%           0 0 1 0];
% B_with_i=[B_maglev(:,2);0];
% 
% sys01=ss(A_with_i,B_with_i,C_reg,zeros(3,1));
% 
% [Klqr_st,L_lqr,P_lqr]=lqry(sys01,Q_lqr,R_lqr);
% 
% Klqr_st;
% Klqr_i=-1*Klqr_st(1,1);
% Klqr_z_dot=-1*Klqr_st(1,2);
% Klqr_gap=-1*Klqr_st(1,3);
% Klqr_igap=-1*Klqr_st(1,4);

%V_i=0; V_gap=0; V_dotz=0;

% mode_sel=1;
% %sim('LQR_maglev',[0 6.6],sim_optns);
% 
%V_i=0;%(0.01*max(abs(current)))^2;                   %current sensor
%V_b=0;%(0.01*max(abs(flux)))^2;                      %flux sensor
%V_gap=0;%(0.01*max(abs(air_gap)))^2;                 %gap sensor
%V_zdot=0;%(0.01*max(abs(vert_velocity)))^2;          %velocity sensor
%V_zdotdot=0;%(0.01*max(abs(vert_acceleration)))^2;   %acceleration sensor
% 
% sensor_matrix=sensor_matrix_igva;
% Fd=0;
% K_2ss=K_2ss_igva;
% sim('LSDP_design_maglev',[0 6.6],sim_optns);

V_i=1e-7%(0.01*max(abs(current)))^2;                   %current sensor
V_b=1e-15%(0.01*max(abs(flux)))^2;                      %flux sensor
V_gap=1e-7%(0.01*max(abs(air_gap)))^2;                 %gap sensor
V_zdot=1e-7%(0.01*max(abs(vert_velocity)))^2;          %velocity sensor
V_zdotdot=1e-7%(0.01*max(abs(vert_acceleration)))^2;   %acceleration sensor
V_sn_full=[V_i V_b V_gap V_zdot V_zdotdot]

                        %i b gap zdot zdotdot
sensor_matrix_full=diag([1 1 1   1    1]);
B_maglev=[Bw_maglev Bu_maglev];

A_hat=A_maglev;
B_hat=B_maglev(:,2);
G_kbf=B_maglev(:,1);

W_kbf=3;

%igva
sensors_selected=[1 3 4 5];
%W_kbf=4;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_ia=sensor_matrix_full(sensors_selected,:);
C_hat_igva=C_maglev(sensors_selected,:);
D_hat_igva=D_maglev(sensors_selected,1);
[Kf_lqg_igva,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_igva,D_hat_igva,W_kbf,V_sn);
%gva
sensors_selected=[3 4 5];
%W_kbf=4;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_gva=sensor_matrix_full(sensors_selected,:);
C_hat_gva=C_maglev(sensors_selected,:);
D_hat_gva=D_maglev(sensors_selected,1);
[Kf_lqg_gva,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_gva,D_hat_gva,W_kbf,V_sn);
%iga
sensors_selected=[1 3 5];
%W_kbf=10;
%V_sn_full(3)=1e-4;
%V_sn_full(4)=1e-15;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_iga=sensor_matrix_full(sensors_selected,:);
C_hat_iga=C_maglev(sensors_selected,:);
D_hat_iga=D_maglev(sensors_selected,1);
[Kf_lqg_iga,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_iga,D_hat_iga,W_kbf,V_sn);
%igv
sensors_selected=[1 3 4];
%W_kbf=4;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_igv=sensor_matrix_full(sensors_selected,:);
C_hat_igv=C_maglev(sensors_selected,:);
D_hat_igv=D_maglev(sensors_selected,1);
[Kf_lqg_igv,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_igv,D_hat_igv,W_kbf,V_sn);
%ga
sensors_selected=[3 5];
%W_kbf=10;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_ga=sensor_matrix_full(sensors_selected,:);
C_hat_ga=C_maglev(sensors_selected,:);
D_hat_ga=D_maglev(sensors_selected,1);
[Kf_lqg_ga,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_ga,D_hat_ga,W_kbf,V_sn);

%ig
sensors_selected=[1 3];
%W_kbf=4;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_ig=sensor_matrix_full(sensors_selected,:);
C_hat_ig=C_maglev(sensors_selected,:);
D_hat_ig=D_maglev(sensors_selected,1);
[Kf_lqg_ig,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_ig,D_hat_ig,W_kbf,V_sn);

%gv
sensors_selected=[3 4];
%W_kbf=4;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_gv=sensor_matrix_full(sensors_selected,:);
C_hat_gv=C_maglev(sensors_selected,:);
D_hat_gv=D_maglev(sensors_selected,1);
[Kf_lqg_gv,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_gv,D_hat_gv,W_kbf,V_sn);

%g
sensors_selected=[3];
%W_kbf=4;
V_sn=diag(V_sn_full(sensors_selected));
sensor_matrix_g=sensor_matrix_full(sensors_selected,:);
C_hat_g=C_maglev(sensors_selected,:);
D_hat_g=D_maglev(sensors_selected,1);
[Kf_lqg_g,Pf_lqg,Ef_lqg]=kbf(A_hat,B_hat,G_kbf,C_hat_g,D_hat_g,W_kbf,V_sn);
