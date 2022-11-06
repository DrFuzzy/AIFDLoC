clc,clear all,close all

initialize_LSDP_controllers_igva

%Threshold values
V_th_i=1e-4;
V_th_v=1e-5;
V_th_a=4e-5;

%slope of the incipient fault (not used here)
i_slope=-0.8;
v_slope=-0.6;
a_slope=-0.6;

%variables:
%-----------------------------------------------------------------
%simulation time: sim_time
%fault flags: i_fault_flag,v_fault_flag,a_fault_flag
%residuals: i_res,v_res,a_res
%recconfiguration signal: recon_sig
%Controllers id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
%Binary Switches outputs: i_bin_sw,v_bin_sw,a_bin_sw
%neuro-fault detection outputs: i_n_FD_out,v_n_FD_out,a_n_FD_out
% measurements:air_gap,current,flux,vert-velocity,vert_acceleration
%------------------------------------------------------------------

%***************************************
%Fault conditions - Fault free case (ff)
%***************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=7;
acceleration_fault_time=7;
%fault types: 1 - abrupt, 2 - incipient
fault_type_i=1;
fault_type_v=1;
fault_type_a=1;
%***************************************
sim('LSDP_design_maglev_FTC_nnFD_igva_DTS',[0 sim_t],sim_optns);

i_fault_flag_ff=i_fault_flag;
v_fault_flag_ff=v_fault_flag;
a_fault_flag_ff=a_fault_flag;
i_res_ff=i_res;
v_res_ff=v_res;
a_res_ff=a_res;
recon_sig_ff=recon_sig;
i_bin_sw_ff=i_bin_sw;
v_bin_sw_ff=v_bin_sw;
a_bin_sw_ff=a_bin_sw;
i_n_FD_out_ff=i_n_FD_out;
v_n_FD_out_ff=v_n_FD_out;
a_n_FD_out_ff=a_n_FD_out;
air_gap_ff=air_gap;
current_ff=current;
flux_ff=flux;
vert_velocity_ff=vert_velocity;
vert_acceleration_ff=vert_acceleration;

% figure
% plot(sim_time,air_gap),grid on;
% Title('Fault Free (airgap)')
% axis tight
% 
% figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
% plot(sim_time,recon_sig_ff),grid on;
% Title('Reconfiguration Signal with Fault Free (airgap)')
% legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
% axis tight
% 
% figure
% subplot(311), plot(sim_time,i_res_ff),grid on;
% %axis tight
% subplot(312), plot(sim_time,v_res_ff),grid on;
% %axis tight
% subplot(313), plot(sim_time,a_res_ff),grid on;
% axis tight

%*********************************************************
%Abrupt Fault conditions - Current Abrupt fault case (caf)
%*********************************************************
%Sensor fault time
current_fault_time=1;
velocity_fault_time=7;
acceleration_fault_time=7;
%fault types: 1 - abrupt, 2 - incipient
fault_type_i=1;
fault_type_v=1;
fault_type_a=1;
%***************************************
sim('LSDP_design_maglev_FTC_nnFD_igva_DTS',[0 sim_t],sim_optns);

i_fault_flag_caf=i_fault_flag;
v_fault_flag_caf=v_fault_flag;
a_fault_flag_caf=a_fault_flag;
i_res_caf=i_res;
v_res_caf=v_res;
a_res_caf=a_res;
recon_sig_caf=recon_sig;
i_bin_sw_caf=i_bin_sw;
v_bin_sw_caf=v_bin_sw;
a_bin_sw_caf=a_bin_sw;
i_n_FD_out_caf=i_n_FD_out;
v_n_FD_out_caf=v_n_FD_out;
a_n_FD_out_caf=a_n_FD_out;
air_gap_caf=air_gap;
current_caf=current;
flux_caf=flux;
vert_velocity_caf=vert_velocity;
vert_acceleration_caf=vert_acceleration;

display('current fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%*********************************************************
%Abrupt Fault conditions - Current Abrupt fault case (caf)
%*********************************************************
%Sensor fault time
current_fault_time=1;
velocity_fault_time=7;
acceleration_fault_time=7;
%fault types: 1-abrupt, 2-incipient
fault_type_i=2;
fault_type_v=1;
fault_type_a=1;
%***************************************
sim('LSDP_design_maglev_FTC_nnFD_igva_DTS',[0 sim_t],sim_optns);

current_cif=current;

figure
plot(sim_time(1:4000),current_caf(1:4000),'--',sim_time(1:4000),current_cif(1:4000),'-.',sim_time(1:4000),current_ff(1:4000)),grid on;
%title('Incipient and abrupt current fault profile')
%legend('Faulty current measurment','Faulty-free current measurment')
xlabel('time')
ylabel('current')
 text(0.9,4.2,'A')
 text(2.6,7.2,'Incipent')
 text(3.1,2.5,'Abrupt')
 text(2,2.75,'faultfree')
axis tight
