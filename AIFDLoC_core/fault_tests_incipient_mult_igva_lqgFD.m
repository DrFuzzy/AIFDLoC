 clc,clear all,close all

initialize_LSDP_controllers_igva
mode_sel=1;
fprintf('Deterministic/ncipient/multiplicative fault tests (%g).\n',mode_sel)
%variables:
%-----------------------------------------------------------------
%simulation time: sim_time
%fault flags: i_fault_flag,v_fault_flag,a_fault_flag
%residuals: i_res,v_res,a_res
%recconfiguration signal: recon_sig
%Controllers id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
%Binary Switches outputs: i_bin_sw,v_bin_sw,a_bin_sw
%neuro-fault detection outputs: i_n_FD_out,v_n_FD_out,a_n_FD_out
%measurements:air_gap,current,flux,vert-velocity,vert_acceleration
%------------------------------------------------------------------

%Threshold values - for Deterministic response
%display('Threshold values - for Deterministic response')
% V_th_i=9e-5;
% V_th_v=10e-6;
% V_th_a=14e-6;
V_th_i=10e-4;
V_th_v=20e-4;
V_th_a=4e-3;

fprintf('Current threshold:%g\nVelocity threshold:%g\nAcceleration threshold:%g\n'...
    ,V_th_i,V_th_v,V_th_a);

%slope of the incipient fault
i_slope=20;
v_slope=6;
a_slope=20;

%FAULT TYPES: 
%1-abrupt\multiplicative, 2-abrupt\additive
%3-incipient\additive, 4-incipient\multiplicative
%5-abrupt\bias
fault_type_i=4;
fault_type_v=4;
fault_type_a=4;

%***************************************
%Incipient/Fault conditions
%Fault free case (ff)
%***************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=7;
acceleration_fault_time=7;
display('Fault free test.')
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_ff=i_fault_flag;
v_fault_flag_ff=v_fault_flag;
a_fault_flag_ff=a_fault_flag;
recon_sig_ff=recon_sig;
air_gap_ff=air_gap;
current_ff=current;
flux_ff=flux;
vert_velocity_ff=vert_velocity;
vert_acceleration_ff=vert_acceleration;

display('Deterministic/incipient/multiplicative fault test')
%*********************************************************
%Deterministic/incipient/multiplicative Fault conditions
%Current incipient fault case (caf)
%*********************************************************
%Sensor fault time
current_fault_time=1;
velocity_fault_time=7;
acceleration_fault_time=7;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_caf=i_fault_flag;
v_fault_flag_caf=v_fault_flag;
a_fault_flag_caf=a_fault_flag;
recon_sig_caf=recon_sig;
air_gap_caf=air_gap;
current_caf=current;
flux_caf=flux;
vert_velocity_caf=vert_velocity;
vert_acceleration_caf=vert_acceleration;

display('current fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,current_caf,'--',sim_time,current_ff),grid on;
%title('Current fault profile')
legend('Measurement with faulty current sensor','Measurement with fault-free conditions')
xlabel('Time - s')
ylabel('Current - A')
%text(1,3.9,'A')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_caf),grid on;
title('Reconfiguration Signal with Faulty acceleration')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_caf,'--',sim_time,air_gap_ff),grid on;
%title('Airgap with faulty current sensor')
legend('Airgap with faulty current sensor','Airgap signal with fault free conditions')
xlabel('time')
ylabel('airgap')
text(0.85,0.0051,'A')
text(1.5,0.0012,'newcontroller')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time(2:2000),recon_sig_caf(2:2000)),grid on;
title('Reconfiguration Signal with Faulty current @ 1s')
%legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
xlabel('Time - sec')
ylabel('Controller index number')
axis tight

%*******************************************
%Deterministic/incipient/multiplicative Fault conditions
%velocity fault case (vaf)
%*******************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=1;
acceleration_fault_time=7;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_vaf=i_fault_flag;
v_fault_flag_vaf=v_fault_flag;
a_fault_flag_vaf=a_fault_flag;
recon_sig_vaf=recon_sig;
air_gap_vaf=air_gap;
current_vaf=current;
flux_vaf=flux;
vert_velocity_vaf=vert_velocity;
vert_acceleration_vaf=vert_acceleration;

display('velocity fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_velocity_vaf),grid on;
title('Velocity incipient fault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_vaf),grid on;
title('Reconfiguration Signal with incipient velocity fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_vaf),grid on;
title('Current Fault')
legend('Fault free','Current Fault')
axis tight


%*******************************************************
%Deterministic/incipient/multiplicative Fault conditions
%Acceleration fault case (aaf)
%*******************************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=7;
acceleration_fault_time=1;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_aaf=i_fault_flag;
v_fault_flag_aaf=v_fault_flag;
a_fault_flag_aaf=a_fault_flag;
recon_sig_aaf=recon_sig;
air_gap_aaf=air_gap;
current_aaf=current;
flux_aaf=flux;
vert_velocity_aaf=vert_velocity;
vert_acceleration_aaf=vert_acceleration;

display('acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_aaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_aaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_aaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Deterministic/incipient/multiplicative Fault conditions
%Current-Acceleration fault case (caaf)
%*******************************************************
%Sensor fault time
current_fault_time=0.5;
velocity_fault_time=7;
acceleration_fault_time=1.5;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_caaf=i_fault_flag;
v_fault_flag_caaf=v_fault_flag;
a_fault_flag_caaf=a_fault_flag;
recon_sig_caaf=recon_sig;
air_gap_caaf=air_gap;
current_caaf=current;
flux_caaf=flux;
vert_velocity_caaf=vert_velocity;
vert_acceleration_caaf=vert_acceleration;

display('current-acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_caaf,sim_time,current_caaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_caaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Deterministic/incipient/multiplicative Fault conditions - Current-velocity fault case (cvaf)
%*******************************************************
%Sensor fault time
current_fault_time=0.5;
velocity_fault_time=1.5;
acceleration_fault_time=7;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_cvaf=i_fault_flag;
v_fault_flag_cvaf=v_fault_flag;
a_fault_flag_cvaf=a_fault_flag;
recon_sig_cvaf=recon_sig;
air_gap_cvaf=air_gap;
current_cvaf=current;
flux_cvaf=flux;
vert_velocity_cvaf=vert_velocity;
vert_acceleration_cvaf=vert_acceleration;

display('current-velocity fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_cvaf,sim_time,current_cvaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_cvaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_cvaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Deterministic/incipient/multiplicative Fault conditions - Velocity-Acceleration fault case (vaaf)
%*******************************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=0.5;
acceleration_fault_time=1.5;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_vaaf=i_fault_flag;
v_fault_flag_vaaf=v_fault_flag;
a_fault_flag_vaaf=a_fault_flag;
recon_sig_vaaf=recon_sig;
air_gap_vaaf=air_gap;
current_vaaf=current;
flux_vaaf=flux;
vert_velocity_vaaf=vert_velocity;
vert_acceleration_vaaf=vert_acceleration;

display('velocity-acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_vaaf,sim_time,current_vaaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_vaaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_vaaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Deterministic/incipient/multiplicative Fault conditions - current-velocity-acceleration fault case (cvaaf)
%*******************************************************
%Sensor fault time
current_fault_time=2.5;
velocity_fault_time=1.5;
acceleration_fault_time=0.5;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_cvaaf=i_fault_flag;
v_fault_flag_cvaaf=v_fault_flag;
a_fault_flag_cvaaf=a_fault_flag;
recon_sig_cvaaf=recon_sig;
air_gap_cvaaf=air_gap;
current_cvaaf=current;
flux_cvaaf=flux;
vert_velocity_cvaaf=vert_velocity;
vert_acceleration_cvaaf=vert_acceleration;

display('current-velocity-acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_cvaaf,sim_time,current_cvaaf),grid on;
title('Acceleration incipient fault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_cvaaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'-.',sim_time,air_gap_cvaaf),grid on;
title('Air gap with acceleration\velocity\current sequent Faults')
text(0.35,4.5e-3,'A')
text(1.5,1.2e-3,'B')
text(2.05,-0.6e-3,'C')
xlabel('Time - s')
ylabel('Airgap - m')
legend('acceleration\velocity\current sequent Faults','Fault free')
axis tight

fprintf('Deterministic/incipient/multiplicative fault tests finished.')
display('Press a key to proceed to stochastic responses test.')

pause
%%
clc,close all

initialize_LSDP_controllers_igva

%***************************
%Stochastic responses check
%***************************
mode_sel=2;
fprintf('Stochastic/incipient/additive fault tests (%d).\n',mode_sel)

%Threshold values - for stochastic response
display('Threshold values - for stochastic response.')

V_th_i=10e-3;
V_th_v=7e-3;
V_th_a=0.04;

fprintf('Current threshold:%g\nVelocity threshold:%g\nAcceleration threshold:%g\n'...
    ,V_th_i,V_th_v,V_th_a);

%slope of the incipient fault
i_slope=20;
v_slope=6;
a_slope=20;

%FAULT TYPES: 
%1-abrupt\multiplicative, 2-abrupt\additive
%3-incipient\additive, 4-incipient\multiplicative
%5-abrupt\bias
fault_type_i=4;
fault_type_v=4;
fault_type_a=4;


%***************************************
%Fault conditions - Fault free case (ff)
%***************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=7;
acceleration_fault_time=7;
display('fault free simulation.')
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_ff=i_fault_flag;
v_fault_flag_ff=v_fault_flag;
a_fault_flag_ff=a_fault_flag;
recon_sig_ff=recon_sig;
air_gap_ff=air_gap;
current_ff=current;
flux_ff=flux;
vert_velocity_ff=vert_velocity;
vert_acceleration_ff=vert_acceleration;

%*********************************************************
%Stochastic/incipient/multiplicative Fault conditions - Current incipient fault case (caf)
%*********************************************************
%Sensor fault time
current_fault_time=1;
velocity_fault_time=7;
acceleration_fault_time=7;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_caf=i_fault_flag;
v_fault_flag_caf=v_fault_flag;
a_fault_flag_caf=a_fault_flag;
recon_sig_caf=recon_sig;
current_caf=current;
flux_caf=flux;
vert_velocity_caf=vert_velocity;
vert_acceleration_caf=vert_acceleration;

display('current fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,current_caf,'--',sim_time,current_ff),grid on;
title('Current fault profile')
legend('Faulty current measurment','Faulty-free current measurment')
xlabel('Time - sec')
ylabel('Current - A')
text(1,3.9,'A')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_caf),grid on;
title('Reconfiguration Signal with Faulty acceleration')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

% figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time(2:2000),recon_sig_caf(2:2000)),grid on;
title('Reconfiguration Signal with Faulty current @ 1s')
%legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
xlabel('Time - sec')
ylabel('Controller index number')
axis tight

%***************************************************
%Stochastic/incipient/multiplicative Fault conditions - velocity fault case (vaf)
%***************************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=1;
acceleration_fault_time=7;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_vaf=i_fault_flag;
v_fault_flag_vaf=v_fault_flag;
a_fault_flag_vaf=a_fault_flag;
recon_sig_vaf=recon_sig;
air_gap_vaf=air_gap;
current_vaf=current;
flux_vaf=flux;
vert_velocity_vaf=vert_velocity;
vert_acceleration_vaf=vert_acceleration;

display('velocity fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_velocity_vaf),grid on;
title('Velocity incipient fault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_vaf),grid on;
title('Reconfiguration Signal with incipient velocity fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_vaf),grid on;
title('Current Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Stochastic/incipient/multiplicative Fault conditions - Acceleration fault case (aaf)
%*******************************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=7;
acceleration_fault_time=1;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_aaf=i_fault_flag;
v_fault_flag_aaf=v_fault_flag;
a_fault_flag_aaf=a_fault_flag;
recon_sig_aaf=recon_sig;
air_gap_aaf=air_gap;
current_aaf=current;
flux_aaf=flux;
vert_velocity_aaf=vert_velocity;
vert_acceleration_aaf=vert_acceleration;

display('acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_aaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_aaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_aaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Stochastic/incipient/multiplicative Fault conditions - Current-Acceleration fault case (caaf)
%*******************************************************
%Sensor fault time
current_fault_time=0.5;
velocity_fault_time=7;
acceleration_fault_time=1.5;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_caaf=i_fault_flag;
v_fault_flag_caaf=v_fault_flag;
a_fault_flag_caaf=a_fault_flag;
recon_sig_caaf=recon_sig;
air_gap_caaf=air_gap;
current_caaf=current;
flux_caaf=flux;
vert_velocity_caaf=vert_velocity;
vert_acceleration_caaf=vert_acceleration;

display('current-acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_caaf,sim_time,current_caaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_caaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_caaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Stochastic/incipient/multiplicative Fault conditions - Current-velocity fault case (cvaf)
%*******************************************************
%Sensor fault time
current_fault_time=0.5;
velocity_fault_time=1.5;
acceleration_fault_time=7;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_cvaf=i_fault_flag;
v_fault_flag_cvaf=v_fault_flag;
a_fault_flag_cvaf=a_fault_flag;
recon_sig_cvaf=recon_sig;
air_gap_cvaf=air_gap;
current_cvaf=current;
flux_cvaf=flux;
vert_velocity_cvaf=vert_velocity;
vert_acceleration_cvaf=vert_acceleration;

display('current-velocity fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_cvaf,sim_time,current_cvaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_cvaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_cvaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Stochastic/incipient/multiplicative Fault conditions - Velocity-Acceleration fault case (vaaf)
%*******************************************************
%Sensor fault time
current_fault_time=7;
velocity_fault_time=0.5;
acceleration_fault_time=1.5;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_vaaf=i_fault_flag;
v_fault_flag_vaaf=v_fault_flag;
a_fault_flag_vaaf=a_fault_flag;
recon_sig_vaaf=recon_sig;
air_gap_vaaf=air_gap;
current_vaaf=current;
flux_vaaf=flux;
vert_velocity_vaaf=vert_velocity;
vert_acceleration_vaaf=vert_acceleration;

display('velocity-acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_vaaf,sim_time,current_vaaf),grid on;
title('Acceleration incipient fault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_vaaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_vaaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

%*******************************************************
%Stochastic/incipient/multiplicative Fault conditions - current-velocity-acceleration fault case (cvaaf)
%*******************************************************
%Sensor fault time
current_fault_time=2.5;
velocity_fault_time=1.5;
acceleration_fault_time=0.5;
fprintf('Current fault time occured:%g\nVelocity fault time occured:%g\nAcceleration fault time occured:%g\n'...
    ,current_fault_time,velocity_fault_time,acceleration_fault_time);
%***************************************
sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);

i_fault_flag_cvaaf=i_fault_flag;
v_fault_flag_cvaaf=v_fault_flag;
a_fault_flag_cvaaf=a_fault_flag;
recon_sig_cvaaf=recon_sig;
air_gap_cvaaf=air_gap;
current_cvaaf=current;
flux_cvaaf=flux;
vert_velocity_cvaaf=vert_velocity;
vert_acceleration_cvaaf=vert_acceleration;

display('current-velocity-acceleration fault')
[i_fault_flag(end,:) v_fault_flag(end,:) a_fault_flag(end,:)]

%Find the fault time
[i_fault_time,v_fault_time,a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time);
fprintf('Current fault time detected:%g\nVelocity fault time detected:%g\nAcceleration fault time detected:%g\n'...
    ,i_fault_time,v_fault_time,a_fault_time);

figure
plot(sim_time,vert_acceleration_cvaaf,sim_time,current_cvaaf),grid on;
title('Acceleration fincipient ault profile')
axis tight

figure%id: 0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g
plot(sim_time,recon_sig_cvaaf),grid on;
title('Reconfiguration Signal with acceleration fault')
legend('0:igva,1:gva,2:iga,3:ga,4:igv,5:gv,6:ig,7:g')
axis tight

figure
plot(sim_time,air_gap_ff,'--',sim_time,air_gap_cvaaf),grid on;
title('Air gap with acceleration Fault')
legend('Fault free','Current Fault')
axis tight

