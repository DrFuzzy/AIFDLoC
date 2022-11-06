clc,clear all,close all

initialize_LSDP_controllers_igva

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
%slope of the incipient fault (not used here)
i_slope=-0.8;
v_slope=-0.6;
a_slope=-0.6;
%Threshold values - for Deterministic response
%display('Threshold values - for Deterministic response')
V_th_i=1;
V_th_v=1;
V_th_a=1;

sim_optns=simset('solver','ode14x','FixedStep',Tsamp);%Simulator parameters

iter=1;

for iter=1:100

fprintf('simulation with iFD....trial %d \n',iter);
overall_time=tic; %reset timer
%sim('LSDP_design_maglev_FTC_nnFD_igva_DTS_NIO',[0 sim_t],sim_optns);
sim('iFD_time',[0 sim_t],sim_optns);
%display('Time taken with iFD - seconds')
iFD_timer=toc(overall_time);
fprintf('Time taken for the iFD configuration %g second \n',iFD_timer);

fprintf('simulation with bank of observers....trial %d \n',iter);
%sim('LSDP_design_maglev_FTC_lqgFD_igva',[0 sim_t],sim_optns);
sim('KE_time',[0 sim_t],sim_optns);
%display('Time taken with observers - seconds')
lqg_timer=toc(overall_time);
fprintf('time taken with bank of observers %g second \n',lqg_timer);

timer_ratio=lqg_timer/iFD_timer;

FD_timer(iter,:)=[iter iFD_timer lqg_timer timer_ratio];


end

iFD_mean_time=sum(FD_timer(:,2))/size(FD_timer,1)
ke_FD_mean_time=sum(FD_timer(:,3))/size(FD_timer,1)

save('fd_timer.txt','FD_timer','-ascii','-double')

figure
plot(FD_timer(:,1),FD_timer(:,2),FD_timer(:,1),FD_timer(:,3),'--'),grid on
legend('iFD unit','Bank of 7 Kalman estimators')
title('Execution time of iFD unit vs Bank of kalman estimators.')
xlabel('Iteration')
ylabel('Execution time - s')
%axis tight

