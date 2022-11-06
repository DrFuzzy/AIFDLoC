
load current_fault_noise
load flux_fault_noise

%initiate MAGLEV parameters
linear_maglev_model

%sim_t=6.6;%determine the simulation time
sim_t=4;

%Determine the deterministic input at 15m/s, 15%
Tsamp=1/1e3;               %simulation sampling time
[t_det,v_out,V_speed]=track_det(1,0.5,5,15,200,Tsamp);
y_det=v_out(:,4);

%figure
%plot(t_det,v_out(:,2:4)),grid on
%axis tight

%Determine the random input(stochastic input) to the MAGLEV
%A_track=1e-7;             %track roughness (m)
%V_speed;                  %vechicle speed (m/s)
%track_var=2*(pi)^2*(A_track*V_speed);
%t_fin=6.6;
%t_stoch=[0:Tsamp:t_fin];
%y_stoch=sqrt(track_var)*randn(1,length(t_stoch))/sqrt(Tsamp); % normalize using Ts

load stoch_track_input.txt
t_stoch=stoch_track_input(2,:);
y_stoch=stoch_track_input(1,:);

%stoch_track_input=[y_stoch t_stoch];
%save('stoch_track_input.txt','stoch_track_input','-ascii','-double')

%sim_optns=simset('solver','ode23tb','MaxStep',1e-4);%Simulator parameters

sim_optns=simset('solver','ode14x','FixedStep',Tsamp);%Simulator parameters

omega_band=logspace(-7,15,100);

display('LSDP initialisation started.')


             %i b gap zdot zdotdot
sensors=diag([1 1 1   1    1]);

k=1;
W1=1;
M_obj=4;%Assign the objective functions (Mk)

%LOAD THE BANK OF LSDP
%

%%%%%DEFINE THE USER'S CONTROLLER SELECTION CRITERIA%%%%%
criteria_selected=[4 16 15];

min_max_criteria=[0    0 ;   %minimum value required
                  0.5  10;   %maximum value required
                  1    15];  %1-select the maximum, 0-select the minimum

sensor_sele=[1 3 4 5]';%SELECT THE SENSOR SER

P_sys_nom=maglevssmodel(sensor_sele,2);

weighting_names=[{'Wi'},{'Wb'},{'Wg'},{'Wz'},{'Wzz'}];

sensors_name=['i'; 'b'; 'g'; 'v' ;'a'];
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_igva=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');
%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_igva,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);

%%
sensor_sele=[3 4 5]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);

%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_gva=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');

%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_gva,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;

%%
sensor_sele=[1 3 5]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);

%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_iga=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');
%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_iga,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;
%%
sensor_sele=[1 3 4]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);
%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_igv=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');

%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_igv,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;
%%
sensor_sele=[3 5]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);
%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_ga=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');


%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_ga,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;
%%
sensor_sele=[1 3]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);
%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_ig=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');

%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_ig,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;
%%
sensor_sele=[3 4]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);

%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************
sensor_matrix_gv=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');

%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_gv,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;
%%
sensor_sele=[3]';%SELECT THE SENSOR SER

cnt_sel=1;

P_sys_nom=maglevssmodel(sensor_sele,2);
sensor_set=sensors_name(sensor_sele',:); %Choose the sensor names
sensors_selected=sensor_set';

load_controller=['cr_' sensors_selected 'measure_f.txt'];
controller=load(load_controller);

%***************************************************************************
V_var=2+size(sensor_sele,1);
load_txt=[sensors_selected 'measure.txt'];%for temp simulations
measurements=load(load_txt);%for temp simulations
load_txt=[sensors_selected '.txt'];%for temp simulations
chromosome=load(load_txt);%for temp simulations
%Select the best controler according to user's controller selection criteria
[controller measurements_cr sensors_cr controller_selected_index]=...
    criteria_req(chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);
%***************************************************************************

sensor_matrix_g=sensors(sensor_sele,:);
x=controller;
weighting_names_selected=weighting_names(sensor_sele');

%lowpass filter for S
Mp=10^x(k,1);
omp=10^x(k,2);
Ap=10^x(k,3);
ordp=1;
W_gap=tf([1/Mp^(1/ordp) omp],[1 omp*Ap^(1/ordp)])^ordp;

W2_m=[x(k,V_var-size(sensor_sele,1)+2:V_var)];

for we=1:size(W2_m,2);
     W2_measur(1,we)=10^W2_m(1,we);
end

n_me=1;n_p=1;
if strmatch('Wi',weighting_names_selected,'exact')
    W2_i=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_i=[];, end
if strmatch('Wb',weighting_names_selected,'exact')
    W2_b=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else W2_b=[];,end
if strmatch('Wg',weighting_names_selected,'exact')
    W2_g=W_gap;
else, W2_g=[];,end
if strmatch('Wz',weighting_names_selected,'exact')
    W2_z=W2_measur(n_me);,n_p=n_p+1;,n_me=n_me+1;
else,W2_z=[];,end
if strmatch('Wzz',weighting_names_selected,'exact')
    W2_zz=W2_measur(n_me);
else, W2_zz=[];,end

W2=append(W2_i,W2_b,W2_g,W2_z,W2_zz);

[K_2ss_g,cl,gopt,info]=ncfsyn(P_sys_nom,W1,W2);
emax=info.emax;

display('LSDP initialisation finished.')


%%
%LOAD THE BANK OF OBSERVERS
%
display('observers initialisation started.')

Initialise_LQG_observers_igva

V_i=0;%(0.01*max(abs(current)))^2;                   %current sensor
V_b=0;%(0.01*max(abs(flux)))^2;                      %flux sensor
V_gap=0;%(0.01*max(abs(air_gap)))^2;                 %gap sensor
V_zdot=0;%(0.01*max(abs(vert_velocity)))^2;          %velocity sensor
V_zdotdot=0;%(0.01*max(abs(vert_acceleration)))^2;   %acceleration sensor

display('Observers initialisation finished.')