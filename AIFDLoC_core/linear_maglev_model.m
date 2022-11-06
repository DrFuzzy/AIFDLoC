%function [maglevssmodel_nom maglevssmodel_uss]=maglev_model(M,G0,sensor_sel)
M_nl=1000;                    %suspended mass -kg
G0_nl=0.015;                  %Nominal air gap -m
B0_nl=1;                      %Nominal flux density -T
I0_nl=10;                     %Nominal current
F0_nl=M_nl*9.81;              %Nominal force
R_nl=10;                      %resistance -ohms
L_nl=0.1;                     %inductance -H
N_nl=2000;                    %
A_nl=0.2*0.05;                %poleface area -m^2
Kb_nl=(B0_nl*G0_nl)/I0_nl;
Kf_nl=F0_nl*Kb_nl^2/(B0_nl)^2;
dotz0_nl=0;
ddotz0_nl=0;

% M_nl_u=ureal('M',M_nl,'percentage',10);
% R_nl_u=ureal('Ra',R_nl,'percentage',50);
% L_nl_u=ureal('La',L_nl,'percentage',50);
% B0_nl_u=ureal('B0',B0_nl,'percentage',10);
% I0_nl_u=ureal('I0',I0_nl,'percentage',10);
% F0_nl_u=ureal('F0',F0_nl,'percentage',10);
% G0_nl_u=ureal('G0',G0_nl,'percentage',10);

%uncrtin_param=[R_nl_u B0_nl_u I0_nl_u F0_nl_u L_nl_u G0_nl_u M_nl_u]

Kb_nl=(B0_nl*G0_nl)/I0_nl;
Kf_nl=F0_nl*Kb_nl^2/(B0_nl)^2;

%states: [delta_i delta_z_dot delta_gap]
A_maglev=[-R_nl/(L_nl+((Kb_nl*N_nl*A_nl)/G0_nl)) -(Kb_nl*N_nl*A_nl*I0_nl)...
    /(G0_nl^2*(L_nl+(Kb_nl*N_nl*A_nl/G0_nl))) 0;
      +(2*Kf_nl*I0_nl)/(M_nl*G0_nl^2) 0 -(2*Kf_nl*I0_nl^2)/(M_nl*G0_nl^3);
       0 -1 0];

Bu_maglev=[1/(L_nl+((Kb_nl*N_nl*A_nl)/G0_nl)) 0 0]';

Bw_maglev=[(Kb_nl*N_nl*A_nl*I0_nl)/(G0_nl^2*(L_nl+(Kb_nl*N_nl*A_nl/G0_nl))) 0 1]';

C_maglev=[1 0 0;                                                                              %current
      Kb_nl/G0_nl 0 -Kb_nl*I0_nl/G0_nl^2                                                %Flux
      0 0 1                                                                                   %Gap
      0 1 0                                                                                   %dot_z
      +(2*Kf_nl*I0_nl)/(M_nl*G0_nl^2) 0 -(2*Kf_nl*I0_nl^2)/(M_nl*G0_nl^3)];       %ddot_z

D_maglev=[0 0
     0 0
     0 0
     0 0
     0 0];


maglevssmodel=ss(A_maglev,[Bw_maglev Bu_maglev],C_maglev,D_maglev,'inputname',{'zt','u'},'outputname',...
    {'i','b','gap','z_dot','z_dotdot'});

%maglevssmodel_uss=uss(A_maglev,[Bw_maglev Bu_maglev],C_maglev,D_maglev,'inputname',{'zt','u'},'outputname',...
%    {'i','b','gap','z_dot','z_dotdot'});