%clc,clear all

sensors_name=['i'; 'b'; 'g'; 'v' ;'a'];

sensor_sele=[1 3 4 5]'%SELECT THE SENSOR SET igva
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_igva_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_igva_det=load(c_out);
%sensor_sele=[1 3 4 5]'%SELECT THE SENSOR SET igva
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_igva_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_igva_stoch=load(c_out);


sensor_sele=[3 4 5]'%SELECT THE SENSOR SET gva
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_gva_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_gva_det=load(c_out);
%sensor_sele=[3 4 5]'%SELECT THE SENSOR SET gva
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_gva_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_gva_stoch=load(c_out);

sensor_sele=[1 3 5]'%SELECT THE SENSOR SET iga
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_iga_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_iga_det=load(c_out);
%sensor_sele=[1 3 5]'%SELECT THE SENSOR SET iga
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_iga_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_iga_stoch=load(c_out);


sensor_sele=[1 3 4]'%SELECT THE SENSOR SET igv
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_igv_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_igv_det=load(c_out);
%sensor_sele=[1 3 4]'%SELECT THE SENSOR SET igv
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_igv_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_igv_stoch=load(c_out);

sensor_sele=[3 5]'%SELECT THE SENSOR SET ga
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_ga_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_ga_det=load(c_out);
%sensor_sele=[3 5]'%SELECT THE SENSOR SET ga
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_ga_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_ga_stoch=load(c_out);

sensor_sele=[1 3]'%SELECT THE SENSOR SET ig
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_ig_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_ig_det=load(c_out);
%sensor_sele=[1 3]'%SELECT THE SENSOR SET ig
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_ig_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_ig_stoch=load(c_out);

sensor_sele=[3 4]'%SELECT THE SENSOR SET gv
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_gv_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_gv_det=load(c_out);
%sensor_sele=[3 4]'%SELECT THE SENSOR SET gv
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_gv_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_gv_stoch=load(c_out);

sensor_sele=[3]'%SELECT THE SENSOR SET g
sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_det.txt'];
contr_in_g_det=load(c_in);
c_out=[sensors_selected '_c_out_det.txt'];
contr_out_g_det=load(c_out);
%sensor_sele=[3]'%SELECT THE SENSOR SET g
%sensor_set=sensors_name(sensor_sele',:);%Choose the sensor names
%sensors_selected=sensor_set';
c_in=[sensors_selected '_c_in_stoch.txt'];
contr_in_g_stoch=load(c_in);
c_out=[sensors_selected '_c_out_stoch.txt'];
contr_out_g_stoch=load(c_out);



zeros(size(contr_in_igva_det,1),1);


sensor_data_in=[contr_in_igva_det; %igva det
            [zeros(size(contr_in_igva_det,1),1) contr_in_gva_det];...%gva det
            [contr_in_iga_det(:,1:2) zeros(size(contr_in_igva_det,1),1) contr_in_iga_det(:,3)];...%iga det
            [contr_in_igv_det zeros(size(contr_in_igva_det,1),1)];...%igv det
            [zeros(size(contr_in_igva_det,1),1) contr_in_ga_det(:,1) zeros(size(contr_in_igva_det,1),1) contr_in_ga_det(:,2)];...%ga det
            [contr_in_ig_det zeros(size(contr_in_igva_det,1),2)];...%ig det
            [zeros(size(contr_in_igva_det,1),1) contr_in_gv_det zeros(size(contr_in_igva_det,1),1)];...%gv det
            [zeros(size(contr_in_igva_det,1),1) contr_in_g_det zeros(size(contr_in_igva_det,1),2)];...%g det
            contr_in_igva_stoch; %igva stoch
            [zeros(size(contr_in_igva_stoch,1),1) contr_in_gva_stoch];...%gva stoch
            [contr_in_iga_stoch(:,1:2) zeros(size(contr_in_igva_stoch,1),1) contr_in_iga_stoch(:,3)];...%iga stoch
            [contr_in_igv_stoch zeros(size(contr_in_igva_stoch,1),1)];...%igv stoch
            [zeros(size(contr_in_igva_stoch,1),1) contr_in_ga_stoch(:,1) zeros(size(contr_in_igva_stoch,1),1) contr_in_ga_stoch(:,2)];...%ga stoch
            [contr_in_ig_stoch zeros(size(contr_in_igva_stoch,1),2)];...%ig stoch
            [zeros(size(contr_in_igva_stoch,1),1) contr_in_gv_stoch zeros(size(contr_in_igva_stoch,1),1)];...%gv stoch
            [zeros(size(contr_in_igva_stoch,1),1) contr_in_g_stoch zeros(size(contr_in_igva_stoch,1),2)];...%g stoch
            ];


uc=[contr_out_igva_det;contr_out_gva_det;contr_out_iga_det;contr_out_igv_det;...
    contr_out_ga_det;contr_out_ig_det;contr_out_gv_det;contr_out_g_det;...
    contr_out_igva_stoch;contr_out_gva_stoch;contr_out_iga_stoch;contr_out_igv_stoch;...
    contr_out_ga_stoch;contr_out_ig_stoch;contr_out_gv_stoch;contr_out_g_stoch;];

filter_in=[uc sensor_data_in];
filter_out=[sensor_data_in];

% save filter_in.mat filter_in -ascii -double
% save filter_out.mat filter_out -ascii -double

nfuzzy_filter=[filter_in filter_out];

size(filter_in)
size(filter_out)
%s = struct('in1',cntrlr_in(:,1),'in2',cntrlr_in(:,2),'in3',cntrlr_in(:,3),'out',cntrlr_out)


%sz=size(cntrlr)