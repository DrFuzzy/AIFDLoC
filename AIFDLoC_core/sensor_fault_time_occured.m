function [i_fault_time v_fault_time a_fault_time]=sensor_fault_time_occured(i_fault_flag,v_fault_flag,a_fault_flag,sim_time)

%for the current sensor
[index_i value_i]=find(i_fault_flag(2:end)==1);
if ~isempty(index_i)
[index_i2 value_i2]=min(index_i);
i_fault_time=sim_time(index_i2+1);
else i_fault_time=inf;
end

%for the velocity sensor
[index_v value_v]=find(v_fault_flag(2:end)==1);
if ~isempty(index_v)
[index_v2 value_v2]=min(index_v);
v_fault_time=sim_time(index_v2+1);
else v_fault_time=inf;
end

%for the acceleeration sensor
[index_a value_a]=find(a_fault_flag(2:end)==1);
if ~isempty(index_a)
[index_a2 value_a2]=min(index_a);
a_fault_time=sim_time(index_a2+1);
else a_fault_time=inf;
end
