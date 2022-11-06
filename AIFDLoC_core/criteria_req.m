function [chromosome_cr measurements_cr sensors_cr controller_selected_index] = criteria_req (chromosome,measurements,V_var,M_obj,...
    criteria_selected,min_max_criteria);


criteria_num=size(min_max_criteria,2);%check how many criteria are required
solution_index=find(measurements(:,14)==0);%check how many controller satisfy the Omega

if isempty(solution_index)%If no controller exist to satisfy Omega stop
    %display('No controller found to satisfy constaints for this sensor set.');
    %display('The controller that results to minimum constraints violation is selected.');
    [dammy_tmp controller_selected_index]=min(measurements(:,14));
    measurements_cr=measurements(controller_selected_index,1:end);
    chromosome_cr=chromosome(controller_selected_index,:);
    sensors_cr=measurements(controller_selected_index,end);
    controller_selected_index=1;
    return
end

%Select the solutions that satisfy the Omega
chromosome=chromosome(solution_index,:);
measurements=[measurements(solution_index,:) solution_index];

criteria_array=measurements;
%Select the controller with 1 criterion
if criteria_num==1
    cr1_min=min_max_criteria(1,1);cr1_max=min_max_criteria(2,1);
    criteria_index=find(cr1_min<criteria_array(:,criteria_selected)...
        & criteria_array(:,criteria_selected)<cr1_max)';
    if isempty(criteria_index)
        %display('No solution exists with such criteria.')
        %display('Controller selection is defined by the selected minimum objective.')
        [dammy_tmp controller_selected_index]=min(measurements(:,min_max_criteria(3,2)));
        measurements_cr=measurements(controller_selected_index,1:end);
        chromosome_cr=chromosome(controller_selected_index,:);
        sensors_cr=measurements(controller_selected_index,end);
        controller_selected_index=1;
        return
    end
      measurements_cr=measurements(criteria_index,1:end);
      chromosome_cr=chromosome(criteria_index,:);
      sensors_cr=measurements(criteria_index,end); 
end

%Select the controller with 2 criteria
if criteria_num==2
    cr1_min=min_max_criteria(1,1);cr1_max=min_max_criteria(2,1);
    cr2_min=min_max_criteria(1,2);cr2_max=min_max_criteria(2,2);
    criteria_index=find(cr1_min<criteria_array(:,criteria_selected(1))...
        & criteria_array(:,criteria_selected(1))<cr1_max &...
        cr2_min<criteria_array(:,criteria_selected(2))...
        & criteria_array(:,criteria_selected(2))<cr2_max)';
    if isempty(criteria_index)
        %display('No solution exists with such criteria.')
        %display('Controller selection is defined by the selected minimum objective.')
        [dammy_tmp controller_selected_index]=min(measurements(:,min_max_criteria(3,2)));
        measurements_cr=measurements(controller_selected_index,1:end);
        chromosome_cr=chromosome(controller_selected_index,:);
        sensors_cr=measurements(controller_selected_index,end);
        controller_selected_index=1;
        return
    end
      measurements_cr=measurements(criteria_index,1:end);
      chromosome_cr=chromosome(criteria_index,:);
      sensors_cr=measurements(criteria_index,end); 
end

%Select the controller with 3 criteria
if criteria_num==3
    cr1_min=min_max_criteria(1,1);cr1_max=min_max_criteria(2,1);
    cr2_min=min_max_criteria(1,2);cr2_max=min_max_criteria(2,2);
    cr3_min=min_max_criteria(1,3);cr3_max=min_max_criteria(2,3);
    criteria_index=find(cr1_min<criteria_array(:,criteria_selected(1))...
        & criteria_array(:,criteria_selected(1))<cr1_max &...
        cr2_min<criteria_array(:,criteria_selected(2))...
        & criteria_array(:,criteria_selected(2))<cr2_max&...
        cr3_min<criteria_array(:,criteria_selected(3))...
        & criteria_array(:,criteria_selected(3))<cr3_max)' ;
    if isempty(criteria_index)
        %display('No solution exists with such criteria.')
        %display('Controller selection is defined by the selected minimum objective.')
        [dammy_tmp controller_selected_index]=min(measurements(:,min_max_criteria(3,2)));
        measurements_cr=measurements(controller_selected_index,1:end);
        chromosome_cr=chromosome(controller_selected_index,:);
        sensors_cr=measurements(controller_selected_index,end);
        controller_selected_index=1;
        return
    end
      measurements_cr=measurements(criteria_index,1:end);
      chromosome_cr=chromosome(criteria_index,:);
      sensors_cr=measurements(criteria_index,end); 
end

%Select the controller with 4 criteria
if criteria_num==4
    cr1_min=min_max_criteria(1,1);cr1_max=min_max_criteria(2,1);
    cr2_min=min_max_criteria(1,2);cr2_max=min_max_criteria(2,2);
    cr3_min=min_max_criteria(1,3);cr3_max=min_max_criteria(2,3);
    cr4_min=min_max_criteria(1,4);cr4_max=min_max_criteria(2,4);
    criteria_index=find(cr1_min<criteria_array(:,criteria_selected(1))...
        & criteria_array(:,criteria_selected(1))<cr1_max &...
        cr2_min<criteria_array(:,criteria_selected(2))...
        & criteria_array(:,criteria_selected(2))<cr2_max&...
        cr3_min<criteria_array(:,criteria_selected(3))...
        & criteria_array(:,criteria_selected(3))<cr3_max...
        & cr4_min<criteria_array(:,criteria_selected(4))...
        & criteria_array(:,criteria_selected(4))<cr4_max)';
    if isempty(criteria_index)
        %display('No solution exists with such criteria.')
        %display('Controller selection is defined by the selected minimum objective.')
        [dammy_tmp controller_selected_index]=min(measurements(:,min_max_criteria(3,2)));
        measurements_cr=measurements(controller_selected_index,1:end);
        chromosome_cr=chromosome(controller_selected_index,:);
        sensors_cr=measurements(controller_selected_index,end);
        controller_selected_index=1;
        return
    end
      measurements_cr=measurements(criteria_index,1:end);
      chromosome_cr=chromosome(criteria_index,:);
      sensors_cr=measurements(criteria_index,end); 
end

if size(criteria_selected,2)~=size(min_max_criteria,2);%Check if a minimum or maximum criteria is required
    crt_selected=criteria_selected(end);
if min_max_criteria(3,1)==0%Check if a minimum criteria is requested
   [dummy controller_selected_index]=min(measurements_cr(:,crt_selected));
else %if a maximum criteria is selected
   [dummy controller_selected_index]=max(measurements_cr(:,crt_selected));
end
else
  controller_selected_index=[];
end




