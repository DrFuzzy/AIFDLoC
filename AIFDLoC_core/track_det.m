function [t,v_out,Vo] = track_det(j_max,a_max,grad_a,V,Tlm,Ts)
%function [t,v_out,Vo] = track_det(j_max,a_max,grad_a,V,Tlm,Ts)
% inputs - j_max: jerk max (m/s^3)
%          a_max: acceleration maximum (m/s^2)
%          grad_a: gradient of track (%)
%          V: vehicle speed (m/s) (optional)
%          Tlm: max track length (m) (optional)
%          Ts: sampling time for simulations (s) (optional)
%
% outputs - t: time vector (s) 
%           v_out: track profiles with time vector 
%           columnwise, i.e. v_out = [jerk_prof accel_prof vel_prof position_prof]
%                                       m/s^3     m/s^2      m/s       m
%           V: vehicle speed (m/s)
%
%
% V,Tlm and Ts optional, if not supplied a vehicle speed of 15m/s, 
% fixed length of 200m test track and 0.001s sampling time are used. 
% To plot the profiles relative to the track length simply use
%
%             plot(t*V,v_out)
%
% where V is the vehicle speed used. 
% 
%
% ACZ, 20/8/2007
%

%---- for checking
% j_max = 1;
% a_max = 0.5;
% grad_a = 10;
% V = 15;
% Tlm = 200;
% Ts=0.01;
%----

% check for input arguments
if nargin == 5
    Ts = 0.001;
else if nargin == 4
        Ts=0.001;
        Tlm=200;
    elseif nargin == 3
        Ts=0.001;
        Tlm=200;
        V=15;
    end
end

% find transition for acceleration profile (assuming trapezoidal prof)
dt1 = a_max/j_max;

% find max track velocity for constant gradient
v_max = grad_a*V/100;

% find time range of constant velocity
dt2 = v_max/a_max-dt1;

if dt2<0
    error('Negative time range for constant acceleration???')
end

% generate track profiles
t = [0:Ts:Tlm/V]; % time vector
    
y1 = zeros(1,length(t)); % initialise jerk profile 

% some interpolation now...
[m1,i1] = min(abs(t-dt1));
if dt2>0
    [m2,i2] = min(abs(t-(dt1+dt2)));
    [m3,i3] = min(abs(t-(2*dt1+dt2)));
    y1(1:i1)=j_max;
    y1(i1+1:i2-1)=[0];
    y1(i2:i3)=-j_max; % jerk prof defined
else
    [m3,i3] = min(abs(t-(2*dt1)));
    y1(1:i1)=j_max;
    y1(i1+1:i3+1)=-j_max;
end
y2=cumsum(y1)*(t(2)-t(1)); % acc profile
y3=cumsum(y2)*(t(2)-t(1)); % vel profile
y4=cumsum(y3)*(t(2)-t(1)); % posn profile
v_out = [y1' y2' y3' y4'];
Vo=V;