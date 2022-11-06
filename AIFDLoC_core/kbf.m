function [Kf,Pf,Ef] = kbf(A,B,G,C,D,W,V)
% function [Kf,Pf,Ef] = kbf(A,B,G,C,D,W,V)
%
% Continuous time Kalman filter solution. Provide A,B,G,C,D based
% upon the following state space expression:
%
%        .
%        x = A x + B u + G w, (w the process noise)
%
%        y = C x + D u +   v, (v the sensor noise)
%
% W process noise covariance matrix, V sensor noise
% covariance matrix. Note that G is the disturbance (process noise) matrix.
% 
% Function returns:- 
% Kf : optimal kalman gain
% Pf : Riccatti solution covariance matrix 
% Ef : eig(A-Kf*C)
%
% Uses conceptual algorithm for the solution of the ARE.
% 
% Argyrios Zolotas, 22/3/2004, a.zolotas@ieee.org
% Systems and Control Group, Loughborough University.
% 

%//////////////////////////////////////////////////////////////////
% SETUP // DO NOT CHANGE THIS PART OF CODE ////////////////////////
%//////////////////////////////////////////////////////////////////
% define Hamiltonian for estimation problem
H_EST = [       A'      -C'*inv(V)*C
           -G*W*G'                -A];
% 
% call Estimation Riccati based on Hamiltonian
[Pf,FLAGf] = solveEstRiccati(H_EST,length(A));
if FLAGf == 0
    error('No solution exists for this ESTIMATION problem! Modify problem and solve again.')
end
% 
% optimal kalman gain
Kf = Pf*C'*inv(V);
% covariance solution
% Pf;
% expected estimator closed-loop eig-values:-
Ef = sort(eig(A-Kf*C));
%//////////////////////////////////////////////////////////////////
% SETUP // DO NOT CHANGE THIS PART OF CODE ////////////////////////
%//////////////////////////////////////////////////////////////////

% Internal functions :-

%==================================================================
% Solution for Kalman Filter == DO NOT CHANGE THIS PART OF CODE ===
%==================================================================
function [P,INFO]=solveEstRiccati(HAMILTONIAN,N)
INFO=1;
[D,V]=eig(HAMILTONIAN);
S=eig(HAMILTONIAN);
p=0;

for i=1:2*N
    if S(i)<0 
   p=p+1;
        EIGVAL(p)=S(i);
       vector(p)=i;    
   end
end

SSS=p;

if p<N
    P=[];
    INFO=0;
end

if p>=N
for i=1:N
    EIGVECTOR(:,i)=D(:,vector(i));
end

for i=1:N
    T1(i,:)=EIGVECTOR(i,:);
    T2(i,:)=EIGVECTOR(i+N,:);
end
T=[T1;T2];
P=T2*inv(T1); % form P
P = real(P); % remove negligible imag parts due to numerical deficiency
INFO=1;
end
%==================================================================
% Solution for Kalman Filter == DO NOT CHANGE THIS PART OF CODE ===
%==================================================================

% end of kbf.m ----------------------------------------------------