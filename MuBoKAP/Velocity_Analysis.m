function [qd,Jac] = Velocity_Analysis(time,q)
%
%Summary: This function controls the kinematic velocity 
%         analysis. The method used is selected by Matlab
%
%Input:   time - Time for velocity evaluation
%         q    - System positions
%
%Output:  qd   - System velocities
%         Jac  - Jacobian matrix
%
%Shared:  Flag - Flags for the analysis type
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access the global memory
global Flag
%%
%... Evaluate r.h.s of velocity equations
Flag.Transfer = 1;
Flag.Velocity = 1;
Flag.Jacobian = 1;
[~,Jac,niu,~] = KinemEval(time,q,[]);
%
%... Evaluate the velocities
qd            = Jac\niu;
%
%... Reset analysis flags
Flag.Transfer = 0;
Flag.Velocity = 0;
Flag.Jacobian = 0;
%%
%... Finalize function Velocity_Analysis
end