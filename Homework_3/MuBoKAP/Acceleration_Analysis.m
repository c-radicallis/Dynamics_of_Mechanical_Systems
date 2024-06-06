function [ qdd ] = Acceleration_Analysis(time,q,qd,Jac)
%
%Summary: This function controls the kinematic acceleration
%         analysis. The method used is selected by Matlab.
%
%Input:   time      - Time for positions are evaluation
%         q         - System positions
%         qd        - System velocities
%         Jac       - Jacobian matrix
%
%Output:  qdd       - System accelerations
%
%Shared:  Flag      - Flags for the analysis type
%         Parameter - Parameters (size, tolerance ...)
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access the global memory
global Flag Parameter Body
%
%%
%... Evaluate r.h.s of acceleration equations
Flag.Acceleration = 1;
[~,~,~,gamma] = KinemEval(time,q,qd);
%
%... Evaluate the accelerations
qdd = Jac\gamma;
%
%... Reset analysis flags
Flag.Acceleration = 0;
%%
%... Finalize function Acceleration_Analysis
end