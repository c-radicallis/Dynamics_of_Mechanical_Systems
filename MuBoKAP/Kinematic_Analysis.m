function [ q,qd,qdd ] = Kinematic_Analysis( q0 )
%
%Summary: This function controls the kinematic analysis for
%         a specified duration and selected time-step.
%
%Input:  q0        - Estimate for the initial positions
%
%Output: q         - Position history
%        qd        - Velocity history
%        qdd       - Acceleration history
%
%Shared: Time      - Analysis time period and time step
%        Parameter - Parameters (size, tolerance ...)
%        Body      - Rigid bodies kinematics and matrices
%        Pts       - Information on points of interest 
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Acess the global memory
global Flag Time Parameter Pts
%%
%... Create the workspaces
q   = zeros(Parameter.NCoordinates,Time.Ntime);
qd  = q;
qdd = qd;
%%
%... For each time step perform a kinematic analysis
for k = 1:Time.Ntime
    t      = Time.time(k);
    q(:,k) = q0;
    %
    %... Position analysis
    [q(:,k)]      = Position_Analysis(t,q(:,k));
    %
    %... Velocity analysis
    [qd(:,k),Jac] = Velocity_Analysis(t,q(:,k));
    %
    %... Acceleration analysis
    [qdd(:,k)]    = Acceleration_Analysis(t,q(:,k),qd(:,k),Jac);
    %
    %... Build kinematics of Points of Interest
    Post_Acceleration(k,qdd(:,k));
    %
    %... Previous positions are estimates for next time step
    q0     = q(:,k);
end
%%
%... Finalize function Kinematic_Analysis
end
