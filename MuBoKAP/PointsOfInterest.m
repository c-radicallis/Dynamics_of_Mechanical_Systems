function [] = PointsOfInterest(k,n)
%
%Summary: This function controls all actions regarding data 
%         for the kinematics of Points of Interest (PoI).
%
%Input:   k            - Point of interest number
%         n            - Time step number
%
%Shared:  Body   - Data for Rigid Bodies in the system
%         Pts    - Information on points of interest 
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access memory
global H Body Flag Nline Time Pts
%%
%... Store data for PoI
if     Flag.ReadInput == 1
    Nline           = Nline + 1; 
    Pts.Int(k).i    = H(Nline,1);
    Pts.Int(k).spPi = H(Nline,2:3)';
end
%%
%... Initialize data for the driver
if Flag.InitData == 1
    Pts.Int(k).q(:,:) = zeros(2,Time.Ntime);
    Pts.Int(k).qd     = Pts.Int(k).q;
    Pts.Int(k).qdd    = Pts.Int(k).q;
end
%%
%... Build the kinematics of the PoI
if Flag.PostAccelera == 1
    i  = Pts.Int(k).i;
%
%... Calculate and store the kinematics of the PoI
    Pts.Int(k).q(1:2,n)   = Body(i).r   + ...
                            Body(i).A*Pts.Int(k).spPi;
    Pts.Int(k).qd(1:2,n)  = Body(i).rd  + ...
                            Body(i).B*Pts.Int(k).spPi* ...
                            Body(i).thetad;
    Pts.Int(k).qdd(1:2,n) = Body(i).rdd + ...
                            Body(i).B*Pts.Int(k).spPi* ...
                            Body(i).thetadd - ...
                            Body(i).A*Pts.Int(k).spPi*...
                            Body(i).thetad^2;
end
%%
%... Finish function PointsOfInterest
end
