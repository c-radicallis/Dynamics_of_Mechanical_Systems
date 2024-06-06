function [] = Post_Acceleration(k,qdd)
%
%Summary: This function controls the storage of accelerations
%         to body information and builds the kinematics of the
%         Points of Interest.
%
%Input:   k         - Time step number
%         qd        - System accelerations
%
%Shared:  Flag      - Flags for the analysis type
%         Parameter - Parameters (size, tolerance ...)
%         Pts       - Information on points of interest 
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access the global memory
global Flag Parameter Pts
%
%%
%... Transfer accelerations from global to local storage
Flag.PostAccelera = 1;
for i = 1:Parameter.NBody
    [~] = BodyData(i,[],[],qdd);
end
%
%... Build the kinematics of Points Of Interest
for i = 1:Pts.NPointsOfInt
    PointsOfInterest(i,k);
end
Flag.PostAccelera = 0;
%%
%... Finalize function Post_Acceleration
end