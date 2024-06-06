function [q] = BodyData(i,q,qd,qdd)
%
%Summary: This function controls all actions regarding data
%         for the rigid bodies, including its input, update
%         and storage into local memory.
%Input:   i      - Body number
%         q      - Vector with the current system positions
%         qd     - Vector with current velocities
%         qdd    - Vector with current accelerations
%
%Shared:  Body   - All data for each Rigid Body in the system
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%... Access memory
global H Body Flag Nline Parameter
%% 
%... Store initial positions for Rigid Bodies
if Flag.ReadInput == 1
    Nline           = Nline + 1; 
    Body(i).r       = H(Nline,1:2)';
    Body(i).theta   = H(Nline,3);
    %
    %... Update the number of system coordinates
    Parameter.NCoordinates = Parameter.NCoordinates + 3;
    return
end
%%
%... Set pointers
i1 = 3*i - 2;
i2 = i1 + 1;
i3 = i2 + 1;
%
%... Transfer coordinates from global to local storage
if Flag.InitData == 1
    q(i1:i2,1) = Body(i).r;
    q(i3:i3,1) = Body(i).theta;
    %
    %... Transformation matrix
    cost          = cos(Body(i).theta);
    sint          = sin(Body(i).theta);
    Body(i).A     = [ cost -sint; sint  cost];
end
%%
%... Transfer coordinates from global to local storage
if Flag.Transfer == 1
    Body(i).r     = q(i1:i2,1);
    Body(i).theta = q(i3:i3,1);
    %
    %... Transformation and B matrices
    cost          = cos(Body(i).theta);
    sint          = sin(Body(i).theta);
    Body(i).A     = [ cost -sint; sint  cost];
    Body(i).B     = [-sint -cost; cost -sint];
end
%%
%... Transfer velocities from global to local storage
if Flag.Acceleration == 1
    Body(i).rd     = qd(i1:i2,1);
    Body(i).thetad = qd(i3:i3,1);
%
%... Transfer accelerations from global to local storage
elseif Flag.PostAccelera == 1
    Body(i).rdd     = qdd(i1:i2,1);
    Body(i).thetadd = qdd(i3:i3,1);
end
%% 
%... Finish function BodyData
end