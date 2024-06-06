function [Phi,Jac,niu,gamma] = ...
         Joint_Rigid (Phi,Jac,niu,gamma,k)
%
%Summary: This function controls the construction of vectors
%         and matrices to build the kinematic equations for
%         a rigid joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%         k       - Number of the Revolute joint
%
%Output:  Phi     - Vector with kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%
%Shared:  Nline     - Number of line for next constraint
%         Body      - Body information
%         Jnt.Rigid - Rigid joint information
%         Parameter - Parameters (size, tolerance ...)
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access global memory
global H Body Jnt Flag Nline Parameter
%%
%... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.Rigid(k).i     = H(Nline,1);
    Jnt.Rigid(k).j     = H(Nline,2);
    Jnt.Rigid(k).spPi  = H(Nline,3:4)';
    Jnt.Rigid(k).spPj  = H(Nline,5:6)';
    Jnt.Rigid(k).angle = H(Nline,7);
    %
    %... Update number of system constraints
    Parameter.NConstraints = Parameter.NConstraints + 3;
    return
end
%
%... No initialization necessary
if Flag.InitData == 1; return; end
%%
%... Line numbers of next constraints
i1      = Nline;
i2      = i1 + 1;
i3      = i1 + 2;
Nline   = Nline + 3;
%%
%... Initialize variables
i = Jnt.Rigid(k).i;
j = Jnt.Rigid(k).j;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i2,1) = Body(i).r + Body(i).A*Jnt.Rigid(k).spPi - ...
                   Body(j).r - Body(j).A*Jnt.Rigid(k).spPj;
	Phi(i3:i3,1) = Body(i).theta-Body(j).theta-Jnt.Rigid(k).angle;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i2,j1:j2) = [ eye(2), Body(i).B*Jnt.Rigid(k).spPi];
    Jac(i1:i2,j3:j4) = [-eye(2),-Body(j).B*Jnt.Rigid(k).spPj];
    Jac(i3:i3,j1:j2) = [ zeros(1,2), 1];
    Jac(i3:i3,j3:j4) = [ zeros(1,2),-1];
end
%%
%... Contribution to rhs of velocity equations is null
if Flag.Velocity == 1
    niu(i1:i3,1) = 0.0;
end
%%
%... Assemble rhs of Acceleration Equations 
if Flag.Acceleration == 1
    gamma(i1:i2,1) = Body(i).A*Jnt.Rigid(k).spPi*Body(i).thetad^2 - ...
                     Body(j).A*Jnt.Rigid(k).spPj*Body(j).thetad^2;
    gamma(i3:i3,1) = 0;
end
%
%... Finish function Joint_Revolute
end
    