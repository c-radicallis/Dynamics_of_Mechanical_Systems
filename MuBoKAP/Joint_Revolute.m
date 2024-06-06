function [Phi,Jac,niu,gamma] = ...
         Joint_Revolute (Phi,Jac,niu,gamma,k)
%
%Summary: This function controls the construction of vectors
%         and matrices to build the kinematic equations for
%         a revolute joint.
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
%Shared:  Nline   - Number of line for next constraint
%         Body    - Body information
%         Jnt.Revolute - Revolute joint information
%         Parameter    - Parameters (size, tolerance ...)
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
    Nline                = Nline + 1;
    Jnt.Revolute(k).i    = H(Nline,1);
    Jnt.Revolute(k).j    = H(Nline,2);
    Jnt.Revolute(k).spPi = H(Nline,3:4)';
    Jnt.Revolute(k).spPj = H(Nline,5:6)';
    %
    %... Update number of system constraints
    Parameter.NConstraints = Parameter.NConstraints + 2;
    return
end
%
%... No initialization necessary
if Flag.InitData == 1; return; end
%%
%... Line numbers of next constraints
i1      = Nline;
i2      = i1 + 1;
Nline   = Nline + 2;
%%
%... Contribution to rhs of velocity equations is null
if Flag.Velocity == 1
    niu(i1:i2,1) = 0.0;
end
%%
%... Initialize variables
i = Jnt.Revolute(k).i;
j = Jnt.Revolute(k).j;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i2,1) = Body(i).r+Body(i).A*Jnt.Revolute(k).spPi- ...
                   Body(j).r-Body(j).A*Jnt.Revolute(k).spPj;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i2,j1:j2) = [ eye(2), Body(i).B*Jnt.Revolute(k).spPi];
    Jac(i1:i2,j3:j4) = [-eye(2),-Body(j).B*Jnt.Revolute(k).spPj];
end
%%
%... Assemble rhs of Acceleration Equations 
if Flag.Acceleration == 1
    gamma(i1:i2,1) = Body(i).A*Jnt.Revolute(k).spPi*Body(i).thetad^2 - ...
                     Body(j).A*Jnt.Revolute(k).spPj*Body(j).thetad^2;
end
%
%... Finish function Joint_Revolute
end
    