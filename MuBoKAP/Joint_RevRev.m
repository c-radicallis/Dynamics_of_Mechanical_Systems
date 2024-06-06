function [Phi,Jac,niu,gamma] = ...
         Joint_RevRev (Phi,Jac,niu,gamma,k)
%
%Summary: This function controls the construction of vectors
%         and matrices to build the kinematic equations for
%         a composite revolute-revolute joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%         k       - Number of the Revolute-Revolute joint
%
%Output:  Phi     - Vector with kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%
%Shared:  Nline   - Number of line for next constraint
%         Body    - Body information
%         Jnt.RevRev - Revolute-Revolute joint information
%         Parameter  - Parameters (size, tolerance ...)
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
    Jnt.RevRev(k).i    = H(Nline,1);
    Jnt.RevRev(k).j    = H(Nline,2);
    Jnt.RevRev(k).l0   = H(Nline,3);
    Jnt.RevRev(k).spPi = H(Nline,4:5)';
    Jnt.RevRev(k).spPj = H(Nline,6:7)';
    %
    %... Initialize data for this joint
    Jnt.RevRev(k).l2   = Jnt.RevRev(k).l0^2;
    %
    %... Update number of system constraints
    Parameter.NConstraints = Parameter.NConstraints + 1;
    return
end
%
%... No initialization necessary
if Flag.InitData == 1; return; end
%%
%... Line numbers of next constraints
i1      = Nline;
Nline   = Nline + 1;
%%
%... Initialize variables
i = Jnt.RevRev(k).i;
j = Jnt.RevRev(k).j;
%
d = Body(i).r + Body(i).A*Jnt.RevRev(k).spPi - ...
    Body(j).r - Body(j).A*Jnt.RevRev(k).spPj;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i1,1) = d'*d - Jnt.RevRev(k).l2;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i1,j1:j2) = 2*[ d',  d'*Body(i).B*Jnt.RevRev(k).spPi];
    Jac(i1:i1,j3:j4) = 2*[-d', -d'*Body(j).B*Jnt.RevRev(k).spPj];
end
%%
%... Contribution to rhs of velocity equations is null
if Flag.Velocity == 1
    niu(i1:i1,1) = 0.0;
end
%%
%... Assemble rhs of Acceleration Equations 
if Flag.Acceleration == 1
    tid = Body(i).thetad;
    tjd = Body(j).thetad;
%
    dd  = Body(i).rd + Body(i).B*Jnt.RevRev(k).spPi*tid - ...
          Body(j).rd - Body(j).B*Jnt.RevRev(k).spPj*tjd;
    gamma(i1:i1,1) = -dd'*dd + d'*(Body(i).A*Jnt.RevRev(k).spPi*tid^2 - ...
                                   Body(j).A*Jnt.RevRev(k).spPj*tjd^2);
end
%
%... Finish function Joint_RevRev
end
    