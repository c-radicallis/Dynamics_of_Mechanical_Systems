function [Phi,Jac,niu,gamma] = ...
         Joint_Simple (Phi,Jac,niu,gamma,k)
%
%Summary: This function controls the construction of vectors
%         and matrices to build the kinematic equations for
%         a simple kinematic constraint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%         k       - Number of the simple kinematic constraint
%
%Output:  Phi     - Vector with kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%
%Shared:  Nline   - Number of line for next constraint
%         Body    - Body information
%         Jnt.Simple - Simple constraint information
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
    Jnt.Simple(k).i    = H(Nline,1);
    Jnt.Simple(k).type = H(Nline,2);
    Jnt.Simple(k).z0   = H(Nline,3);
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
i    = Jnt.Simple(k).i;
type = Jnt.Simple(k).type;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    switch type
        case 1
            aux = Body(i).r(1,1) - Jnt.Simple(k).z0;
        case 2
            aux = Body(i).r(2,1) - Jnt.Simple(k).z0;
        case 3
            aux = Body(i).theta  - Jnt.Simple(k).z0;
    end
    Phi(i1:i1,1) = aux;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*(i-1) + type;
    Jac(i1:i1,j1:j1) = 1.0;
end
%%
%... Contribution to  rhs of velocity equations is null
if Flag.Velocity == 1
    niu(i1:i1,1) = 0.0;
end
%%
%... Assemble rhs of Acceleration Equations 
if Flag.Acceleration == 1
    gamma(i1:i1,1) = 0.0;
end
%
%... Finish function Joint_Simple
end
    