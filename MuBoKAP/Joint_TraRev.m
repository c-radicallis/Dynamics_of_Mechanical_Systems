function [Phi,Jac,niu,gamma] = ...
         Joint_TraRev (Phi,Jac,niu,gamma,k)
%
%Summary: This function controls the construction of vectors
%         and matrices to build the kinematic equations for
%         a composite revolute-translation joint.
%
%Input:   Phi     - Vector with the kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%         k       - Number of the Revolute-Translation joint
%
%Output:  Phi     - Vector with kinematic constraints
%         Jac     - Jacobian matrix
%         niu     - rhs of velocity equations
%         gamma   - rhs of acceleration equations
%
%Shared:  Nline   - Number of line for next constraint
%         Body    - Body information
%         Jnt.TraRev - Revolute-Translation joint information
%         Parameter  - Parameters (size, tolerance ...)
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access global memory
global H Body Jnt Flag Nline Parameter
global A90
%%
%... Read input for this joint
if Flag.ReadInput == 1
    Nline              = Nline + 1;
    Jnt.TraRev(k).i    = H(Nline,1);
    Jnt.TraRev(k).j    = H(Nline,2);
    Jnt.TraRev(k).l0   = H(Nline,3);
    Jnt.TraRev(k).spPi = H(Nline,4:5)';
    Jnt.TraRev(k).spPj = H(Nline,6:7)';
    Jnt.TraRev(k).spQj = H(Nline,8:9)';
    %
    %... Initialize data for this joint
    spj = Jnt.TraRev(k).spPj - Jnt.TraRev(k).spQj;
    Jnt.TraRev(k).hpj = A90*(spj/norm(spj));
    %
    %... Update number of system constraints
    Parameter.NConstraints = Parameter.NConstraints + 1;
    return
end
%%
%... Line numbers of the constraint equations & Pointer of next constraints
i1      = Nline;
Nline   = Nline + 1;
%%
%... Initialize variables
i  = Jnt.TraRev(k).i;
j  = Jnt.TraRev(k).j;
%
d  = Body(i).r + Body(i).A*Jnt.TraRev(k).spPi - ...
     Body(j).r - Body(j).A*Jnt.TraRev(k).spPj;
hj = Body(j).A*Jnt.TraRev(k).hpj;
%%
%... Data initialization
if Flag.InitData == 1
    %
    %... Ensure that direction of vector hj is consistent
    if (d'*hj)<0
        Jnt.TraRev(k).hpj =-Jnt.TraRev(k).hpj;
    end
    return
end
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i1,1) = d'*hj-Jnt.TraRev(k).l0;
end
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
    Jac(i1:i1,j1:j2) = [ hj',  hj'*Body(i).B*Jnt.TraRev(k).spPi];
    Jac(i1:i1,j3:j4) = [-hj', -hj'*Body(j).B*Jnt.TraRev(k).spPj + ...
                                d'*Body(j).B*Jnt.TraRev(k).hpj];
end
%%
%... Contribution to the r.h.s. of the velocity equations is null
if Flag.Velocity == 1
    niu(i1:i1,1) = 0.0;
end
%%
%... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    tid = Body(i).thetad;
    tjd = Body(j).thetad;
    dd  = Body(i).rd + Body(i).B*Jnt.TraRev(k).spPi*tid - ...
          Body(j).rd - Body(j).B*Jnt.TraRev(k).spPj*tjd;
    hjd = Body(j).B*Jnt.TraRev(k).hpj*tjd;
    gamma(i1:i1,1) = d'*hj*tjd^2 - 2*hjd'*dd + ...
                     hj'*(Body(i).A*Jnt.TraRev(k).spPi*tid^2 - ...
                          Body(j).A*Jnt.TraRev(k).spPj*tjd^2);
end
%
%... Finish function Joint_TraRev
end
    