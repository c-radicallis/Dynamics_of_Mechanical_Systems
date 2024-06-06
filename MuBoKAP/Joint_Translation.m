function [Phi,Jac,niu,gamma] = ...
         Joint_Translation (Phi,Jac,niu,gamma,k)
%
%Summary: This function controls the construction of vectors
%         and matrices to build the kinematic equations for
%         a translation joint.
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
%Shared:  Nline   - Number of line for the next constraint
%         Body    - Body information
%         Jnt.Translation - Translation joint information
%         Parameter       - Parameters (size, tolerance ...)
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
    Nline                   = Nline + 1;
    Jnt.Translation(k).i    = H(Nline,1);
    Jnt.Translation(k).j    = H(Nline,2);
    Jnt.Translation(k).spPi = H(Nline,3:4)';
    Jnt.Translation(k).spQi = H(Nline,5:6)';
    Jnt.Translation(k).spPj = H(Nline,7:8)';
    Jnt.Translation(k).spQj = H(Nline,9:10)';
    %
    %... Initialize data for this joint
    Jnt.Translation(k).spi  = Jnt.Translation(k).spPi - ...
                              Jnt.Translation(k).spQi;
    spj = Jnt.Translation(k).spPj - Jnt.Translation(k).spQj;
    Jnt.Translation(k).hpj  = A90*(spj/norm(spj));
    %
    %... Update number of system constraints
    Parameter.NConstraints = Parameter.NConstraints + 2;
    return
end
%
%... No initialization necessary
if Flag.InitData == 1; return; end
%%
%... Line numbers of the constraint equations & Pointer of next constraints
i1    = Nline;
i2    = i1 + 1;
Nline = Nline + 2;
%%
%... Initialize variables
i   = Jnt.Translation(k).i;
j   = Jnt.Translation(k).j;
%
si  = Body(i).A*Jnt.Translation(k).spi;
hj  = Body(j).A*Jnt.Translation(k).hpj;
sPi = Body(i).A*Jnt.Translation(k).spPi;
sPj = Body(j).A*Jnt.Translation(k).spPj;
sij = Body(i).r + sPi - Body(j).r - sPj;
%%
%... Assemble position constraint equations
if Flag.Position == 1 
    Phi(i1:i2,1) = [hj'*si; ...
                    hj'*sij];
    if (Flag.Jacobian == 0 && Flag.Acceleration == 0); return; end
end
%%
%... Initialize variables
Bspi  = Body(i).B*Jnt.Translation(k).spi;
Bhpj  = Body(j).B*Jnt.Translation(k).hpj;
BspPi = Body(i).B*Jnt.Translation(k).spPi;
BspPj = Body(j).B*Jnt.Translation(k).spPj;
%%
%... Assemble Jacobian matrix 
if Flag.Jacobian == 1
    j1 = 3*i - 2;
    j2 = j1 + 2;
    j3 = 3*j - 2; 
    j4 = j3 + 2;
%
    Jac(i1:i2,j1:j2) = [ zeros(1,2), hj'*Bspi; ...
                           hj'     , hj'*BspPi];
    Jac(i1:i2,j3:j4) = [ zeros(1,2), si'*Bhpj;...
                          -hj'     , sij'*Bhpj-hj'*BspPj];
end
%%
%... Contribution to the r.h.s. of the velocity equations is null
if Flag.Velocity == 1
    niu(i1:i2,1) = 0.0;
end
%%
%... Assemble the right hand side of the Acceleration Equations 
if Flag.Acceleration == 1
    tid  = Body(i).thetad;
    tjd  = Body(j).thetad;
    sid  = Body(i).B*Jnt.Translation(k).spi*tid;
    hjd  = Bhpj*tjd;
    sijd = Body(i).rd + BspPi*tid - Body(j).rd - BspPj*tjd;
%
    gamma(i1:i2,1) = [(-sid'*Bhpj*tjd  + si'*hj*tjd^2 - ...
                       hjd'*Bspi*tid  + hj'*si*tid^2); ...
                      (-sijd'*Bhpj*tjd + sij'*hj*tjd^2 -...
                       hjd'*sijd      + hj'*(sPi*tid^2-sPj*tjd^2))];
end
%
%... Finish function Joint_Translation
end
    