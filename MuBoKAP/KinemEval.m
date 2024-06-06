function [ Phi,Jac,niu,gamma ] = KinemEval(time,q,qd)
%
%Summary: This function controls the construction of vectors
%         and matrices to solve the kinematic analysis, i.e.,
%         vector of constraint equations, Jacobian matrix,
%         rhs of velocity and acceleration equations. 
%         Furthermore, it transfers variables from global 
%         arrays to local storage and builds matrices A and B
%
%Input:   time - Time at which positions are evaluated
%         q    - System positions
%         qd   - System velocities
%
%Output:  Phi   - Vector with the kinematic constraints
%         Jac   - Jacobian matrix
%         niu   - r.h.s of the velocity equations
%         gamma - r.h.s. of the acceleration equations
%
%Shared:  Flag      - Flags for the analysis type
%         Parameter - Parameters (size, tolerance ...)
%         Jnt       - Kinematic Joints data
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access global memory
global Flag Parameter Jnt Nline
%%
%... Initialize vectors and Jacobian matrix
if (Flag.ReadInput==1 || Flag.InitData==1)
    Phi = []; Jac = []; niu = Phi; gamma = Phi;
else
    Phi = zeros(Parameter.NConstraints,1);
    Jac = zeros(Parameter.NConstraints,Parameter.NCoordinates);
    niu = Phi; gamma = Phi; Nline = 1;
end
%%
%... Transfer coordinates from global to local storage
for i = 1:Parameter.NBody
    [q]                 = BodyData(i,q,qd,[]);
end
if (Flag.InitData==1); Phi = q; end
%
%... Contributions by Revolute Joints
for k = 1:Jnt.NRevolute
    [Phi,Jac,niu,gamma] = Joint_Revolute (Phi,Jac,niu,gamma,k);
end
%
%... Contributions by Translation Joints
for k = 1:Jnt.NTranslation
    [Phi,Jac,niu,gamma] = Joint_Translation(Phi,Jac,niu,gamma,k);
end
%
%... Contributions by Revolute-Revolute Joints
for k = 1:Jnt.NRevRev
    [Phi,Jac,niu,gamma] = Joint_RevRev (Phi,Jac,niu,gamma,k);
end
%
%... Contributions by Translation-Revolute Joints
for k = 1:Jnt.NTraRev
    [Phi,Jac,niu,gamma] = Joint_TraRev (Phi,Jac,niu,gamma,k);
end
%
%... Contributions by Rigid Joints
for k = 1:Jnt.NRigid
    [Phi,Jac,niu,gamma] = Joint_Rigid (Phi,Jac,niu,gamma,k);
end
%
%... Contributions by Simple Joints
for k = 1:Jnt.NSimple
    [Phi,Jac,niu,gamma] = Joint_Simple (Phi,Jac,niu,gamma,k);
end
%
%... Contributions by Driving constraints
for k = 1:Jnt.NDriver
    [Phi,Jac,niu,gamma] = Joint_Driver(Phi,Jac,niu,gamma,k,time);
end
%%
%... Finalize function KinemEval
end


