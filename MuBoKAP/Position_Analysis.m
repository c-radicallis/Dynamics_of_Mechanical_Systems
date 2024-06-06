function [ q ] = Position_Analysis(time,q)
%
%Summary: This function controls the kinematic position analysis.
%         The Newton-Raphson method is used.
%
%Input:   time - Time at which positions are evaluated
%         q    - Estimate for positions
%
%Output:  q    - Corrected positions
%
%Shared:  Flag - Flags for the analysis type
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access the global memory
global Flag
%
%... Set the analysis flags
Flag.Transfer = 1;
Flag.Position = 1;
Flag.Jacobian = 1;
%
%... Solve the nonlinear system of equations w/ Newton-Raphson
[ q,Warning ] = Newton_Raphson_Method(@KinemEval,q,time);
%
%... Check if maximum number of iterations is reached and warn
if Warning.Flag==0
    String = ['@ Time = ' num2str(time)];
    disp(Warning.String)
    disp(String)
end
%
%... Reset analysis flags
Flag.Transfer = 0;
Flag.Position = 0;
Flag.Jacobian = 0;
%%
%... Finalize function Position_Analysis
end