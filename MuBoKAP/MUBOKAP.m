clear all
%
%MUBOKAP - Multibody Kinematic Analysis Program
%
%Summary: This program preforms the kinematic analysis of
%         a general planar mechanism for which the model is
%         supplied by loading a workspace previously generated.
%         The results are loaded to a new workspace for
%         post-processing
%
%Shared:  Time        - Analysis time period and time step
%         Parameter   - Parameters (size, tolerance ...)
%         Body        - Rigid bodies kinematics and matrices 
%         Jnt         - Data for kinematic joints and drivers 
%         Pts         - Information on points of interest 
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access global memory
global Time Parameter Pts Body Jnt
%%
%... Query for the input file with the model
[Filename,Path] = uigetfile('*.mat','Select Model Data file');
%
%... Load the model workspace
load([Path Filename]);
CPUStart = cputime;
%
%... Perform the Kinematic Analysis
[q,qd,qdd] = Kinematic_Analysis(q0);
%
%... Report analysis CPU time
CPUTime = cputime-CPUStart;
disp(['CPU Time = ' num2str(CPUTime)])
%%
%... Store the workspace in file 'model'.mat
[~,Name,ext] = fileparts(Filename);
save([Path Name '_Results.mat']);
%
%... Terminate the Kinematic Analysis Program
