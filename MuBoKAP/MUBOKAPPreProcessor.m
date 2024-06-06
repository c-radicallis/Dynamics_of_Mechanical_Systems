clear all
%
%Preprocess model data for MUBOKAP
%
%Summary: This script controls reading the input file for 
%         the model and the data storage into a workspace
%
%Output:  Workspace data saved in file to be used by MUBOKAP.
%         The model data includes:
%         Body        - Rigid bodies kinematics and matrices
%         Jnt         - Data for kinematic joints and drivers 
%         Pts         - Information on points of interest 
%         Time        - Analysis time period and time step
%         Parameter   - Parameters (size, tolerance ...)
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access memory
global H Flag Nline A90
global Time Parameter Jnt Pts
%%
%... Query for the input file with the model
[Filename,Path] = uigetfile('*.txt','Select Model Data file');
%
%... Read data from input file
H = dlmread([Path Filename]); %datamatrix 
%
%... Initialize counters
Nline                  = 1;
Parameter.NCoordinates = 0;
Parameter.NConstraints = 0;
%
%... Initialize analysis flags
Flag.ReadInput    = 1;
Flag.InitData     = 0;
Flag.Transfer     = 0;
Flag.Position     = 0;
Flag.Jacobian     = 0;
Flag.Velocity     = 0;
Flag.Acceleration = 0;
Flag.PostAccelera = 0;
%
%... Initialize vectors and orthogonal matrix
q0                = [];
A90               = [0 -1; 1 0];
%%
%... Store data in Local Variables
Parameter.NBody  = H(Nline,1);
Jnt.NRevolute    = H(Nline,2);
Jnt.NTranslation = H(Nline,3);
Jnt.NRevRev      = H(Nline,4);
Jnt.NTraRev      = H(Nline,5);
Jnt.NRigid       = H(Nline,6); 
Jnt.NSimple      = H(Nline,8);
Jnt.NDriver      = H(Nline,9);
Pts.NPointsOfInt = H(Nline,10);
%%
%... 1st pass- Read Input; 2nd pass - Initialize data
for pass = 1:2
    if pass == 2; Flag.InitData = 1; end
    %
    %... Read/Initialize Body and Constraint Data
    [q0,~,~,~] = KinemEval([],[],[]);
    %
    %...Store information for Points of Interest 
    for k = 1:Pts.NPointsOfInt
        PointsOfInterest(k,[]);
    end
    %
    %... In the 2nd pass (initialization) no more actions
    if pass==2
        Flag.InitData = 0;
        break
    end
    %
    %... Store the Newton-Raphson Parameters 
    Nline                 = Nline + 1;  
    Parameter.NLMaxIter   = H(Nline,1);
    Parameter.NLTolerance = H(Nline,2);
    %
    %...Store time analysis information 
    Nline      = Nline + 1; 
    Time.start = H(Nline,1);
    Time.step  = H(Nline,2);
    Time.tend  = H(Nline,3);
    %
    Time.time  = Time.start:Time.step:Time.tend;
    Time.Ntime = length(Time.time);
    %
    %... Verify if the kinematic analysis is feasible
    if Parameter.NConstraints ~= Parameter.NCoordinates
        disp('FATAL ERROR')
        string = ['# Constraints (' ...
                   num2str(Parameter.NConstraints) ...
                  ') differs from # Coordinates (' ...
                   num2str(Parameter.NCoordinates) ')'];
        disp(string)
        disp('Program stopped')
        return
    end
    %
    %... Reset Read Input Data Flag
    Flag.ReadInput = 0;
end
%%
%... Store the workspace in file 'model'.mat
[~,Name,ext] = fileparts(Filename);
save([Path Name '.mat']);
%%
%... Finish MUBOKAPPreProcess
