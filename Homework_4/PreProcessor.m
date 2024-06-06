% PreProcessor.m
function [tspan, y_init]=PreProcessor();
%
% access global memory
global M SpringDamper Body tstart tstep tend solver FuncEval

%Create Mass matrix
M=[Body(1).mass,     0,                           0;
        0,                       Body(2).mass,       0;
        0,                       0,                             Body(3).mass];

% create initial y vector
y_init=[Body(1).x Body(1).xd Body(2).x Body(2).xd Body(3).x Body(3).xd]';

% time analysis profile
tspan=tstart:tstep:tend;
end