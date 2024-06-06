function [tspan, y_init]=PreProcessor();
%
%
global M SpringDamper Body tstart tstep tend solver FuncEval L
%
M=[ Body(1).mass      0; ...
           0      Body(2).mass]
%
y_init=[Body(1).x  Body(1).xd Body(2).x Body(2).xd]'
%
tspan=tstart:tstep:tend;
end
