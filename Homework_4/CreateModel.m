%CreateModel.m
clear all
%
SpringDamper(1).K=25; 
SpringDamper(1).C=2; 
SpringDamper(1).L0=0.5;
%
SpringDamper(2).K=16;
SpringDamper(2).C=2; 
SpringDamper(2).L0=0.1;
%
SpringDamper(3).K=30; 
SpringDamper(3).C=1; 
SpringDamper(3).L0=0.2;
%
%
Body(1).mass=15; 
Body(1). x=0.0; 
Body(1).xd=0;
%
Body(2).mass=25; 
Body(2).x=0.5;
Body(2).xd=0;
%
Body(3).mass=5; 
Body(3).x=0.1;
Body(3).xd=0;
%
tstart=0; tstep=0.01; tend=7;
solver='ode45';
FuncEval='FirstDynamicSystem';
%
save('FirstDynamicModel.mat');
%
% End of the script