% FirstDynamicSystem.m
function [yd]= FirstDynamicSystem(t,y)
% access global data memory
global M SpringDamper Body tstart tstep tend solver FuncEval

% update local data
Body(1).x=y(1,1);
Body(1).xd=y(2,1);
Body(2).x=y(3,1);
Body(2).xd=y(4,1);
Body(3).x=y(5,1);
Body(3).xd=y(6,1);
%
k1= SpringDamper(1).K;
c1= SpringDamper(1).C; 
L10=SpringDamper(1).L0;
x1= Body(1).x; 
xd1= Body(1).xd;
%
k2= SpringDamper(2).K; 
c2= SpringDamper(2).C;
L20=SpringDamper(2).L0;
x2= Body(2).x;
xd2= Body(2).xd;
%
k3= SpringDamper(3).K;
c3= SpringDamper(3).C; 
L30=SpringDamper(3).L0;
x3= Body(3).x;
xd3= Body(3).xd;
% create the force vector from local data
g=[( k1*(x2-x1-L10)  + k3*(x3-x1-L30) + c1*( xd2-xd1)+ c3*(xd3-xd1) );
( -k1*(x2-x1-L10)  + k2*(x3-x2-L20) - c1*( xd2-xd1) + c2*(xd3-xd2) );
( -k2*(x3-x2-L20)  - k3*(x3-x1-L30) - c2*( xd3-xd2) - c3*(xd3-xd1) )];
%
% Evaluate System accelerations
qdd=M\g;
% form the time derivative of y
yd=[Body(1).xd; qdd(1); Body(2).xd; qdd(2);Body(3).xd; qdd(3)];


