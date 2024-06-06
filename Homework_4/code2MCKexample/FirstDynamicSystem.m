function [yd]= FirstDynamicSystem(t,y)
%
%
global M SpringDamper Body tstart tstep tend solver FuncEval L
%
Body(1).x=y(1,1)
Body(2).x=y(3,1)
Body(1).xd=y(2,1)
Body(2).xd=y(4,1)
%
% create force vector
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
%
%
  g=[(-k1*(x1-L10)-...
       c1* xd1-...
       k2*(x1-x2+L20)-...
       c2*(xd2-xd1));      
     (-k2*(x2-x1-L20)-...
       c2*(xd2-xd1)-...
       k3*(x2-L+L30)-...
       c3*(xd2))];
%
%  Evaluate System accelerations
%
qdd=M\g;
%
yd=[Body(1).xd; qdd(1); Body(2).xd; qdd(2)];
%
