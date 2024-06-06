clear all
%
%MUBOKAP - Multibody Kinematic Analysis Program Post-Processor
%
%Summary: This function controls controls the processing of the
%         results of the model analysis and generates their
%         graphs and general reports
%
%Input:   Worspace with time histories loaded
%
%Output:  No output specified 
%
% Dynamics of Mechanical Systems
% Version 2.0     March, 2022
%
%%
%... Access global memory
global Time Pts Parameter
%
%%
%... Query for the input file with the model
[Filename,Path] = uigetfile('*.mat','Select ModelResults file');
%
%... Load the model workspace
load([Path Filename]);
%%
%... Initialize working quantities
n   = 0;
t   = Time.time;
lb0 = '$ '; lbn = ')}$'; lbv = '/s )}$'; lba = '/s\:^{2} )}$';
%

%... Make the general plots for the rigid bodies
% for i = 1:Parameter.NBody
%     %
%     %... Initializations
%     body = num2str(i);
%     lb1 = ['_\mathsf{' body '} \; \mathsf{(m'];
%     lb2 = ['_\mathsf{' body '} \; \mathsf{(rad'];
%     for k = 1:3
%         n = n+1;
%         %
%         %... Plot position, velocity & acceleration vs time
%         switch k
%             case 1
%                 plot(t,q(n,:),'color','black')
%                 label1 = [lb0 'x' lb1 lbn];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%                 %                
%                 plot(t,qd(n,:),'color','black')
%                 label1 = [lb0 '\dot{x}' lb1 lbv];
%                 ylabel(label1,'Interpreter','latex') 
%                 xlabel('time (s)'); figure
%                 %                
%                 plot(t,qdd(n,:),'color','black')
%                 label1 = [lb0 '\ddot{x}' lb1  lba];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%             case 2
%                 plot(t,q(n,:),'color','black')
%                 label1 = [lb0 'y' lb1 lbn];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%                 %                
%                 plot(t,qd(n,:),'color','black')
%                 label1 = [lb0 '\dot{y}' lb1 lbv];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%                 %                
%                 plot(t,qdd(n,:),'color','black')
%                 label1 = [lb0 '\ddot{y}' lb1 lba];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%             case 3
%                 plot(t,q(n,:),'color','black')
%                 label1 = [lb0 '\theta' lb2 lbn];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%                 %                
%                 plot(t,qd(n,:),'color','black')
%                 label1 = [lb0 '\dot{\theta}' lb1 lbv];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%                 %                
%                 plot(t,qdd(n,:),'color','black')
%                 label1 = [lb0 '\ddot{\theta}' lb1 lba];
%                 ylabel(label1,'Interpreter','latex')
%                 xlabel('time (s)'); figure
%         end
%     end
% end

%                
%%
%... Make the general plots for the rigid bodies
for k = 1:Pts.NPointsOfInt
    %
    %... Initializations
    x     = Pts.Int(k).q(1,:);    y   = Pts.Int(k).q(2,:);
    xd    = Pts.Int(k).qd(1,:);   yd  = Pts.Int(k).qd(2,:);
    xdd   = Pts.Int(k).qdd(1,:);  ydd = Pts.Int(k).qdd(2,:);
    point = num2str(k);
    lb1   = ['_\mathsf{' point '} \; \mathsf{(m'];
    %
    %... Plot the trajectory
    plot(x,y,'k')
    hold on
    plot(x(1),y(1),'ro','MarkerSize',8)
    axis([-4.0 8.0 -4.0 8.0]); 
    pbaspect([1 1 1]);
    xlabel([lb0 'x' lb1 lbn],'Interpreter','latex')
    ylabel([lb0 'y' lb1 lbn],'Interpreter','latex')
    figure

    %... Plot position vs time
    plot(t,x,'k', t,y,'k--')
    legend('x','y')
    ylabel([lb0 'z' lb1 lbn],'Interpreter','latex')
    xlabel('time (s)'); figure
    %
    %... Plot Velocities
    plot(t,xd,'k', t,yd,'k--')
    lgd = legend('$\dot{x}$','$\dot{y}$');
    set(lgd,'Interpreter','latex');
    ylabel([lb0 '\dot{z}' lb1 lbv],'Interpreter','latex')
    xlabel('time (s)'); figure
    %
    %... Plot Accelerations
    plot(t,xdd,'k', t,ydd,'k--')
    lgd = legend('$\ddot{x}$','$\ddot{y}$');
    set(lgd,'Interpreter','latex');
    ylabel([lb0 '\ddot{z}' lb1 lba],'Interpreter','latex')
    xlabel('time (s)'); figure
end
%%
%... Terminate the Kinematic Analysis Program Post-Processor

NTime = length(t);
for i = 1 : NTime
        A = [a , b];
        B =Model.l3*[cos(q(3,i)) , sin(q(3,i))  ];
        C =  A + Model.l2 *[ cos(q(2,i)),  sin(q(2,i))];
        D_linha = B - 0.9*Model.l2*[ cos( q(4,i)-deg2rad(99.15) ),  sin( q(4,i)-deg2rad(99.15) ) ];
        D = D_linha  - 0.2428*Model.l2*[ cos( q(4,i)-deg2rad(138.85) ),  sin( q(4,i)-deg2rad(138.85) ) ];
        E = D_linha - Model.l5*[ cos(q(5,i)),  sin(q(5,i))];
        F =[0,0];
        G = E+[-d,c];
        H = E+Model.l1 *[ cos(q(1,i)), sin(q(1,i))];
        pinca = G + 200*[ cos(q(1,i)-deg2rad(90)), sin(q(1,i)-deg2rad(90))];
        base =  G - 200*[1 , 0];

    plot(D(1),D(2),'g.');
    hold on;
    plot(pinca(1),pinca(2),'b.');
    hold on;
    plot(base(1),base(2),'r.');
    hold on;    

