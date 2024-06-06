function PostProcessResults (t,y,Filename)
%ReadInputData
%
%Summary: This function controls controls the processing of the results of
%         the model analysis and generates their graphs and general reports
%
%Input:   t        - Vector with the time steps of the analysis
%         y        - Vector with history of positions and velocities
%         Filename - Filename of input data
%
%Output:  No output specified 
%
%
%%
%... Access memory
global NBody Body Frc NCoordinates NCoord1
global Ntime Pts Jnt NJoint Flag
global lambda acc
%
global w q
%
%% ... Rename and open report files
[~,name,~]      = fileparts(Filename);
FileOutput      = strcat(name,'.out');
FilePoInterest  = strcat(name,'.poi');
FileJntReaction = strcat(name,'.jnt');
%
fileOUT = fopen(FileOutput,'w');
filePOI = fopen(FilePoInterest,'w');
fileJNT = fopen(FileJntReaction,'w');
%
%% ... Create headers for the report files
%
% ... Header for the output file with bodies kinematics
str0  = '...Time...|';
str1  = '....................................................';
str2  = '..................Body (';
str3  = ')...................';
str4  =['.......X........|.......Y........|.......O........|'];
str5  =['.......Xd.......|.......Yd.......|.......Od.......|'];
str6  =['.......Xdd......|.......Ydd......|.......Odd......|'];
strA  = str0;
strB  = '          |';
for i=1:NBody
    str  = sprintf('%03d',i);
    strA = strcat(strA,'.',str1,str2,str,str3,str1,'|');
    strB = strcat(strB,str4,str5,str6);
end
fprintf(fileOUT, '%s\n',strA);
fprintf(fileOUT, '%s\n',strB);
%
% ... Header for the output file with Points of Interest
str1 = '..............';
str2 = 'Point of Interest (';
str3 = ')...';
str4 = 'Body (';
str5 = 'Coordinates (';  
str6 = ')';  
str7 = '.......X........|.......Y........|';
str8 = '.......Xd.......|.......Yd.......|';
str9 = '.......Xdd......|.......Ydd......|';
strA = str0;
strB = '          |';
for k=1:Pts.NPointsInt
    stra = sprintf('%03d',k);
    strb = sprintf('%03d',Pts.Int(k).i);
    strc = sprintf('%8.4f , %8.4f',Pts.Int(k).spPi);
    strA = strcat(strA,'.',str1,str2,stra,str3,str4, strb, str3, ...
                  str5,strc,str6,str1,'|');
    strB = strcat(strB,str7,str8,str9);
end
fprintf(filePOI, '%s\n',strA);
fprintf(filePOI, '%s\n',strB);
%
% ... Header for the output file with Joint Reactions
%       12345678901234567890123456789012345678901234
str1 = '..............';
str2 = '...Joint (';
str3 = ').....';
str4 = 'Type (';
str5 = '......Body (';
str6 = ')';  
str7 = '.......fX.......|.......fY.......|......Mom.......|';
strA = str0;
strB = '          |';
strC = '          |';
for k=1:Jnt.NReaction
    stra = sprintf('%03d',k);
    strb = sprintf('%s'  ,Jnt.Reaction(k).Type);
    strc = sprintf('%03d',Jnt.Reaction(k).Number);
%
    strd = sprintf('%03d',Jnt.Reaction(k).i);
    strB = strcat(strB,'.',str1,str5,strd,str3,str1,'|');
    strC = strcat(strC,str7);
    if Jnt.Reaction(k).j == 0
        strA = strcat(strA,'.',str2,stra,str3,...
                               str4,strb,strc,str3,'|');
    else
        strd = sprintf('%03d',Jnt.Reaction(k).j);
        strA = strcat(strA,'.',str1,str1,str2,stra,str3,...
                               str4,strb,strc,str6,str1,str1,'|');
        strB = strcat(strB,'.',str1,str5,strd,str3,str1,'|');
        strC = strcat(strC,str7);
    end
%
end
fprintf(fileJNT, '%s\n',strA);
fprintf(fileJNT, '%s\n',strB);
fprintf(fileJNT, '%s\n',strC);
%
%% ... Create waitbar
% w       = waitbar(0,'Report Progress');
%
%% ... Build the time history of accelerations, joint forces & PoI
Flag.Position     = 0;
Flag.Velocity     = 0;
Flag.Jacobian     = 1;
Flag.General      = 1;
%
% Allocates memory for the forces
SDForces    = zeros(Ntime, 1);
ActForce    = zeros(Ntime, 1);

for k=1:Ntime
    Flag.Transfer     = 1;
    Flag.Acceleration = 1;
    Flag.Reaction     = 0;
    [~]               = FuncEval(t(k,1),y(k,:)');
    q  (:,k)          = y(k,1:NCoordinates)';
    qd (:,k)          = y(k,NCoord1:end)';
    qdd(:,k)          = acc;
%
% ... Print information on Output file with the bodies kinematics
    fprintf(fileOUT, '%10.4f',t(k,1));
    for i=1:NBody
%
% ... Find the acceleration of the rigid bodies
        i1 = 3*i - 2;
        i2 = i1 + 1;
        i3 = i2 + 1;
        Body(i).rdd     = qdd(i1:i2,k);
        Body(i).thetadd = qdd(i3:i3,k);
        
%... Makes an animation of the bodies
        switch i
            case 1
                % Points of body 1
                P1 = Body(i).r + Body(i).A * [-0.300; 0.000];
                P2 = Body(i).r + Body(i).A * [0.300;  0.000];
                % plot([P1(1) P2(1)],...
                    % [P1(2) P2(2)], 'r');
                hold on;
            case 2
                % Points for body 2
                P1 = Body(i).r + Body(i).A * [-0.400; 0.0];
                P2 = Body(i).r + Body(i).A * [0.000; 0.0];
                % plot([P1(1) P2(1)], [P1(2) P2(2)], 'b');
            case 3
                % Points for body 3
                P1 = Body(i).r + Body(i).A * [-0.150; 0.0];
                P2 = Body(i).r + Body(i).A * [0.150; 0.0];
                % plot([P1(1) P2(1)], [P1(2) P2(2)], 'k');
        end        
        
%
        % fprintf(fileOUT, '%17.9d',Body(i).r  ,Body(i).theta  );
        % fprintf(fileOUT, '%17.9d',Body(i).rd ,Body(i).thetad );
        % fprintf(fileOUT, '%17.9d',Body(i).rdd,Body(i).thetadd);
    end
    % fprintf(fileOUT, '\n');
    % axis([-0.4 0.4 0.0 0.8]);
    % pbaspect([1 1 1]);
    % hold off;
    % pause(0.05);
    %
%....
%    Evaluate spring-damper forces
    for sd = 1 : Frc.NSprDamper
        i  = Frc.SprDamper(sd).i;
        j  = Frc.SprDamper(sd).j;
%
% ... Evaluate the Spring force
        d  = Body(i).r + Body(i).A*Frc.SprDamper(sd).spPi - ...
             Body(j).r - Body(j).A*Frc.SprDamper(sd).spPj;
        l  = sqrt(d'*d);
        u  = d/l;
        fk = Frc.SprDamper(sd).k*(l-Frc.SprDamper(sd).l0);
%
% ... Evaluate the Damper force
        dd = Body(i).rd+Body(i).B*Frc.SprDamper(sd).spPi*Body(i).thetad- ...
             Body(j).rd-Body(j).B*Frc.SprDamper(sd).spPj*Body(j).thetad;
        ld = dd'*u;
        fd = Frc.SprDamper(sd).c*ld;
%
% ... Evaluate the total force
        SDForces(k)  = (fk + fd + Frc.SprDamper(sd).a);
    end       

%
% ... Print information on Points of Interest file
    PointsOfInterest(k);
    % fprintf(filePOI, '%10.4f',t(k,1));
    for n=1:Pts.NPointsInt
        % fprintf(filePOI, '%17.9d',Pts.Int(n).q(:,k));
        % fprintf(filePOI, '%17.9d',Pts.Int(n).qd(:,k));
        % fprintf(filePOI, '%17.9d',Pts.Int(n).qdd(:,k));
    end
    % fprintf(filePOI, '\n');
%
% ... Print information on Joint Reaction Forces file
%
%... Evaluate the Joint Reaction Forces
    NJoint            = 0;
    Flag.Transfer     = 0;
    Flag.Acceleration = 0;
    Flag.Reaction     = 1;
    [~,Jac,~,~]         = KinemEval(t(k,1),[],[]);
%
%... Print the Joint Reaction forces to file
    fprintf(fileJNT, '%10.4f',t(k,1));
    for NJoint=1:Jnt.NReaction
        % fprintf(fileJNT, '%17.9d',Jnt.Reaction(NJoint).gi);
        if Jnt.Reaction(NJoint).j ~= 0
            % fprintf(fileJNT, '%17.9d',Jnt.Reaction(NJoint).gj);
        end
    end
    % fprintf(fileJNT, '\n');
    
    %... Computes the driver joint reaction force
    if (Jnt.NDriver ~= 0)
        % Computes the vector g for the driver
        gdr = - Jac(end,:)' * lambda(end);
        % Saves the force for body 4
        ActForce(k) = -gdr(1);   
    end
end
fclose(fileOUT);
fclose(filePOI);
fclose(fileJNT);
%... Plot the spring-damper force
% if (Frc.NSprDamper ~= 0)
%     figure();
%     plot(t, SDForces);
%     xlabel('Time (s)');
%     ylabel('Spring-damper-actuator force (N)');
% end
% if (Jnt.NDriver ~= 0)
%     figure();
%     plot(t, ActForce);
%     xlabel('Time (s)');
%     ylabel('Actuator force (N)');
% end  
%
%%
%... Make the general plots
% for k = 1:Pts.NPointsInt
%
%... Plot position vs time
    % plot(t,Pts.Int(k).q(1,:),'k',t,Pts.Int(k).q(2,:),'k--')
    % title(['Point of interest ' num2str(k) ' - Position'])
    % figure
%
%... Plot the trajectory
    % plot(Pts.Int(k).q(1,:),Pts.Int(k).q(2,:),'k')
    % title(['Point of interest ' num2str(k) ' - Trajectory'])
    % figure
%
%... Plot Velocities
%     plot(t,Pts.Int(k).qd(1,:),'k',t,Pts.Int(k).qd(2,:),'k--')
%     title(['Point of interest ' num2str(k) ' - Velocity'])
%     figure
% %
% %... Plot Accelerations
%     plot(t,Pts.Int(k).qdd(1,:),'k',t,Pts.Int(k).qdd(2,:),'k--')
%     title(['Point of interest ' num2str(k) ' - Acceleration'])
%     figure
% end
return
%% my plots
global connects

connects = [5 6; % body2
    6 3; % body3
    3 4; % body4
    4 14; %body4-body5
    1 7; % hy1 
    8 9 % hy2
    10 11 % caixa
    11 13 % caixa
    13 12 % caixa
    12 10 % caixa
    15 16]; %hydraulic 3 - telescopic 

image_array = {};

global Pts
close all
figure, hold on, grid on,pbaspect([1 1 1]);
xlim([-400 900])
ylim([0 1000])
pbaspect([1 1 1])

plt_data = {};

for i = 1:10:length(t) %skip 5 frames
    
    tic
    i
    for j = 1:size(connects,1)
        if j == 11
            continue
        end

        p1 = Pts.Int(connects(j,1)).q(:,i);
        p2 = Pts.Int(connects(j,2)).q(:,i);

        if i == 1
            plt_data{j} = plot([p1(1) p2(1)], [p1(2) p2(2)]);

        else
            plt_data{j}.XData = [p1(1) p2(1)];
            plt_data{j}.YData = [p1(2) p2(2)];
        end

    end
    drawnow
    waitforbuttonpress
end


%% size of hydrauilics
global connects
figure,
n_connect = 5;
hy1_size = sqrt( sum((Pts.Int(connects(n_connect,1)).q - Pts.Int(connects(n_connect,2)).q).^2,1) );
plot(hy1_size)
title("hydraulic 1 size")


figure,n_connect = 6;
hy2_size = sqrt( sum((Pts.Int(connects(n_connect,1)).q - Pts.Int(connects(n_connect,2)).q).^2,1) );
plot(hy2_size)
title("hydraulic 2 size")

figure,n_connect = 11;
hy2_size = sqrt( sum((Pts.Int(connects(n_connect,1)).q - Pts.Int(connects(n_connect,2)).q).^2,1) );
plot(hy2_size)
title("hydraulic 3 size")

%% angle of the bucket
global q
body_num = 5;
ax1 = subplot(2,2,1)
ax2 = subplot(2,2,2)
ax3 = subplot(2,2,[3 4])
plot(ax3, q(body_num*3,:)), 
plot(ax2, q(body_num*3-1,:))
plot(ax1, q(body_num*3-2,:))
title(ax1,"x")
title(ax2,"y")
title(ax3,"\theta")

%
%
%%
%... Finish function PreProcessData
end
