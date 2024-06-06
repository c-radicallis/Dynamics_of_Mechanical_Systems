function PostProcessResults (q,qd,qdd);
%ReadInputData
%
%Summary: This function controls controls the processing of the results of
%         the model analysis and generates their graphs and general reports
%
%Input:   q   - Vector with the history of the system positions
%         qd  - Vector with the history of the system velocities
%         qdd - Vector with the history of the system accelerations
%
%Output:  No output specified 
%
%
%%
%... Access memory
global Time Pts Jnt
%
%%
%... Make the general plots
for k = 1:Pts.NPointsOfInt
%
%... Plot position vs time
    plot(Time.time,Pts.Int(k).q(1,:),'k',Time.time,Pts.Int(k).q(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Position'])
    figure
%
%... Plot the trajectory
    plot(Pts.Int(k).q(1,:),Pts.Int(k).q(2,:),'k')
    title(['Point of interest ' num2str(k) ' - Trajectory'])
    figure
%
%... Plot Velocities
    plot(Time.time,Pts.Int(k).qd(1,:),'k',Time.time,Pts.Int(k).qd(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Velocity'])
    figure
%
%... Plot Accelerations
    plot(Time.time,Pts.Int(k).qdd(1,:),'k',Time.time,Pts.Int(k).qdd(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Acceleration'])
    figure
%
%... Plot position vs the first driver angle/dispacement
    if (Jnt.NDriver ==0 || Jnt.Driver(1).type>2); break; end
    c = 3*(Jnt.Driver(1).i-1)+Jnt.Driver(1).coortype;
    plot(q(c,:),Pts.Int(k).q(1,:),'k',q(c,:),Pts.Int(k).q(2,:),'k--')
    title(['Point of interest ' num2str(k) ' - Position vs crank angle'])
    figure
end
%
%
%%
%... Finish function PreProcessData
end

%... Plot position vs time
%     plot(q(7,:),q(8,:))
%     title(['Point of interest ' num2str(1) ' - Position'])
%     figure

