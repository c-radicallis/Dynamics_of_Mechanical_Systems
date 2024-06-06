%Solve_Dynamic.m
clear all
close all
clc
%
global SpringDamper Body tstart tstep tend solver FuncEval
%
disp('Need to run first the model to create mat file');
% select the workspace file with model data
[filename, Path]=uigetfile('*.mat','SelectModel Data');
% load model data
load([Path,filename]);
% Pre-process the input data
[tspan,y_init]=PreProcessor();
% integration of the equations of motion

CPUStart = cputime;
[t,y]=ode45(@FirstDynamicSystem,[tstart,tend],y_init);
CPUTime_ode45 = cputime-CPUStart

%Animation

Nframes=length(y);
colmap = ['b' , 'r' , 'y' , 'k'];

for i=1:Nframes
    for j=1:3
        ybody = y( i , 2*j-1);

        P1=[-0.04 , ybody-0.02];
        P2=[0.04 , ybody-0.02];
        P3=[0.04 , ybody+0.02];
        P4=[-0.04 , ybody+0.02];
       
        fill([P1(1) P2(1) P3(1) P4(1)],[P1(2) P2(2) P3(2) P4(2)] , colmap(j));
        text(0,ybody,num2str( j));
        hold on;
    end

    axis([-0.35 0.35 -0.1 0.6]);
    pbaspect([1 1 1]);
    hold off;
    pause(0.1);
end
close all

% post-process the results
x1_position=y(:,1);
x1_velocity=y(:,2);
x2_position=y(:,3);
x2_velocity=y(:,4);
x3_position=y(:,5);
x3_velocity=y(:,6);
%
figure(1);
xlabel('t'); ylabel('x_i(t)');
plot(t,x1_position,t,x2_position,t,x3_position); 
%legend('x1 - ode45','x2 - ode45','x3 - ode45'); 
hold on;
grid on

figure(2);
xlabel('t'); ylabel('v_i(t)');
plot(t,x1_velocity,t,x2_velocity,t,x3_velocity); 
%legend('v1 - ode45','v2 - ode45','v3 - ode45'); 
hold on;
grid on


CPUStart = cputime;
[t,y]=ode23(@FirstDynamicSystem,[tstart,tend],y_init);
CPUTime_ode23 = cputime-CPUStart

% post-process the results
x1_position=y(:,1);
x1_velocity=y(:,2);
x2_position=y(:,3);
x2_velocity=y(:,4);
x3_position=y(:,5);
x3_velocity=y(:,6);


%
figure(1);
hold on;
plot(t,x1_position,t,x2_position,t,x3_position); 
%legend('x1 - ode23','x2 - ode23','x3 - ode23'); 
hold on;
grid on

figure(2);
hold on;
plot(t,x1_velocity,t,x2_velocity,t,x3_velocity); 
%legend('v1 - ode23','v2 - ode23','v3 - ode23'); 
hold on;
grid on


CPUStart = cputime;
[t,y]=ode113(@FirstDynamicSystem,[tstart,tend],y_init);
CPUTime_ode113 = cputime-CPUStart


% post-process the results
x1_position=y(:,1);
x1_velocity=y(:,2);
x2_position=y(:,3);
x2_velocity=y(:,4);
x3_position=y(:,5);
x3_velocity=y(:,6);
%
figure(1);
hold on;
plot(t,x1_position,t,x2_position,t,x3_position); 
legend('x1 - ode45','x2 - ode45','x3 - ode45','x1 - ode23','x2 - ode23','x3 - ode23','x1 - ode113','x2 - ode113','x3 - ode113'); 
hold on;
grid on

figure(2);
hold on;
plot(t,x1_velocity,t,x2_velocity,t,x3_velocity);    
legend('v1 - ode45','v2 - ode45','v3 - ode45','v1 - ode23','v2 - ode23','v3 - ode23','v1 - ode113','v2 - ode113','v3 - ode113'); 
hold on;
grid on
