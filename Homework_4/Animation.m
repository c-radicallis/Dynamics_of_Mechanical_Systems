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