fill([0 0 2.5 2.5] , [0 -1 -1 0],"c")
hold on
plot([0 0 2.5 2.5], [0 -0.18 -0.18  0],"r-",'LineWidth',5)
plot([0 0 2.5 2.5], [0 -0.18*2 -0.18*2  0],"b-",'LineWidth',5)
legend("Area 1", "1st Pass", "2nd Pass")
axis([-1 3 -2 2])
    xlabel('x (m)');
    ylabel('y (m)');
%pbaspect([1 1 1])