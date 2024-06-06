
clear all, close all

global solver Pts Body Frc NTime q NCoordinates Flag

% preprocess
[Filename] = ReadInputData();

bezier_curves(1, 92, 145)
bezier_curves(2, 123, 215)
bezier_curves(3, 345, 345)

figure(1) % theta of bucket
ax = gca;
hold on

% iter over many damping coeffs.
c_iter = linspace(50,300, 5);
for i = 1:length(c_iter)
    
    [Filename] = ReadInputData();
    Frc.SprDamper.c = c_iter(i);

    [tspan,y_init] = PreProcessData();
    [t,y] = Integration_odes(y_init,solver,tspan);
    

    for k = 1:length(t)
        q(:,k) = y(k,1:NCoordinates)';
    end

    plot(wrapToPi(q(end,:)),'DisplayName',sprintf("c=%.2f", c_iter(i)))

end

