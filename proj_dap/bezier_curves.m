function bezier_curves(hydraulic_num, start_lenght, end_lenght)

global tend

n=4;
n1 = n-1;

p = [0      start_lenght;
    tend/2  start_lenght;
    tend/2  end_lenght;
    tend  end_lenght];

for    i=0:1:n1
    sigma(i+1)=factorial(n1)/(factorial(i)*factorial(n1-i));  % for calculating (x!/(y!(x-y)!)) values
end

l=[];
UB=[];
for u=0:0.01:1
    for d=1:n
        UB(d)=sigma(d)*((1-u)^(n-d))*(u^(d-1));
    end
    l=cat(1,l,UB);                                      %catenation
end
P=l*p;

% line(P(:,1),P(:,2))
% line(p(:,1),p(:,2))

dlmwrite(sprintf("Driver_00%i.txt", hydraulic_num), P, 'delimiter', ' ');

end