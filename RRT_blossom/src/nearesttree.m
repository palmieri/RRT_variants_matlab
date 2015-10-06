function vnear=nearestTree(tau,qrand)

[nr nc]=size(tau);

mind=100;
min=0;
for v=1:nc

    d=norm(qrand(1:2)-tau(v).pose(1:2));
    if(d<mind)
        min=v;
        mind=d;
    end
    
end

vnear=tau(min);



end

