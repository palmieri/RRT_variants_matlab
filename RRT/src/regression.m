function flag=regression(qnear,qnew,tau)



rho=norm(qnear.pose(1:2)-qnew.pose(1:2));

[nr nc ]=size(tau);

for i=1:nc

    if(norm(qnew.pose(1:2)-tau(i).pose(1:2))<rho)
    flag=1
    break;
    else
    flag=0;
    end
    
end





end
