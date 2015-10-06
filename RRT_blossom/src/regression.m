function flag=regression(qnear,qnew,tau)



rho=norm(qnear.pose(1:2)-qnew.pose(1:2));



flag=0;
vchild=qnear;
id=vchild.pid;
[nr nc]=size(tau);
d=1000;
while(id>0)

    
    
    
    if(norm(qnew.pose(1:2)-vchild.pose(1:2))<rho)
    flag=1
    break;
    else
    flag=0;
    end

    for i=1:nc
        if(tau(i).id==id)
            vchild=tau(i);
            id=vchild.pid;
            break;
        end
    end
    
    
    
end

% for c=1:nc
%     
%     if(norm(qnew.pose(1:2)-tau(c).pose(1:2))<rho)
%     flag=1
%     break;
%     else
%     flag=0;
%     end
%     
% end
% 




end
