function p=extractpath(vchild,tau)

id=vchild.pid;
[nr nc]=size(tau);
p=[];
while(id>0)

    p=[vchild.edgeq;p];
    for i=1:nc
        if(tau(i).id==id)
            vchild=tau(i);
            id=vchild.pid;
            break;
        end
    end
    
    
    
end




end

