function u=getControls(vchild,tau)

id=vchild.pid;
[nr nc]=size(tau);
u=[];
while(id>0)

    u=[vchild.edgeu;u];
    for i=1:nc
        if(tau(i).id==id)
            vchild=tau(i);
            id=vchild.pid;
            break;
        end
    end
    
    
    
end


end
