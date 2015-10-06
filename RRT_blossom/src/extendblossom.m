function [vnew tau]=extendblossom(tau,obs,vnear,qrand,motionPrimitiveCommandArray,delta,PRINTITER)

global addedVert
global cntId
[nPrimitives ~]=size(motionPrimitiveCommandArray);

% ode parameters
tSpan = [0 delta]; % solve from t=0 to t=delta
q0 = vnear.pose;
addedV=[];




for i=1:nPrimitives
    
    
    %     generate the i-th motion primitive
    v     = motionPrimitiveCommandArray(i,1);
    omega = motionPrimitiveCommandArray(i,2);
    [T Q] = ode45(@(t,q) unicycleKinematics( t, q, v ,omega ),tSpan,q0); % solve ODE
    
    
    cntId=cntId+1;
    
    vnewpartial.id=cntId;
    vnewpartial.pose=Q(end,:)';
    vnewpartial.edgeq=Q(:,:);
    vnewpartial.edgeu=[v omega];
    vnewpartial.pid=vnear.id;

    
    
    status=checkedge(obs,vnewpartial);
    if(status == -1)
        disp('collision')
        cntId=cntId-1;
      
        continue
    end
     



    flag_regression=regression(vnear,vnewpartial,tau);
    if(flag_regression == 1)
        cntId=cntId-1;
        disp('regression')
        continue
    end
    
    
    
    addedVert=addedVert+1;
    tau(addedVert).id=vnewpartial.id;
    tau(addedVert).pose=vnewpartial.pose;
    tau(addedVert).edgeq=vnewpartial.edgeq;
    tau(addedVert).edgeu=vnewpartial.edgeu;
    tau(addedVert).pid=vnewpartial.pid;
    
    
    
    if(PRINTITER)
    figure(10),plot(tau(addedVert).edgeq(:,1),tau(addedVert).edgeq(:,2));
    end
    
    
    
end


vnew=nearesttree(tau,qrand);

end

