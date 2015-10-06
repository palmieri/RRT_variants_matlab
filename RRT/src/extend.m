function vnew=extend(vnear,qrand,motionPrimitiveCommandArray,delta)

global cntId
[nPrimitives ~]=size(motionPrimitiveCommandArray);

% ode parameters
tSpan = [0 delta]; % solve from t=0 to t=delta
q0 = vnear.pose;

mind=100;
min=0;

for i=1:nPrimitives
    
    
    %     generate the i-th motion primitive
    v     = motionPrimitiveCommandArray(i,1);
    omega = motionPrimitiveCommandArray(i,2);
    [T Q] = ode45(@(t,q) unicycleKinematics( t, q, v ,omega ),tSpan,q0); % solve ODE
    
    
    d = norm(Q(end,1:2)'-qrand(1:2));
    if d < mind
        mind=d;
        min=i;
            
    end
    
    
end

    velnew     = motionPrimitiveCommandArray(min,1);
    omeganew = motionPrimitiveCommandArray(min,2);
    [Tnew Qnew] = ode45(@(t,q) unicycleKinematics( t, q, velnew ,omeganew ),tSpan,q0); % solve ODE
        
    
    cntId=cntId+1;

    vnew.id=cntId;
    vnew.pose=Qnew(end,:)';
    vnew.edgeq=Qnew(:,:);
    vnew.edgeu=[velnew omeganew];
    vnew.pid=vnear.id;


end

