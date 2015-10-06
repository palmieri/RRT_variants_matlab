function vnew=extend(vnear,qrand)

global cntId

q0 = vnear.pose;
         
[Qnew, speedvec, vel] = positioncontrolWP(q0, qrand, 1, .1, 0.35);


Qnew=Qnew';
vnew.id=cntId;
vnew.pose=Qnew(end,:);
vnew.edgeq=Qnew(:,:);
vnew.edgeu=vel;
vnew.pid=vnear.id;
vnew.cost=computecost(Qnew,qrand,q0)+vnear.cost;


end

