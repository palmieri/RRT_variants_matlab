function [p,u] = extractpath(vchild,tau)

id = vchild.pid;
nvertices = length(tau);

p = [];
u = [];
while id > 0,
  
  p = [vchild.edgeq; p];
  u = [vchild.edgeu; u];
  for i = 1:nvertices,
    if tau(i).id == id,
      vchild = tau(i);
      id = vchild.pid;
      break;
    end;
  end;
  
end;
