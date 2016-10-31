function collision = checkedgecollision(qnew,robotradius,obs)

npoints = size(qnew.edgeq, 1);
collision = false;
for i = 1:npoints,   
   isincollision = checkcollision(qnew.edgeq(i,1:2), robotradius, obs);
   if isincollision
    collision = true;
    break;
  else
    collision = false;
  end;
  
end;

end
