function c=computecost(path,qrand,q0)
c=0;
[nr nc]=size(path);
if(nr>2)
    for i=2:nr
%         cpath=norm(path(i,1:2)-path(i-1,1:2));
%         cheading=diffangle(path(i,3),path(i-1,3));
%         cmax=max([cpath cheading]);
%         c=cpath/cmax+cheading/cmax;
%         c=cpath+cheading;
    
%       c=c+norm(path(i,1:2)-path(i-1,1:2)) + diffangle(path(i,3),path(i-1,3));
    c=c+diffangle(path(i,3),path(i-1,3));
    end
else
    c=0;
end

% c=c+0.5*norm(path(end,1:2)'-qrand(1:2));
c=c+0.5*norm(path(end,1:2)-q0(1:2));

end
