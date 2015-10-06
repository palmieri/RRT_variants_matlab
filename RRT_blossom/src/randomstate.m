function qrand=randomstate(ndim,dim)
%  generate a configuration of ndim dimensions
%   
%  dim is a row vector containing the domain limits where to generate the
%  configuration
%  e.g configuration of two elements (x,y)
%  randomstate(2,[xmin xmax, ymin , ymax]
%  



l=floor(length(dim)/2);
if (ndim~=l)
disp('Error: ndim and the given dimensions do not match');
qrand=[];
return;
end

for j=1:ndim
qrand(j)=  dim(2*j-1)+(dim(2*j) -dim(2*j-1))*rand();  
end

qrand=qrand';

end

