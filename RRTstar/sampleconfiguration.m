function qrand = sampleconfiguration(climits)
%  Generates a random configuration of ndim dimensions from an uniform
%  distribution. spacedim is a row vector containing the limits of the configuration space 

ndim = length(climits)/2;

% Iterate through all dimensions and generate sample
qrand = zeros(ndim,1);
for idim = 1:ndim
  qrand(idim,1) = climits(2*idim-1) + (climits(2*idim) - climits(2*idim-1)) * rand;
end;


