function qnear = findnearestvertex(tau,qrand)

% Put all poses of the tree struct into one matrix
vposes = [tau(:).pose];

% Subtract qrand pose from all columns
diffs = vposes(1:2,:) - repmat(qrand(1:2),1,size(vposes,2));

% Compute 2nd norm of each column 
distances = sqrt(sum(diffs.^2,1));

% Find vertex with minimal distance
[dmin,ivmin] = min(distances);

% Return nearest vertex
qnear = tau(ivmin);
