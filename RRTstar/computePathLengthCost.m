% compute the cost c associated to an edge of size MxN, where M are
% the points and N the dimension of the state
function c=computePathLengthCost(edge)
% the cost here is simply the Euclidean distance between initial and final
% pose of the edge
[M,N] = size(edge);
dy = edge(end,2) - edge(1,2);
dx = edge(end,1) - edge(1,1);
c = sqrt(dx*dx + dy*dy);
end
