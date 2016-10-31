function qNearVertices = findNearVertices(tau, tau_qnew, r)

qnew = tau_qnew.pose;
% Put all poses of the tree struct into one matrix
vposes = [tau(:).pose];

% Subtract qrand pose from all columns
diffs = vposes(1:2,:) - repmat(qnew(1:2),1,size(vposes,2));

% Compute 2nd norm of each column 
distances = sqrt(sum(diffs.^2,1));

% Find vertex with minimal distance
%distances(distances < r & distances>0);

% Return nearest vertex
qNearVertices_partial = tau(distances < r & distances>0);
[nr nc] = size(qNearVertices_partial);
if(nr>1 | nc>1)
    % can it be improved via some smart MATLAB operator?..    
    for j=1:nc
        if(qNearVertices_partial(j).id == tau_qnew.pid)
            remove_parent_id = j;
        end 
    end
    
    qNearVertices_partial(remove_parent_id)=[];
else
    qNearVertices_partial=[];

end

qNearVertices = qNearVertices_partial;

end 
