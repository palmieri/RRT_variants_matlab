function collision = checkcollision(q,robotradius,obs)

% Checks the configuration q of circular robot with radius robotradius
% for all the circular obstacles in obs

% Compute distances in center positions
diffs = obs(:,1:2) - repmat([q(1) q(2)], size(obs,1), 1);
cdist = sqrt(sum(diffs.^2,2));

% Compute distances between peripheries
pdist = cdist - obs(:,3) - robotradius;

% Check if any pdist is smaller than zero
if any(pdist < 0),
    collision = true;
    return 
else
    collision = false;
end;

end
