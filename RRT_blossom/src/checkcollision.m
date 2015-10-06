function flag=checkcollision(q,obs);
% check the single configuration q for all the obstacles in obs.
% the collision checker implemented works only for rectangular obstacles
global widthrobot
global lengthrobot

robotmaxsize= max([widthrobot lengthrobot]);
[h nobs]=size(obs);


% flag == 1  collision on the state
% flag == -1 no collision on the state
%flag = 1;
for o=1:nobs


    dist=norm(q'-obs(o).position);
    
    
    if(dist-(obs(o).radius+robotmaxsize/2)>0)
        flag=-1;
    else
        flag=1;
        break;
    end

    
    
    
    
end





end

