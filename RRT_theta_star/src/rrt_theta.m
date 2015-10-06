%% basic RRT algorithm
% the script aims to show the basic funcionalities of RRT, for a nonholonomic
% differential drive wheeled mobile robot.
% Motion Primitives are generated using the ODE command
% The kinematic is descrirbed by the unicycleKinematics.m script.
% 1.0 v Luigi Palmieri,Social Robotic Lab Freiburg
% 
% How obstacles are defined. Obstacles are considered circle
%  obs(i).position
%  obs(i).width
%  obs(i).heigth
% 
% How a vertex is defined
% v.id  -- ID of the vertex
% v.pose -- Pose of the vertex
% v.edgeq -- edge where to save intermediate configurations
% v.edgeu -- edge where to save intermediate controls u
% v.pId -- parent ID


clear all
clc
close all

RNDSEED=1;
SEED=2486
% ----- Determine random seed
if RNDSEED,
  rndvec = randperm(200);
  rndseed = rndvec(1);
else
  rndseed = SEED;
end;
rand('twister',rndseed);

 addpath('/media/data/software/librobotics')

%% Parameters

PRINTITER=1
PRINTSAMPLE=1
LOAD=0

% size robot

global widthrobot
global lengthrobot

widthrobot=0.4;
lengthrobot=0.6;


% configuration space
N=3;
dim= [0 10 0 10 0 2*pi];


% tree
tau={};
% max number of iterations
I=500;

% initial and end pose
qinit=[2;1;0]';
qgoal=[4;3.5;0]';
radiusGoal=0.70;


%% obstacle definitions/reading


if(LOAD)
obst = dlmread('environment.txt');
[nr_obst nc_obst]=size(obst);

for i=1:nr_obst
obs(i).position=[obst(i,1);obst(i,2)];
obs(i).radius=obst(i,3);

end

else
    
obs(1).position=[3;2.5];
obs(1).radius=0.51;

obs(2).position=[8;1];
obs(2).radius=0.51;

obs(3).position=[1;3];
obs(3).radius=0.51;


obs(4).position=[5;2.5];
obs(4).radius=0.51;

obs(5).position=[1;5];
obs(5).radius=0.51;

obs(5).position=[7;3];
obs(5).radius=0.21;


obs(6).position=[2.5;5.5];
obs(6).radius=0.51;

obs(7).position=[4.3;8];
obs(7).radius=0.21;

obs(8).position=[2;7];
obs(8).radius=0.41;

obs(9).position=[8;2];
obs(9).radius=0.31;

obs(10).position=[5;5];
obs(10).radius=0.21;

end






[h nobs]=size(obs);
figure(10),hold on,axis equal

%% plot environment
for i=1:nobs
        drawellipse([obs(i).position;0],obs(i).radius,obs(i).radius,'r');
    
end

%% plot initial position
drawrobot(qinit,'b',2,widthrobot,lengthrobot)


%% plot goal region
drawellipse(qgoal,radiusGoal,radiusGoal,'b');





%% global id counter
global cntId
cntId=1;



%% RRT Steps

%% add qinit to the tree

tau(1).id=1;
tau(1).pose=qinit;
tau(1).edgeq= [];
tau(1).edgeu= [];
tau(1).pid=0;
tau(1).cost=0;



addedVert=1;
collision=0;
goalFound=0;

max_number_iteration=1000;
for it=1:max_number_iteration

    
    %% Generate Configuration
    
    qrand=randomstate(N,dim);
    % check
    if(isempty(qrand))
        break;
    end
    
    if(checkcollision(qrand(1:2)',obs)>0)
        disp('wrong qrand')
        continue
    end
    
    if(PRINTSAMPLE)
    figure(10),plot(qrand(1),qrand(2),'*g')
    end
    
    
    vnear=nearestTree(tau,qrand);
    
%% all parents vnear
    

    

    % adding vertices to the  set_parents_vnear
    % from each vertex, qrand has to be reachable, with no collision
    rootreached=1;
    set_parents_vnear={};
    counter_parents_vnear=1; % counter parents added
    counter_parents_checked=1; % counter parents checked

    status_collision=1;
    hvec=[];
    
    while(rootreached>0)
    
        
        
        % first iteration  extend vnear
        if(counter_parents_checked==1)
        
        vtoextend=vnear;


        else
        % check if root otherwise select the new vertex as the parent of the current one 
            if(vtoextend.pid==0)
           
                rootreached=0;
                break;

            else
                vtoextend=searchvertex(vtoextend.pid,tau)
                disp('Parent ID:')
                vtoextend.pid
                
            end
        
        end
        
 
       
        
        disp('extending the tree from vtoextend');
        vnew_i=extend(vtoextend,qrand);
        figure(10)
        hold on,
        h=plot(vnew_i.edgeq(:,1),vnew_i.edgeq(:,2),'.g');
        pause(1)
        
        
        
        disp('checking collision');
        status_collision=checkedge(obs,vnew_i);
        counter_parents_checked=counter_parents_checked+1;
        
        
        if(status_collision == -1)
        h1=plot(vnew_i.edgeq(:,1),vnew_i.edgeq(:,2),'.r');
        pause(.51)

        disp('collision');
        delete(h)
        delete(h1)
        continue
            
        else
                
  
        
        disp('vertex reachable');
        set_parents_vnear(counter_parents_vnear).id=vnew_i.id;
        set_parents_vnear(counter_parents_vnear).pose=vnew_i.pose;
        set_parents_vnear(counter_parents_vnear).edgeq=vnew_i.edgeq;
        set_parents_vnear(counter_parents_vnear).edgeu=vnew_i.edgeu;
        set_parents_vnear(counter_parents_vnear).pid=vnew_i.pid;
        set_parents_vnear(counter_parents_vnear).cost=vnew_i.cost;
        counter_parents_vnear=counter_parents_vnear+1;

        end
        
        
        delete(h)
        
    end
    

   
   
    disp('choosing parent according to the cost');
    % choosing parent according to the cost
    [nr nv]=size(set_parents_vnear)
    % no connections avialable
    if(nv==0)
        continue;
    end
    
    vmin=set_parents_vnear(end);
    mincost=set_parents_vnear(end).cost;
    
    for i=1:nv
        
        % root vertex pid can be zero only if the vertex is added at the
        % first iteration of the expansion
        if(set_parents_vnear(i).pid==0 || set_parents_vnear(i).pid==1  )
            vmin=set_parents_vnear(i);
            disp('direct connection')
            break;
        end
        
        
        if(set_parents_vnear(i).cost<=mincost)
            vmin=set_parents_vnear(i);
            mincost=set_parents_vnear(i).cost;
        end
        
        
    end
    
    
    disp('adding vmin to the tree');
    addedVert=addedVert+1;
    cntId=cntId+1
    vmin.id=cntId;
    tau(addedVert).id=vmin.id;
    tau(addedVert).pose=vmin.pose;
    tau(addedVert).edgeq=vmin.edgeq;
    tau(addedVert).edgeu=vmin.edgeu;
    tau(addedVert).pid=vmin.pid;
    tau(addedVert).cost=vmin.cost;
    
    plot(vmin.pose(1),vmin.pose(2),'*b')

    
    
    
    %% plot the new edge
    if(PRINTITER)
    figure(10),plot(vmin.edgeq(:,1),vmin.edgeq(:,2))
    end
    
    
    
    %% If Goal return path
    
    if(norm(vmin.pose(1:2)-qgoal(1:2))<radiusGoal)
        disp('GOAL!! reached!!!')
        goalFound=1;
        p=extractpath(vmin,tau);
        u=getControls(vmin,tau);
        break;
        
    end
    
    
    
    
    
    
end


%% Plot Results
if(goalFound==1)
figure(10),hold on,
[npoints nc]=size(p);

for i=1:npoints
    drawrobot(p(i,:),'r',1,widthrobot,lengthrobot);
end



figure(11),hold on
subplot(2,1,1),plot(u(1,:)),title('v_{left}')
subplot(2,1,2),plot(u(2,:)),title('v_{right}')
end
