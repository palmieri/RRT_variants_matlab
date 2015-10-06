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
  rndvec = randperm(100);
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
BLOSSOM=1


global addedVert
% size robot

global widthrobot
global lengthrobot

widthrobot=0.4;
lengthrobot=0.6;


% configuration space
N=3;
dim= [0 5 0 5 0 2*pi];


% tree
tau={};
% max number of iterations
I=500;

% initial and end pose
qinit=[2;1;0];
qgoal=[4;3.5;0];
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

obs(2).position=[1;1];
obs(2).radius=0.51;

obs(3).position=[1;3];
obs(3).radius=0.51;


obs(4).position=[1;2.5];
obs(4).radius=0.51;

obs(5).position=[1;5];
obs(5).radius=0.51;

obs(5).position=[4;3];
obs(5).radius=0.21;


end






[h nobs]=size(obs);
figure(10),hold on

%% plot environment
for i=1:nobs
        drawellipse([obs(i).position;0],obs(i).radius,obs(i).radius,'r');
    
end

%% plot initial position
drawrobot(qinit,'b',2,widthrobot,lengthrobot);


%% plot goal region
drawellipse(qgoal,radiusGoal,radiusGoal,'b');


%%  motion primitives array
motionPrimitiveCommandArray = [1 0; 1 1.3; 1 -1.3; 0.8 0.3; 0.8 -0.3; 1 2; 1 -2];
delta=0.25;


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


addedVert=1;
collision=0;
goalFound=0;

% for i=1:I
    while(goalFound<1)
    
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
    
    
    
    %% Nearest Vertex
    
    vnear=nearesttree(tau,qrand);
    figure(10)
    hv=plot(vnear.pose(1),vnear.pose(2),'*r');
    
    
    
    %% Extend
    [vnew, tau]=extendblossom(tau,obs,vnear,qrand,motionPrimitiveCommandArray,delta,PRINTITER);
    
    

    if(isempty(vnew.edgeq))
        continue;
    end
    
    
    
    

    
    
    %% If Goal return path
    
    if(norm(vnew.pose(1:2)-qgoal(1:2))<radiusGoal)
        disp('GOAL!! reached!!!')
        goalFound=1;
        p=extractpath(vnew,tau);
        u=getControls(vnew,tau);
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
subplot(2,1,1),plot(u(:,1)),title('v')
subplot(2,1,2),plot(u(:,2)),title('w')
end
