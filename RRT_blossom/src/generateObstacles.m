%%  generateObstacles.m
% Script to generate Obstacles. Each obstacle is a circle situated at the
% position p with the radius r.
addpath('/media/data/software/librobotics')



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

%% parameter
ndim=2;
dim= [0 5 0 5 0 2*pi];
maxradius=0.6;
minradius=0.2;
widthrobot=0.4;
lengthrobot=0.6;
qinit=[2;1;0];
qgoal=[4;3.5;0];
Nobs=10
radiusGoal=1;
radiusInit=1.5;

%% generate obstacles
for i=1:Nobs
for j=1:ndim
pobst(i,j)=  dim(2*j-1)+(dim(2*j) -dim(2*j-1))*rand();  
if((norm(qinit(1:2)'-pobst(i,j))>radiusInit) & (norm(qgoal(1:2)'-pobst(i,j))>radiusGoal))
obst(i,j)=pobst(i,j);
else
    continue
end

end
obst(i,ndim+1)=minradius+(maxradius-minradius)*rand();
end


figure(40),hold on


%% plot initial position
drawrobot(qinit,'b',2,widthrobot,lengthrobot)


%% plot goal region
drawellipse(qgoal,radiusGoal,radiusGoal,'b');

%% plot obstacles
for i=1:Nobs
        drawellipse([obst(i,1); obst(i,2) ;0],obst(i,3),obst(i,3),'r');
    
end


dlmwrite('./environment.txt', obst);