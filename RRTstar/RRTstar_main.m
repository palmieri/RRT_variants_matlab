% University of Freiburg, Germany
% (c) SRL 2016
% Author: Luigi Palmieri
% This script demonstrates the basic funcionalities of RRT* for a 2D system
% The tree is an array of struct with vertices defined as follows:
%   v.id    -- ID of the vertex
%   v.pid   -- parent ID
%   v.pose  -- Pose of the vertex
%   v.edgeq -- edge where to save intermediate configurations
%   v.edgeu -- edge where to save intermediate controls u

clear all; clc; close all;

% Include the librobotics 
addpath('/home/palmieri/Documents/MATLAB/librobotics/librobotics');

% Set random seed
% rng('shuffle','twister');
rng(9,'twister');

% Parameters
ROBOTRADIUS = 0.2;          % Radius (not diameter!) of circular robot in [m]
LIMITS = [0 5 0 5 0 2*pi];  % lower and upper limits of C-space in [m,m,rad]
GOALRADIUS = 0.3;           % radius of the goal region in [m]
PRINT = 1;
PRINT_OUT_INFOS = 0;
MAX_ITERATIONS = 1000;
% Initial and goal configurations
qinit = [2; 1];   % initial pose
qgoal = [2; 2.2];   % goal pose
n_dim = 2; % dimension of the state space
volume_xfree = (LIMITS(2)-LIMITS(1)*LIMITS(4)-LIMITS(3)); % at the moment hard coded
volumeBall = ballvol(n_dim, 1000); % volume of the unit ball given the current dimension
gamma = 5*((volume_xfree)/volumeBall)^(1/n_dim); % gamma param for RRT*


% ----------------------------------------------- %
%  Plot environment
% -------------------------------e---------------- %
figure(1); clf; hold on; box on; axis equal;
axis([LIMITS(1)-0.5 LIMITS(2)+0.5 LIMITS(3)-0.5 LIMITS(4)+0.5]);

obstacles = load('environment.txt');
nobs = size(obstacles,1);

angles = linspace(0,2*pi,100);
for i = 1:nobs,
  x = obstacles(i,1) + obstacles(i,3)*cos(angles);
  y = obstacles(i,2) + obstacles(i,3)*sin(angles);
  fill(x,y,[.4 .4 .4],'EdgeColor','none');
end;

% plot initial and goal poses
drawrobot([qinit; 0],'k',1,2*ROBOTRADIUS,2*ROBOTRADIUS);
drawellipse([qgoal; 0],GOALRADIUS,GOALRADIUS,'g');


% ----------------------------------------------- %
% RRT* Steps
% ----------------------------------------------- %
% Figure handles
hedge_parent = [];
hrand = [];
hnear = [];
hnearVertices = [];
htree = [];
hbestparent = [];
% Initialize tree with qinit
tau(1).id = 1;
tau(1).pose = qinit;
tau(1).edgeq = [];
tau(1).g = 0;
tau(1).edgeu = [];
tau(1).pid = 0;
nvertices = 1;

% Main loop
q_in_goal = [];
goalfound = false;
k = 1;
while (~goalfound || k<MAX_ITERATIONS),
  k=k+1;
  disp(['Iteration: ' num2str(k)])
  % Sample new configuration and check collision
  qrand = sampleconfiguration(LIMITS);
  
  if ~checkcollision(qrand,ROBOTRADIUS,obstacles),
    hrand = plot(qrand(1),qrand(2),'r+','MarkerSize',16);
    drawnow;
    
    % Find nearest vertex
    qnear = findnearestvertex(tau,qrand);
    hnear = plot(qnear.pose(1),qnear.pose(2),'m.','MarkerSize',25);
    drawnow;
    
    % Steer and check collisions on the edge
    qnew = steer(qnear,qrand);
    hedge = plot(qnew.edgeq(:,1),qnew.edgeq(:,2),'g-','LineWidth',2);
    drawnow;

    if ~checkedgecollision(qnew,ROBOTRADIUS,obstacles),
      
      % Save qnew in the tree
      nvertices = nvertices + 1;
      tau(nvertices).id = nvertices;
      tau(nvertices).pose = qnew.pose;
      tau(nvertices).edgeq = qnew.edgeq;
      tau(nvertices).g = computePathLengthCost(qnew.edgeq) + tau(qnew.pid).g;
      tau(nvertices).edgeu = qnew.edgeu;
      tau(nvertices).pid = qnew.pid;
      
      % Plot the new edge
      if exist('hedge','var'), delete(hedge); hedge=[]; end;
      htree(nvertices-1) = plot(tau(nvertices).edgeq(:,1),tau(nvertices).edgeq(:,2),'b-','LineWidth',2);
      
      drawnow;
      
      % Connect to the best parent
      % selects the near vertices
       if exist('hnear','var'), delete(hnear);  hnear=[]; end;

        [aa size_tree] = size(tau);
        radius_rrtstar = gamma * (1/volumeBall*log(size_tree)/size_tree)^(1/n_dim);
        qNear = findNearVertices(tau, tau(nvertices), radius_rrtstar);
        hnearVertices_i = drawcircle(tau(nvertices).pose(1), tau(nvertices).pose(2), radius_rrtstar);
        hnearVertices = [ hnearVertices_i ;hnearVertices];
        
       % choose the new best parent
        [nr n_near_q] = size(qNear);
        curr_tree_cost_to_come = tau(nvertices).g;
        curr_parent_tree_id = tau(nvertices).pid;
        best_in_qnearvertices_id = -1;
        best_in_qnearvertices_conf = [];
        best_in_qnearvertices_parent_pose = tau(tau(nvertices).pid).pose;
        for qi=1:(n_near_q)
            q_curr_parent = steer(qNear(qi), tau(nvertices).pose);

            hnear_i = plot(qNear(qi).pose(1),qNear(qi).pose(2),'g.','MarkerSize',25);
            hnear = [hnear_i hnear];
            hedge_parent_i = plot(q_curr_parent.edgeq(:,1),q_curr_parent.edgeq(:,2),'m-','LineWidth', 2);
            hedge_parent = [ hedge_parent_i  hedge_parent];
            
            if ~checkedgecollision(q_curr_parent,ROBOTRADIUS,obstacles)
                curr_cost = computePathLengthCost(q_curr_parent.edgeq) + qNear(qi).g;
                
                if(PRINT_OUT_INFOS)
                    disp(['Current Cost: ' num2str(curr_cost) ', curr_tree_cost_to_come: ' num2str(curr_tree_cost_to_come)])
                end
                
                if(curr_cost<curr_tree_cost_to_come)
                    curr_tree_cost_to_come = curr_cost;
                    curr_parent_tree_id = qNear(qi).id;
                    best_in_qnearvertices_id = qi;
                    best_in_qnearvertices_conf = q_curr_parent;
                    best_in_qnearvertices_parent_pose = qNear(qi).pose;
                end

            end
        end
        
        
        if(best_in_qnearvertices_id ~= -1)
             if(PRINT_OUT_INFOS) 
                 disp(['Removing old parent:' num2str(tau(tau(nvertices).pid).pose(1)) ' ' num2str(tau(tau(nvertices).pid).pose(2))  ' ' num2str(tau(nvertices).pid) ' with cost:' num2str(tau(nvertices).g)]) 
             end
            tau(nvertices).pid = curr_parent_tree_id;
            tau(nvertices).edgeq = best_in_qnearvertices_conf.edgeq;
            tau(nvertices).edgeu = best_in_qnearvertices_conf.edgeu;
            tau(nvertices).g = curr_tree_cost_to_come;
            tau(tau(nvertices).pid).pose = best_in_qnearvertices_parent_pose;
            
            if(PRINT_OUT_INFOS) 
                 disp(['Adding new parent:' num2str(tau(tau(nvertices).pid).pose(1)) ' ' num2str(tau(tau(nvertices).pid).pose(2))  ' ' num2str(tau(nvertices).pid) ' with cost:' num2str(tau(nvertices).g)]) 
            end
            delete(htree(nvertices-1))
            drawnow 
            htree(nvertices-1) = plot(tau(nvertices).edgeq(:,1),tau(nvertices).edgeq(:,2),'b-','LineWidth',2);
            drawnow 

            hbestparent = plot(best_in_qnearvertices_parent_pose(1),best_in_qnearvertices_parent_pose(2),'m.','MarkerSize',25);
        end
        
        if exist('hedge_parent','var'),  delete(hedge_parent); hedge_parent = []; end;
        if exist('hnear','var'), delete(hnear);  hnear=[]; end;

        drawnow 
        
        if exist('hbestparent','var'), delete(hbestparent);  hbestparent=[]; end;

        % Rewire
        [aa size_tree] = size(tau);
        radius_rrtstar = gamma * (1/volumeBall*log(size_tree)/size_tree)^(1/n_dim);
        qNear = findNearVertices(tau, tau(nvertices), radius_rrtstar);
        hnearVertices_i = drawcircle(tau(nvertices).pose(1), tau(nvertices).pose(2), radius_rrtstar);
        hnearVertices = [ hnearVertices_i ;hnearVertices];
        
        % For plotting before the nodes in qnear
        for qi=1:(n_near_q)
            hnear_i = plot(qNear(qi).pose(1),qNear(qi).pose(2),'y.','MarkerSize',25);
            hnear = [hnear_i hnear];    
        end
        
        
        for qi=1:(n_near_q)
            q_curr_parent = steer(tau(nvertices), qNear(qi).pose);
            hnear_i = plot(qNear(qi).pose(1),qNear(qi).pose(2),'y.','MarkerSize',25);
            hnear = [hnear_i hnear];
            hedge_parent_i = plot(q_curr_parent.edgeq(:,1),q_curr_parent.edgeq(:,2),'m-','LineWidth', 2);
            hedge_parent = [ hedge_parent_i  hedge_parent];
   
            if exist('hedge_parent','var'),  delete(hedge_parent); hedge_parent = []; end;

            if ~checkedgecollision(q_curr_parent, ROBOTRADIUS, obstacles)
                curr_cost = tau(nvertices).g + computePathLengthCost(q_curr_parent.edgeq);
                if(PRINT_OUT_INFOS) 
                    disp(['Current Cost: ' num2str(curr_cost) ', qNear(qi) cost: ' num2str(qNear(qi).g)]) 
                end
                
                if(curr_cost < qNear(qi).g)
                    
                    if(PRINT_OUT_INFOS) 
                        disp(['Rewiring old parent:' num2str(tau(tau(qNear(qi).id).pid).pose (1)) ' ' num2str(tau(tau(qNear(qi).id).pid).pose (2))  ' ' num2str(tau(qNear(qi).id).pid) ' with cost:' num2str(tau(qNear(qi).id).g)]) 
                    end

                    tau(qNear(qi).id).pid = tau(nvertices).id;
                    tau(qNear(qi).id).g = curr_cost;
                    tau(qNear(qi).id).edgeq =  q_curr_parent.edgeq;
                    tau(qNear(qi).id).edgeu =  q_curr_parent.edgeu;   
                    tau(tau(qNear(qi).id).pid).pose = tau(nvertices).pose;
                    
                    if(PRINT_OUT_INFOS) 
                        disp(['Rewiring new parent:' num2str(tau(tau(qNear(qi).id).pid).pose (1)) ' ' num2str(tau(tau(qNear(qi).id).pid).pose (2))  ' ' tau(qNear(qi).id).pid ' with cost:' num2str(tau(qNear(qi).id).g)]) 
                    end

                    delete(htree(qNear(qi).id-1))
                    drawnow 
                    htree(qNear(qi).id-1) = plot(tau(qNear(qi).id).edgeq(:,1),tau(qNear(qi).id).edgeq(:,2),'b-','LineWidth',2);
                    drawnow 
                end

            end
        end

        drawnow 
       
      
      % Check if goal reached
      if norm(tau(nvertices).pose(1:2)-qgoal(1:2)) < GOALRADIUS
        if(isempty(q_in_goal))
            q_in_goal = tau(nvertices);
            disp('First time, Goal reached!!!')
        else
            if(tau(nvertices).g<q_in_goal.g)
                q_in_goal = tau(nvertices);
                disp('Better path in goal area!!!')
            end
        end
        goalfound = true;
      end

    else
      set(hedge,'Color','r');
      
      
    end;
    drawnow;
    
    if exist('hedge_parent','var'),  delete(hedge_parent); hedge_parent = []; end;
    if exist('hrand','var'), delete(hrand); end;
    if exist('hnear','var'), delete(hnear);  hnear=[]; end;
    if exist('hedge','var'), delete(hedge); end;
    if exist('hnearVertices','var'), delete(hnearVertices); hnearVertices = []; end;
    
  end; % 
  

end; % while


if goalfound,
  % Extract path and controls
  [p,u] = extractpath(q_in_goal,tau);
  npoints = size(p,1);
  plot(p(:,1),p(:,2),'r*')
else
   disp(['No path found in ' num2str(k) ' iterations'])
end;
