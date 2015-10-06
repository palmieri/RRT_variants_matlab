%POSCONTROLSTEP   Robot control function
%   POSCONTROLSTEP(T, XCURRENT, XEND, DIR, B) is the control function which
%   is called after every timer tick by PositionController.
%
%   Where:
%   t        : Current simulation time (starts at 0)
%   xcurrent : 3x1-vector of the current pose [x; y; theta]
%   xend     : 3x1-vector of the goal [x; y; theta]
%   dir      : optional direction of motion (1 forward (default),
%              -1 backward, 0 automatic)
%   b        : b of robot
%   vl, vr   : Left/right wheel velocity
%   eot      : Boolean value. 1 if end of trajectory
%
%   See also PositionController

% Based on the position controller by Allesandro Astolfi, ETHZ
% (c) 1998 R�my Blank EPFL-ASL / Modified 2000.10.09 Gilles Caprari EPFL-ASL
% v.1.0, Kai Arras, EPFL-ASL
% v.1.1, Jan Weingarten, Kai Arras: wheelbase argument B added
% v.1.1.1, Kai Arras, minor modifications.
% v 1.3  Luigi Palmieri, waypoints management added, new controller



function [vl, vr, eot,vm,vd] = PosControlStepWP(t, xcurrent, xend, dir, b)
global CtrlType
persistent oldBeta controllerType;
CtrlType=1;


Kv=3.8;
Krho    = 1;                  % Condition: Kalpha + 5/3*Kbeta - 2/pi*Krho > 0 !
Kalpha  = 6;    
Kbeta   = -1;
Kphi    = -1;

Vmax    =5;                % [m/s]
if(CtrlType==1)
RhoEndCondition = 0.35;      % [m]
else
RhoEndCondition = 0.00000510;      % [m]
end
PhiEndCondition = 10*pi/180;

Verbose = 0;

if t == 0									    % First call in loop
    oldBeta = 0;
    controllerType = 1;			  % Controller type for endpoint hysteresis
end;

% extract coordinates
xc = xcurrent(1); yc = xcurrent(2); tc = xcurrent(3);
xe = xend(1); ye = xend(2); te = xend(3);

% rho
dx = xe - xc;
dy = ye - yc;
rho = sqrt(dx^2 + dy^2);
fRho = rho;


if fRho > (Vmax/Krho),
    fRho = Vmax/Krho;
end;

% alpha
alpha = atan2(dy, dx) - tc;
alpha = normangle(alpha, -pi);


% direction
if dir == 0							  % controller choose the forward direction
    if alpha > pi/2
        fRho = -fRho;					% backwards
        alpha = alpha-pi;
    elseif alpha <= -pi/2
        fRho = -fRho;					% backwards
        alpha = alpha+pi;
    end;
elseif dir == -1					% arrive backwards
    fRho = -fRho;
    alpha = alpha+pi;
    if alpha > pi
        alpha = alpha - 2*pi;
    end;
else
    % arrive forward
end;

% phi
phi = te-tc;
phi = normangle(phi, -pi);

beta = normangle(phi-alpha, -pi);
if abs(oldBeta-beta) > pi			% avoid instability
    beta = oldBeta;
end;
oldBeta = beta;



% New version
vm= Krho*f(fRho,Kv);
vd=(Kalpha*alpha + Kbeta*beta);

% set speed
% Original version of Astolfi's Paper
% vm = Krho*fRho;
% vd = Kalpha*alpha + Kbeta*beta;


% eot = (rho < RhoEndCondition) & (abs(phi) < PhiEndCondition);
% eot = (rho < RhoEndCondition);
if(CtrlType==1)
eot = (rho < RhoEndCondition);      
else
eot = (rho < RhoEndCondition) & (abs(phi) < PhiEndCondition);
end



if eot,
    if Verbose,
        disp(sprintf('t: %G sec  x: %G  y: %G  theta: %G',t, xc, yc, tc*180/pi));
    end;
end;

% Convert speed to wheel speeds
vl = vm - vd*b/2;
if abs(vl) > Vmax
    vl = Vmax*sign(vl);
end;

vr = vm + vd*b/2;
if abs(vr) > Vmax
    vr = Vmax*sign(vr);
end;



end


function vf=f(Rho,Kv)


        kv=Kv;
        vf=tanh(Rho*kv);

end



% Values for Pygmalion
% ====================
% krho      = 1.0;							% Value of diploma work: krho = 3.0;
% kalpha    = 4.0;							% Value of diploma work: kalpha = 8.0;
% kphi      = -1.0;	  					% Value of diploma work: kphi = -1.0;
% rhoThreshold1 = 0.005;       	% Zielkreis, in [m]
% rhoThreshold2 = 0.05;   		  % Kreis in dem nur Winkelkorrektur, in [m]
% rhoThreshold3 = 0.005;   			% Umschaltkreis auf rho/phi-Regler (ohne alpha), in [m]
% phiThreshold  = 0.01744/10;   % Winkelgenauigkeit bei reiner Winkelkorrektur, in [rad]