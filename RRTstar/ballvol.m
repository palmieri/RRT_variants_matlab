function val = ballvol(n, m)
% BALLVOL Compute volume of unit ball in R^n
%
% Computes the volume of the n-dimensional unit ball  
% using monte-carlo method.
% usage:   val = BallVol(n, m)
% where:     n = dimension 
%            m = number of realisations
% If the second argument is omitted, 1e4 is taken as default for m.

% (c) 1998, Rolf Krause, krause@math.fu-berlin.de

M = 1e4;
error = 0;
if(nargin <1 | nargin > 2), error('wrong number of arguments'); end
if nargin == 2, M = m; end 

R = rand(n, M);
in = 0;
for i=1:M
 if(norm(R(:,i),2) <= 1.0), in = in+1; end
end

val = 2^n*in/M;
