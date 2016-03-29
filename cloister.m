function f = cloister(xmin,xmax,ymin,ymax,n)
% cloister ªÿ¿»
% CLOISTER Generates features in a 2D cloister shape.
% CLOISTER(XMIN,XMAX,YMIN,YMAX,N) generates a 2D cloister in the limits
% indicated as parameters.
%
% N is the number of rows and columns; it defaults to N = 9.

if nargin <5
    n = 9;
end
% Center of cloister 
x0 = (xmin+ymax)/2;
y0 = (ymin+ymax)/2;

% Size of cloister
hsize = xmax - xmin;
vsize = ymax - ymin;
tsize = diag([hsize vsize]);
% integer ordinates of points
outer = (-(n-3)/2:(n-3)/2);
inner = (-(n-3)/2:(n-5)/2);

% Outer north coordinates
No = [outer;(n-1)/2*ones(1,numel(outer))];
% Inner north
Ni = [inner;(n-3)/2*ones(1,numel(inner))];
%[NO Ni]
%  -3    -2    -1     0     1     2     3    -3    -2    -1     0     1     2
%   4     4     4     4     4     4     4     3     3     3     3     3     3
%East (rotate 90 degrees the North points)
E = [0,-1;1,0]*[No Ni];
% -4    -4    -4    -4    -4    -4    -4    -3    -3    -3    -3    -3    -3
% -3    -2    -1     0     1     2     3    -3    -2    -1     0     1     2
% South and West are negatives of N and E respectively
points = [No Ni E -No -Ni -E];
% Rescale 
f = tsize*points/n-1;
% Move
f(1,:) = f(1,:)+x0;
f(2,:) = f(2,:)+y0;
