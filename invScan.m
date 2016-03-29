function [p,P_y] = invScan(y)
% INVSCAN Backproject a range?and?bearing measure into a 2D point.
%
% In:
% y : range?and?bearing measurement y = [range ; bearing]
% Out:
% p : point in sensor frame p = [px ; py]
% P_y: Jacobian wrt y


d = y(1);
a = y(2);
px = d*cos(a);
py = d*sin(a);

p = [px;py];

if nargout>1 
    P_y = [cos(a),-d*sin(a);sin(a),d*cos(a)];
end    
end

function f()
%% test
syms yd ya real
y = [yd;ya];
[p,P_y] = invScan(y);
simplify(P_y-jacobian(p,y))
end