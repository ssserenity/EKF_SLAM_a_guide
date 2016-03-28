function [y,Y_p] = scan(p)
% SCAN perform a range?and?bearing measure of a 2D point.
%
% Input:
% p : point in sensor frame p = [px ; py]
% Output:
% y : measurement y = [range ; bearing]
% Y_p: Jacobian wrt p

px = p(1);
py = p(2);

d = sqrt(px^2+py^2);
a = atan2(py,px);
y = [d;a];
if nargout >1
    Y_p = [[px/sqrt(px^2+py^2),py/sqrt(px^2+py^2)]
           [-py/(px^2*(py^2/px^2+1)),1/(px*(py^2/px^2+1))]];
end
end

function f()
%% Symbolic code below ?? Generation and/or test of Jacobians
% ? Enable 'cell mode' to use this section
% ? Left?click once on the code below ? the cell should turn yellow
% ? Type ctrl+enter (Windows, Linux) or Cmd+enter (MacOSX) to execute
% ? Check the Jacobian results in the Command Window.
syms px py real
p = [px;py];
y = scan(p);
Y_p = jacobian(y,p)

[y,Y_p] = scan(p);
simplify(Y_p-jacobian(y,p))
% [0,0;0,0]
end