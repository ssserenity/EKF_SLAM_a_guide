function [p,P_r,P_y]=  invObserve(r,y)
% INVOBSERVE Backproject a range?and?bearing measurement and transform
% to map frame.
% 注意输入的传感器信息，所以是frame->global
% In:
% r : robot frame r = [rx ; ry ; ralpha]
% y : measurement y = [range ; bearing]
% Out:
% p : point in sensor frame
% P_r: Jacobian wrt r
% P_y: Jacobian wrt y


%nargout 是 number of arguement output,就是输出变量的数量
if nargout ==1 
    % invScan
    % INVSCAN Backproject a range?and?bearing measure into a 2D point.
    %
    % In:
    % y : range?and?bearing measurement y = [range ; bearing]
    % Out:
    % p : point in sensor frame p = [px ; py]
    % P_y: Jacobian wrt y
    % fromFrame 
    % FROMFRAME Transform a point PF from local frame F to the global frame.
    %
    % In:
    % F : reference frame F = [fx ; fy ; falpha]
    % pf: point in frame F pf = [pfx ; pfy]
    % Out:
    % pw: point in global frame
    % PW f: Jacobian wrt F
    p = fromFrame(r,invScan(y));
else
    [p_r,PR_y]= invScan(y);
    [p,P_r,P_pr] = fromFrame(r,p_r);
     P_y = P_pr * PR_y;
end
end

function f()
%% Symbolic code below ?? Generation and/or test of Jacobians
% ? Enable 'cell mode' to use this section
% ? Left?click once on the code below ? the cell should turn yellow
% ? Type ctrl+enter (Windows, Linux) or Cmd+enter (MacOSX) to execute
% ? Check the Jacobian results in the Command Window.
syms rx ry ra yd ya real
r = [rx;ry;ra];
y = [yd;ya];
[p, P_r, P_y] = invObserve(r, y); % We extract also the coded Jacobians P_r and P_y
% We use the symbolic result to test the coded Jacobians
%  注意 P_r ,P_y 是我们自己求的，jacobian(p,r)是matlab自己求的
simplify(P_r - jacobian(p,r)) % zero?matrix if coded Jacobian is correct
simplify(P_y - jacobian(p,y)) % zero?matrix if coded Jacobian is correct
end
    
    
    