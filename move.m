function [ro,RO_r,RO_n] = move(r,u,n)
% MOVE Robot motion, with separated control and perturbation inputs.
%
% In:
% r: robot pose r = [x ; y ; alpha]
% u: control signal u = [dx ; dalpha]
% n: perturbation, additive to control signal
% Out:
% ro:    updated robot pose
% RO_r: Jacobian d(ro) / d(r)
% RO_n: Jacobian d(ro) / d(n)

a = r(3);
dx = u(1)+n(1);
da = u(2)+n(2);
ao = a+da;

if ao > pi
    ao = ao - 2*pi;
end
if ao < -pi
    ao = ao + 2*pi;
end

% build position increment dp=[dx;dy], from control signal dx
dp = [dx;0];
if nargout ==1 % No Jacobians requested
    to = fromFrame(r,dp);
else
    [to,TO_r,TO_dt] = fromFrame(r,dp);
    AO_a =1;
    AO_da =1;
    % to = R*dp+r(1:2)
    % ao = a+da
    %RO_r = d([to;ao])/d(r)=[TO_r;0,0,AO_r]
    RO_r = [TO_r;0 0 AO_a];
    % to =  R*dp+r(1:2) = R*(u+n)+r(1:2) = R*([u(1)+n(1),0])+r(1:2)
    % ao =  a+da = a+u(2)+n(2)
    % RO_n = RO_dp*(dp/dn) = [TO_dt;0,0,Ro_da]*[1,0,0;0,0,0;0,0;1]
    RO_n = [TO_dt(:,1) zeros(2,1);0 AO_da];
end
ro = [to;ao];
 
    