function [pw,PW_f,PW_pf] = fromFrame(F,pf)
%Express a local point pf from local frame F to global frame 
%input:
%F: reference frame F = [fx; fy; f_alpha]
%pf : point in frame F pf = [px; py]

%output:
%pw piont in gloal frame
%pw_f jacobian wrt F 
t = F(1:2);
a = F(3);
R = [cos(a) -sin(a);sin(a) cos(a)];
pw = R*pf + repmat(t,1,size(pf,2));

if nargout>1
    px = pf(1);
    py = pf(2);
    % pw = [cos(a) -sin(a) [px    [fx
    %                           + 
    %       sin(a) cos(a) ] py]    fy]
    % PW_f = d(pw)/d([fx,fy,a]')
    PW_f = [[1,0,-py*cos(a)-px*sin(a)]
            [0,1,px*cos(a)-py*sin(a)]];
    
    PW_pf = R;

end
end

function f()
%% Symbolic code below ?? Generation and/or test of Jacobians
% ? Enable 'cell mode' to use this section
% ? Left?click once on the code below ? the cell should turn yellow
% ? Type ctrl+enter (Windows, Linux) or Cmd+enter (MacOSX) to execute
% ? Check the Jacobian results in the Command Window.
syms x y a px py real
F = [x;y;a];
pf = [px;py];
pw = fromFrame(F,pf);
PW_f = jacobian(pw,F)
PW_pf = jacobian(pw,pf)
end