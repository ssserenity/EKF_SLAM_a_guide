function [pw,PW_f,PW_pf] = fromFrame(F,pf)
%Express a local point pf from local frame F to global frame 
%input:
%F: reference frame F = [f_x; f_y; f_alpha]
%pf : point in frame F pf = [pf_x; pf_y]

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