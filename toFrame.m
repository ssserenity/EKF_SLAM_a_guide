function[pf, PF_f, PF_p] = toFrame(F, p)
% express a global point in a local frame
%input:
%F = [f_x;f_y;f_alpha]  reference frame
%p= [p_x;p_y] point in gloal frame
%output:
%pf :point in Frame F
%PF_f: jacobian wrt F
%PF_p: jacobin wrt p

t = F(1:2);
a = F(3);

R=[cos(a) -sin(a);sin(a) cos(a)];

pf = R'*(p-t);

if nargout >1 
    px = p(1);
    py = p(2);
    x = F(1);
    y = F(2);
    
    % pf = R'*(p-t) = [cos(a) -sin(a);sin(a) cos(a)]'*(p-t)
    % PF_f = d(pf)/d[x,y,a]T
    PF_f = [...
           [-cos(a),-sin(a),cos(a)*(py-y)-sin(a)*(px-x)]
           [sin(a),-cos(a),-cos(a)*(px-x)-sin(a)*(py-y)]];
    %PF_p = d(pf)/d[px,py]T
    PF_p = R';
end
end

function f()
%% test of jacobian
% - check the jacobian
syms x y a px py real
F = [x y a]';
p = [px py]';
pf = toFrame(F, p);
PF_f = jacobian(pf,F)
PF_p = jacobian(pf,p)
% 在此处按 COMMAND+RETURN
end

