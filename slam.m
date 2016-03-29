%% SLAM2D A 2D EKF?SLAM algorithm with simulation and graphics.
%
% HELP NOTES:
% 1. The robot state is defined by [xr;yr;ar] with [xr;yr] the position
%    and [ar] the orientation angle in the plane.
% 2. The landmark states are simply Li=[xi;yi]. There are a number of N
%    landmarks organized in a 2?by?N matrix W=[L1 L2 ... Ln]
%    so that Li = W(:,i).
% 3. The control signal for the robot is U=[dx;da] where [dx] is a forward
%    motion and [da] is the angle of rotation.
% 4. The motion perturbation is additive Gaussian noise n=[nx;na] with
%    covariance Q, which adds to the control signal.
% 5. The measurements are range?and?bearing Yi=[di;ai], with [di] the
%    distance from the robot to landmark Li, and [ai] the bearing angle from
%     the robot's x?axis.

% 6. The simulated variables are written in capital letters,
%    R: robot
%    W: set of landmarks or 'world'
%    Y: set of landmark measurements Y=[Y1 Y2 ... YN]

% 7. The true map is [x;l1;l2;l3;...lN] = [xr;yr;ar;x1;y1;x2;y2;x3;y3; ... ;xN;yN]

% 8. The estimated map is Gaussian, defined by
%    x: mean of the map
%    P: covariances matrix of the map

% 9. The estimated entities (robot and landmarks) are extracted from {x,P}
%    via pointers, denoted in small letters as follows:
%    r: pointer to robot state. r=[1,2,3]
%    l: pointer to landmark i. We have for example l=[4,5] if i=1,
%    l=[6,7] if i=2, and so on.
%    m: pointers to all used landmarks.
%    rl: pointers to robot and one landmark.
%    rm: pointers to robot and all landmarks (the currently used map).
%    Therefore: x(r) is the robot state,
%               x(l) is the state of landmark i
%               P(r,r) is the covariance of the robot
%               P(l,l) is the covariance of landmark i
%               P(r,l) is the cross?variance between robot and lmk i
%               P(rm,rm) is the current full covariance ?? the rest is unused.
%    NOTE: Pointers are always row?vectors of integers.

% 10. Managing the map space is done through the variable mapspace.
%      mapspace is a logical vector the size of x. If mapspace(i) = false,
%      then location i is free. Otherwise mapspace(i) = true. Use it as
%      follows:
%             * query for n free spaces: s = find(mapspace==false, n);
%             * block positions indicated in vector s: mapspace(s) = true;
%             * liberate positions indicated in vector s: mapspace(s) = false;
%      example: mapspace = [1,0,0,0,0,1,1,0,1,0,1,0]
%               s = find (mapspace==false, )= [2,3,4,5,8,10,12]

% 11. Managing the existing landmarks is done through the variable landmarks.
%     landmarks is a 2?by?N matrix of integers. l=landmarks(:,i) are the
%     pointers of landmark i in the state vector x, so that x(l) is the
%     state of landmark i. Use it as follows:
%             * query 1 free space for a new landmark: i = find(landmarks(1,:)==0,1)
%             * associate indices in vector s to landmark i: landmarks(:,i) = s
%             * liberate landmark i: landmarks(:,i) = 0;

% 12. Graphics objects are Matlab 'handles'. See Matlab doc for information.
% 13. Graphic objects include:
%       RG: simulated robot
%       WG: simulated set of landmarks
%       rG: estimated robot
%       reG: estimated robot ellipse
%       lG: estimated landmarks
%       leG: estimated landmark ellipses

%% I.INTIALIZE
%  I.1 SIMULATOR  --use captial letters for variable names
%  W: set of external landmarks
W = cloister(-4,4,-4,4,7);% Type 'help cloister' for help
% N :number of landmarks
N = size(W,2);
% R: robot pose [x;y;alpha]
R=[0;-2;0];
% U: control [dx,dalpha]
U=[0.1;0.05]; % fixing advance and turn increments creates a circle
% Y :measurements of all landmarks
Y = zeros(2,N)

% I.2 ESTIMATOR 
% Map:Gaussian {x,P}
% x :state vector's mean
% P :state vector's covariance matrix
% add by zx:
% x = [R M]' = [R L1 ... Ln]'
% P = [Prr Prm ; Pmr Pmm]=[Prr Prl1...Prln; Pl1r Pl1l1...Pl1ln;... ... ...;Plnr Plnl1 ... Plnln]


x = zeros(numel(R)+numel(W),1);%numel: Number of elements in an array or subscripted array expression.
                              % N = numel(A) returns the number of elements, N
p = zeros(numel(x),numel(x));

% System noise: Gaussian {0,Q}
q = [.01;.02]; % amplitude or standard deviation
Q = diag(q.^2); % covariances matrix

% Measurement noise: Gaussian {0,S}
s = [.1;1*pi/180]; % amplitude or standard deviation
S = diag(s.^2); % covariances matrix

% Map management 
mapspace = false(1,numel(x)); %see the 10 above
   
%Landmarks mangament 
landmarks = zeros(2,N); %see the 11 above

%Place robot in map
r = find(mapspace==false,numel(R)); %set robot pointer
mapspace(r) = true;                 %block map positions
x(r)  = R;                          %initialize robot states
P(r,r) = 0;                         %initialize robot covariance 

%% I.3 GRAPHICS -- USE the variable names of simulated and estimated
% variables,followed by a captial G to indicate 'graphics'
% NOTE :the graphics code is long but absolutely necessary
% 毫无技术含量的画图
% Set figure and axes for Map
mapFig = figure(1); % create figure
cla % clear axes
axis([-6 6 -6 6]) % set axes limits
axis square % set 1:1 aspect ratio

% Simulated World -- set of all landmarks,red crosses
WG = line(...
'linestyle','none',...
'marker','+',...
'color','r',...
'xdata',W(1,:),...
'ydata',W(2,:));

%Simulated robot ,red triangle
% 画一个三角形 代表 robot
Rshape0 = 0.2*[2 -1 -1 2;0 1 -1 0];% a triangle at the origin
% 用同时转换多点坐标的方法
Rshape = fromFrame(R,Rshape0);     % a triangle at the robot pose
RG = line('linestyle','-',...
'marker','none',...
'color','r',...
'xdata',Rshape(1,:),...
'ydata',Rshape(2,:));
% Estimated robot, blue triangle
rG = line('linestyle','-',...
'marker','none',...
'color','b',...
'xdata',Rshape(1,:),...
'ydata',Rshape(2,:));
% Estimated robot ellipse, magenta
reG = line('linestyle','-',...
'marker','none',...
'color','m',...
'xdata',[ ],...
'ydata',[ ]);
% Estimated landmark means, blue crosses
lG = line('linestyle','none',...
'marker','+',...
'color','b',...
'xdata',[ ],...
'ydata',[ ]);
% Estimated landmark ellipses, green
leG = zeros(1,N);
for i = 1:numel(leG)
leG(i) = line('linestyle','-',...
'marker','none',...
'color','g',...
'xdata',[ ],...
'ydata',[ ]);
end

%% II. TEMPORAL LOOP
for t = 1:200
    %II.1 SIMULATOR
    % a.motion
    
    n = q.*randn(2,1);%preturbation vector
    R = move(R,U,zeros(2,1));% we will perturb the estimator
                             % instead of the simulator
    %b.observations
    for i = 1:N              % i: landmark index
        v = s.*randn(2,1);   % measurement noise
        Y(:,i) = observe(R,W(:,i))+v; 
    end
    %II.2 ESTIMATOR
    % a. create dynamic map pointers to be used hereafter
    m = landmarks(landmarks~=0)'; % all pointers to landmarks
    rm = [r , m]; % all used states: robot and landmarks
                   % ( also OK is rm = find(mapspace); )
    % b. Prediction -- robot motion
    [x(r),R_r,R_n] = move(x(r),U,n);% Estimator perturbed with n
    P(r,m) = R_r*P(r,m);
    P(m,r) = P(r,m)';
    P(r,r) = R_r*P(r,r)*R_r'+R_n*Q*R_n';
    
    %c.Landmark correction -- know landmarks
    lids = find(landmarks(1,:)); % returns all indices of existing landmarks
    for i = lids
        %expectation:Gaussian{e,E}
        l = landmarks(:,i)';      % landmark pointer
        [e,E_r,E_l]  = observe(x(r),x(l)); % this is h(x) in EKF
        rl = [r,l]; %pointers to robot and lmk.
        E_rl = [E_r,E_l]; %expection Jacobian
        E = E_rl*P(rl,rl)*E_rl';
        % measurement of landmark i
        Yi = Y(:,i);
        % innovation: Gaussian {z,Z}
        z = Yi - e; % this is z = y ? h(x) in EKF
        % we need values around zero for angles:
        if z(2) > pi
        z(2) = z(2) - 2*pi;
        end
        if z(2) < -pi
        z(2) = z(2) + 2*pi;
        end
        Z = S + E;
        % Individual compatibility check at Mahalanobis distance of 3?sigma
        % (See appendix of documentation file 'SLAM course.pdf')
        if z'*Z^-1*z<9
            %Kalman gain
            % this is K = P*H'*Z??1 in EKF
            % K = [Prr Prl;Pmr PmL]*[Hr';Hl']*Z^-1
            K = P(rm,rl)*E_rl'*Z^-1;  
            
            % map update (use pointer rm)
            x(rm) = x(rm)+ K*z;           %x <- x+Kz
            P(rm,rm)= P(rm,rm)- K*Z*K';   % P<- P-KZK'
        end
    end
    
    % d. Landmark Initialization ?? one new landmark only at each iteration
    % Landmark initialization happens when the robot discovers landmarks that are not yet
    % mapped and decides to incorporate them in the map. As such, this operation results
    % in an increase of the state vector’s size. The EKF becomes then a filter of a state of
    % dynamic size.
    
    lids = find(landmarks(1,:)==0); % all non?initialized landmarks
    if ~isempty(lids)               % there are still landmarks to initialize
        i = lids(randi(numel(lids))); % pick one landmark randomly, its index is i
        l = find(mapspace == false,2) % pointer of the new landmark in the map
        if ~isempty(l)
            mapspace(l) = true;     % block map space
            landmarks(:,i) = l;     % store landmark pointers
            % measurement
            Yi = Y(:,i);
            %initialization
            
            [x(l),L_r,L_y]  = invObserve(x(r),Yi);
            P(l,rm) = L_r*P(r,rm);
            P(rm,l) = P(l,rm)';
            P(l,l)  = L_r*P(r,r)*L_r'+L_y*S*L_y';
        end
    end
    %II.3 GRAPHICS
    
    % Simulated robot
    Rshape = fromFrame(R,Rshape0);
    set(RG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    % Estimated robot
    Rshape = fromFrame(x(r), Rshape0);
    set(rG, 'xdata', Rshape(1,:), 'ydata', Rshape(2,:));
    % Estimated robot ellipse
    re = x(r(1:2)); % robot position mean
    RE = P(r(1:2),r(1:2)); % robot position covariance
    [xx,yy] = cov2elli(re,RE,3,16); % x? and y? coordinates of contour
    set(reG, 'xdata', xx, 'ydata', yy);
    % Estimated landmarks
    lids = find(landmarks(1,:)); % all indices of mapped landmarks
    lx = x(landmarks(1,lids)); % all x-coordinates
    ly = x(landmarks(2,lids)); % all y-coordinates
    set(lG, 'xdata', lx, 'ydata', ly);
    % Estimated landmark ellipses ?? one per landmark
    for i = lids
        l = landmarks(:,i);
        le = x(l);
        lE = P(l,l);
        [xx,yy] = cov2elli(le,lE,3,16);
        set(leG(i), 'xdata', xx, 'ydata', yy);
    
    end
    
    % force Matlab to draw all graphic objects before next iteration
    drawnow
    % pause(1)
end
