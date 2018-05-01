%-------------------------------------------------------------------------
%-------------------------------------------------------------------------
% Inputs
%-------------------------------------------------------------------------
%   X0 = [x0, y0, th0, K0];
%   - x0, initial x position, m
%   - y0, intial y position, m
%   - th0, initial heading (angle w.r.t. positive x axis), rad
%   - K0, initial kappa (curvature) value, rad
%   Xf = [x0, y0, th0, K0];
%   - xf, final x position, m
%   - yf, final y position, m
%   - thf, final heading (angle w.r.t. positive x axis), rad
%   - Kf, final kappa (curvature) value, rad
%   Xobst = [xobst1, xobst2, ..., xobstn];
%   - xobst1, x coordinate of point obstacle 1, m
%   Yobst = [yobst1, yobst2, ..., yobstn];
%   - yobst1, y coordinate of point obstacle 1, m
%   ObstacleAvoidanceBool
%   - 1: perform obstacle avoidance
%   - 0: do not perform obstacle avoidance
%-------------------------------------------------------------------------
%-------------------------------------------------------------------------
% Outputs
%-------------------------------------------------------------------------
%   PlannedPath
%   - 3xn array of points forming path from (x0,y0) to (xf,yf)
%   - x(s), m
%   - y(s), m
%   - th(s), rad
%   Stats
%   - 5x2 cell array of convergence statistics
%   - PassBool = 1 if path planning converged
%   - error in X posn, m
%   - error in Y posn, m
%   - error in heading, rad
%   - error in curvature, 1/m
%-------------------------------------------------------------------------
%-------------------------------------------------------------------------

function [PlannedPath,Stats] = PathPlanning(X0,Xfdes,Xobst,Yobst,ObstacleAvoidanceBool)

% Define convergence tolerance
X_MinErr = [0.01; 0.01; 0.1; 0.01]; % x(m), y(m), th(rad), K(1/m)

% Translate function inputs
x0 = X0(1);     xf = Xfdes(1);     % m
y0 = X0(2);     yf = Xfdes(2);     % m
th0 = X0(3);    thf = Xfdes(3);    % rad
K0 = X0(4);     Kf = Xfdes(4);     % 1/m

% Ensure Xf is defined as a column vector
Xf = [xf; yf; thf; Kf];

% Term used for finite difference jacobian approximation
h = 0.01; 

% Initial Heuristic Guess:
d = sqrt(xf^2 + yf^2);      % distance d to final position
D_th = abs(thf);            % delta theta
s = d*((D_th^2)/5 + 1) + 0.4*D_th;
c = 0;
a = 6*thf/s^2 - 2*K0/s + 4*Kf/s;
b = 3*(K0+Kf)/s^2 + 6*thf/s^3;

P = [a;b;c;s];

% solve for final pose from intial guess
%   - create a function to integrate for x,y
funx_ths = @(A,B,C,S) cos(th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4);
funy_ths = @(A,B,C,S) sin(th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4);

fun_xs = @(A,B,C,S) x0 + integral(@(S) funx_ths(A,B,C,S),0,S);
fun_ys = @(A,B,C,S) y0 + integral(@(S) funy_ths(A,B,C,S),0,S);
fun_ths = @(A,B,C,S) th0 + K0*s + (A*S.^2)/2 + (B*S.^3)/3 + (C*S.^4)/4;
fun_Ks = @(A,B,C,S) K0 + A*S + B*S.^2 + C*S.^3;

% Create a cell array of the forward functions
FwdFcns = {fun_xs,fun_ys,fun_ths,fun_Ks};

% Set an upper bound for the first loop to prevent inf loop
maxIter = 2000;

% Setup the command window counter
fprintf('Path Planning Iteration:\n');

PassBool = 0;

for iter = 1:maxIter

    Xs = [feval(fun_xs,P(1),P(2),P(3),P(4));
            feval(fun_ys,P(1),P(2),P(3),P(4));
            feval(fun_ths,P(1),P(2),P(3),P(4));
            feval(fun_Ks,P(1),P(2),P(3),P(4))];

    % Calculate delta X
    Dx = Xs - Xf;

    % Check if the delta is within the convergence criteria
    if ( (abs(Dx(1,1)) < X_MinErr(1,1)) && (abs(Dx(2,1)) < X_MinErr(2,1)) && ...
            (abs(Dx(3,1)) < X_MinErr(3,1)) && (abs(Dx(4,1)) < X_MinErr(4,1)))
        PassBool = 1;
        break
    end

    % Intialize the jacobian
    J = zeros(4);

    for ii = 1:4
        for jj=1:4
            % Finite difference approximation of the jacobian
            Ptemp = P;
            Ptemp(jj) = P(jj)+h;
            J(ii,jj) = (Xs(ii) - FwdFcns{ii}(Ptemp(1),Ptemp(2),Ptemp(3),Ptemp(4)))/h;
        end
    end

    % Calculate the change in pose using Dx and the inverse jacobian
    Dp = J\Dx;

    % Update the pose with a small gain constant for stability
    P = P + 0.2*Dp;

    % Update the command window with the progress
    if iter > 1
        %Delete previous iteration count
        fprintf(repmat('\b',1,numel(msg)));
    end
    msg = sprintf('%d/%d', iter, maxIter);
    fprintf(msg);
    pause(.001); % allows time for display to update

end

fprintf('\n')

% Create the Stats vector
Stats = cell(5,2);
Stats{1,1} = 'Convergence Bool';Stats{1,2} = PassBool;
Stats{2,1} = 'X Error';Stats{2,2} = Dx(1,1);
Stats{3,1} = 'Y Error';Stats{3,2} = Dx(2,1);
Stats{4,1} = 'Heading Error';Stats{4,2} = Dx(3,1);
Stats{5,1} = 'Curvature Error';Stats{5,2} = Dx(4,1);

% Form the cubic polynomial
npts = 45;
s = linspace(0,P(4),npts);
PlannedPath = zeros(3,npts);
for ii = 1:npts
    PlannedPath(1,ii) = feval(fun_xs,P(1),P(2),P(3),s(ii));
    PlannedPath(2,ii) = feval(fun_ys,P(1),P(2),P(3),s(ii));
    PlannedPath(3,ii) = feval(fun_ths,P(1),P(2),P(3),s(ii));
end
 
% % starting with the first polynomial as the intial guess, will now 
% D = 0;
% P2 = [P(1); P(2); P(3); D; P(4)];   % initialize and add the fourth parameter, D
% X_err2 = [0.001; 0.001; 0.1; 0.01;0.005]; % x(m), y(m), th(rad), K(1/m); cost fcn
% Xf2 = [Xf(1); Xf(2); Xf(3); Xf(4); 0]; % the cost solution should approach zero   
% Dc = 0.02;   % distance clearance, m
% xobst = 2; % obstacle position, x, m
% yobst = 2; % obstacle position, y, m 
% Lambda = 1; % repulsive constant, need to experiment to quantify impact
% N = 1;  % number of obstacles
% NTrapPts = 100; % number of trapezoidal integration points to use with the cost function, L
% 
% % update the functions for the new number of inputs
% funx_ths2 = @(A,B,C,D,S) cos(th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4 + (D.*S.^5)/5);
% funy_ths2 = @(A,B,C,D,S) sin(th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4 + (D.*S.^5)/5);
% fun_xs2 = @(A,B,C,D,S) x0 + integral(@(S) funx_ths2(A,B,C,D,S),0,S);
% fun_ys2 = @(A,B,C,D,S) y0 + integral(@(S) funy_ths2(A,B,C,D,S),0,S);
% fun_ths2 = @(A,B,C,D,S) th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4 + (D.*S.^5)/5;
% fun_Ks2 = @(A,B,C,D,S) K0 + A*S + B*S.^2 + C*S.^3 + D*S.^4;
% 
% % setup the cost function (lower = farther from obstacle)
% fun_Ds = @(A,B,C,D,S) min(sqrt((fun_xs2(A,B,C,D,S)-xobst).^2+(fun_ys2(A,B,C,D,S)-yobst).^2),Dc);
% fun_L = @(A,B,C,D,S) trapz(NTrapPts,Lambda./fun_Ds(A,B,C,D,S))-S.*N*Lambda./Dc;
% 
% % calculate functions
% x = feval(fun_xs2,P2(1),P2(2),P2(3),P2(4),P2(5));
% y = feval(fun_ys2,P2(1),P2(2),P2(3),P2(4),P2(5));
% th = feval(fun_ths2,P2(1),P2(2),P2(3),P2(4),P2(5));
% K = feval(fun_Ks2,P2(1),P2(2),P2(3),P2(4),P2(5));
% L = feval(fun_L,P2(1),P2(2),P2(3),P2(4),P2(5));
% 
% Xs2 = [x;y;th;K;L];
% 
% % create a cell array of the forward funtions
% FwdFcns2 = {fun_xs2,fun_ys2,fun_ths2,fun_Ks2,fun_L};
% 
% fprintf('running loop #2\n')
% 
% for iter = 1:15
%         
%     % calculate functions
%     x = feval(fun_xs2,P2(1),P2(2),P2(3),P2(4),P2(5));
%     y = feval(fun_ys2,P2(1),P2(2),P2(3),P2(4),P2(5));
%     th = feval(fun_ths2,P2(1),P2(2),P2(3),P2(4),P2(5));
%     K = feval(fun_Ks2,P2(1),P2(2),P2(3),P2(4),P2(5));
%     L = feval(fun_L,P2(1),P2(2),P2(3),P2(4),P2(5))
% 
%     Xs2 = [x;y;th;K;L];
% 
%     % calculate delta X using the same desired final position
%     Dx2 = Xs2 - Xf2;
% 
%     % check if the delta is within the convergence criteria
%     if abs(Dx2) < X_err2
%         break
%     end
% 
%     % intialize the jacobian
%     J2 = zeros(5);
% 
%     for i = 1:5
%         for j=1:5
%             % finite difference approximation of the jacobian
%             Ptemp2 = P2;
%             Ptemp2(j) = P2(j)+h;
%             J2(i,j) = (Xs2(i) - FwdFcns2{i}(Ptemp2(1),Ptemp2(2),Ptemp2(3),Ptemp2(4),Ptemp2(5)))/h;
%         end        
%     end
%    
%     % no change in final row of jacobian for all variables except S...
%     % should be affecting the distance formula, unless tripping the min
%     % statement
%     J2;
%     
%     % calculate delta P using Dx and the inverse jacobian
%     Dp2 = J2\Dx2;
% 
%     % update P with a small gain constant for stability
%     P2 = P2 + 0.2*Dp2;
%     
% end
% 
% fprintf('loop #2 complete\n')
% 
% % print the iteration outputs to the workspace
% iter
% P2
% Xs2
% Dx2
% 
% % add the new cubic polynomial to the plot
% npts = 50;
% s = linspace(0,P2(5),npts);
% x2 = zeros(1,npts);
% y2 = zeros(1,npts);
% for i = 1:npts
%     x2(i) = feval(fun_xs2,P2(1),P2(2),P2(3),P2(4),s(i));
%     y2(i) = feval(fun_ys2,P2(1),P2(2),P2(3),P2(4),s(i));
% end
% 
% figure
% hold on
% %plot(x,y)
% plot(x2,y2)
% %plot(xf,yf,'r*')



end