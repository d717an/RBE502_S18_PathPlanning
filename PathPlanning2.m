% using four parameters, a,b,c,s to determine four states: x,y,th,K
% can set the initial position and posture to 0
% [1] linearizes x = f(p), then inverts to solve for p
% [1]'s algorithm: assume c = 0, find s, then solve for a,b with remaining
% 2 eqn's

% s = distance traveled along the curve

% (1) assume all ICs are 0 (starting at the origin)
% (2) given a,b,c and s, produce the posture x,y,th,K for the final posture
% and compare it against the desired
% (3) if all termination conditions are met, end the algorithm
% (4) if not, compute the finite jacobian difference which will be used to
% modify the parameters, then add the deltas to the present parameters, and
% repeat...

function [] = PathPlanning2()

% Clean up 
clear all; close all; clc;

% Inputs
%   - x0, initial x position
%   - xf, final x position
%   - y0, intial y position
%   - yf, final y position
%   - th0, initial heading (angle w.r.t. positive x axis)
%   - thf, final heading (angle w.r.t. positive x axis)
%   - K0, initial kappa (curvature) value
%   - Kf, final kappa (curvature) value

x0 = 0; % m
xf = 3; % m
y0 = 0; % m
yf = 3; % m
th0 = -pi/4; % rad
thf = pi/8; % rad
K0 = 0; % 1/m
Kf = 0; % 1/m
X_err = [0.001; 0.001; 0.1; 0.005]; % x(m), y(m), th(rad), K(1/m)
Xf = [xf; yf; thf; Kf]; % x(m), y(m), th(rad), K(1/m)

h = 0.01; % term used for finite difference jacobian approximation

% Intial Heuristic Guess:
d = sqrt(xf^2 + yf^2);      % distance d to final position
D_th = abs(thf);            % delta theta
s = d*((D_th^2)/5 + 1) + 0.4*D_th;
c = 0;
a = 6*thf/s^2 - 2*K0/s + 4*Kf/s;
b = 3*(K0+Kf)/s^2 + 6*thf/s^3;

P = [a;b;c;s];

% solve for final pose from intial guess
%   - create a function to integrate for x,y
funx_th_s = @(A,B,C,S) cos(th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4);
funy_th_s = @(A,B,C,S) sin(th0 + K0.*S + (A.*S.^2)/2 + (B.*S.^3)/3 + (C.*S.^4)/4);

fun_xs = @(A,B,C,S) x0 + integral(@(S) funx_th_s(A,B,C,S),0,S);
fun_ys = @(A,B,C,S) y0 + integral(@(S) funy_th_s(A,B,C,S),0,S);
fun_ths = @(A,B,C,S) th0 + K0*s + (A*S.^2)/2 + (B*S.^3)/3 + (C*S.^4)/4;
fun_Ks = @(A,B,C,S) K0 + A*S + B*S.^2 + C*S.^3;

% create a cell array of the forward funtions
FwdFcns = {fun_xs,fun_ys,fun_ths,fun_Ks};
%FwdFcns{1}(s) % sample, evaluate fcn 1 at s

fprintf('running!\n')

for iter = 1:10000

    Xs = [feval(fun_xs,P(1),P(2),P(3),P(4));
            feval(fun_ys,P(1),P(2),P(3),P(4));
            feval(fun_ths,P(1),P(2),P(3),P(4));
            feval(fun_Ks,P(1),P(2),P(3),P(4))];

    % calculate delta X
    Dx = Xs - Xf

    % check if the delta is within the convergence criteria
    if abs(Dx) < X_err
        break
    end

    % intialize the jacobian
    J = zeros(4);

    for i = 1:4
        for j=1:4
            % finite difference approximation of the jacobian
            % not just a function of s??
            Ptemp = P;
            Ptemp(j) = P(j)+h;
            J(i,j) = (Xs(i) - FwdFcns{i}(Ptemp(1),Ptemp(2),Ptemp(3),Ptemp(4)))/h;
        end
    end

    J;

    % calculate delta P using Dx and the inverse jacobian
    Dp = J\Dx;

    % update P with a small gain constant for stability
    P = P + 0.2*Dp;

end

fprintf('iterations complete\n')

% print the iteration outputs to the workspace
iter
P
Xs
Dx


% plot the cubic polynomial
npts = 50;
s = linspace(0,P(4),npts);
x = zeros(1,npts);
y = zeros(1,npts);
for i = 1:npts
    x(i) = feval(fun_xs,P(1),P(2),P(3),s(i));
    y(i) = feval(fun_ys,P(1),P(2),P(3),s(i));
end

figure
hold on
plot(x,y)
plot(xf,yf,'r*')


end