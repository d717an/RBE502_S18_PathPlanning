function main()

% Clean the workspace and environment
%   - turn off warnings
clear; clc; close all; w = warning ('off','all');

% Choose whether or not to run obstacle avoidance
AvoidObstacles = 0;

% Define robot plotting par
robot_params = containers.Map;
robot_params('l') = 1;

% Define desired starting and ending robot poses
X0 = [0,0,-pi/4,0]; Xfdes = [4,1,pi/8,0];

fprintf('\nDeriving cubic path from: x = %.2f m, y = %.2f, heading = %.2f rad\n',X0(1),X0(2),X0(3));
fprintf('to xf = %.2f m, yf = %.2f, heading = %.2f rad\n',Xfdes(1),Xfdes(2),Xfdes(3));

% set last three to 0 for normal
[Path,Success,BasicPathStats,ObstSuccess,ObstStats] = PathPlanning(X0,Xfdes,3,0.2,AvoidObstacles);

if Success == 1
    fprintf('\nBasic path planning successful!\n');
    % Print the solved path results to the workspace
    BasicPathStats
else
    fprintf('\nBasic path planning unsuccessful... exiting code\n');
    exit
end

if ObstSuccess == -1
    fprintf('\nObstacle avoidance not attempted (recommended)\n');
elseif ObstSuccess == 1
    fprintf('\nPath planning with obstacle avoidance was successful!\n');
    % Print the solved path results to the workspace
    ObstStats
else
    fprintf('\nPath planning with obstacle avoidance was successful...continuing with basic path\n');
end

% Making a simplifying assumption that each point on the path is 1 second
%   - define the derivatives of the solved path for desired control inputs
Time = zeros(1,size(Path,2));
for i = 1:size(Path,2)
    Time(i) = i-1;
end

% Calculate desired velocity
dx_d = diff(Path(1,:))./diff(Time);
dy_d = diff(Path(2,:))./diff(Time);
tt = 0:(Time(end)/(length(Time)-2)):Time(end);

% Calculate desired acceleration
dx_d2 = diff(dx_d)./diff(tt);
dy_d2 = diff(dy_d)./diff(tt);
tt2 = 0:(tt(end)/(length(tt)-2)):tt(end);

% Calculate third derivative
dx_d3 = diff(dx_d2)./diff(tt2);
dy_d3 = diff(dy_d2)./diff(tt2);
tt3 = 0:(tt2(end)/(length(tt2)-2)):tt2(end);

% Calculate control inputs
ud1 = zeros(1,size(Time,2));
ud2 = zeros(1,size(Time,2));
for i=1:size(Time,2)
    
    % Differentiating reduced vector lengths
    %   - interpolate to find derivatives in full time vector
    xd1 = interp1(tt,dx_d,i);
    yd1 = interp1(tt,dy_d,i);
    xd2 = interp1(tt2,dx_d2,i);
    yd2 = interp1(tt2,dx_d2,i);
    xd3 = interp1(tt3,dx_d3,i);
    yd3 = interp1(tt3,dy_d3,i);
    
    % set the control inputs
    ud1(i) = xd1;
    ud2(i) = (yd3*xd1^2-xd3*yd1*xd1-3*yd2*xd1*xd2+3*yd1*xd2^2)/xd1^4;
end

% Define the control law parameters
%   - positive values make the system unstable
%   - more negative = more accurate but slower
lambda1 = -40;
lambda2 = -40;
modulus = -40;
damp = 1;

k = zeros(4, 1);
k(1) = lambda1;
k(2) = lambda2 + 2*damp*modulus;
k(3) = modulus^2 + 2*damp*modulus*lambda2;
k(4) = modulus^2 * lambda2;

% Set the robot's initial state
init_state = [0, 0, 0, 0];

fprintf('\nStarting ode45...\n');

[T, X] = ode45(@(t, x)MobileRobot(t, x, ...
          chain_form(interp1(Time,Path(1,:),t),interp1(Time,Path(2,:),t),interp1(Time,Path(3,:),t),0,robot_params), ...
          [interp1(Time,ud1,t), interp1(Time,ud2,t)], robot_params, k), [0 Time(end)], init_state);   

fprintf('\nStarting simulation...\n');
      
% Plot the robot's actual path against the desired path                              
figure;hold on
plot(Path(1,:),Path(2,:),'k.');
plot(X(:,1), X(:,4),'r-');
legend('Desired Trajectory','Robot Trajectory','Location','Northwest')

% Animate the Robot following the path
DrawRobot(T,X,Path(1,:),Path(2,:),robot_params);

fprintf('\nMain function complete!\n');

end

function tf = chain_form(x, y, theta, phi, params)

% Transform states [x, y, theta, phi] -> [x1, x2, x3, x4]
%   - x1 = x
%   - x2 = (tan(phi) * cos(theta)^2) / l
%   - x3 = tan(theta)
%   - x4 = y

l = params('l');
tf = zeros(1, 4);

tf(1) = x;
tf(2) = (tan(phi) / l) * cos(theta)^3;
tf(3) = tan(theta);
tf(4) = y;
end

function dx = MobileRobot(T, X, xd, ud, params,k)

% State variables
% x         position error of the vehicle in the world's x axis
% y         position error of the vehicle in the world's y axis
% theta     heading error of the vehicle with respect to the world's x axis
% phi       steering angle between the front wheel and body axis
% dx        velocity of the vehicle in the world's x axis
% dy        velocity of the vehicle in the world's y axis
% dtheta    angular velocity of the vehicle with respect to world's x axis
% dphi      angular velocity of the steering angle

% Control States
%   - x1 = x
%   - x2 = (tan(phi) * cos(theta)^2) / l
%   - x3 = tan(theta)
%   - x4 = y

% Vehicle's Parameters
l = params('l'); 

% Current states
x1 = X(1);
x2 = X(2);
x3 = X(3);
x4 = X(4);

% State errors: xd - x
ex1 = xd(1) - x1;
ex2 = xd(2) - x2;
ex3 = xd(3) - x3;
ex4 = xd(4) - x4;

eu1 = -k(1)*ex1;
eu2 = -k(2)*ex2 - k(3)*ex3/ud(1) - k(4)*ex4/(ud(1)^2);

dx = zeros(4, 1);
dx(1) = eu1;
dx(2) = eu2;
dx(3) = ex2*ud(1) + eu1*xd(2);
dx(4) = ex3*ud(1) + eu1*xd(3);
end