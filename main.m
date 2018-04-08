function main()

clear;
clc;
close all;

robot_params = containers.Map;
robot_params('l') = 1;

% Sine wave properties
A = 3;
freq = 0.2*pi;

init_state = [-2, 0, A*freq, -1];

% Positive values make the system unstable
% The more negative, the more accurate the system but slower response
lambda1 = -40;
lambda2 = -40;
modulus = -40;
damp = 1;

k = zeros(4, 1);
k(1) = lambda1;
k(2) = lambda2 + 2*damp*modulus;
k(3) = modulus^2 + 2*damp*modulus*lambda2;
k(4) = modulus^2 * lambda2;

[T, X] = ode45(@(t, x)MobileRobot(t, x, ...
                                  chain_form(t, path_generator(A, freq, t), atan2(A*freq*cos(freq*t), 1), 0, robot_params), ...
                                  [1, -A*(freq^3)*cos(freq*t)], robot_params, k), [0, 10], init_state);

size(X, 1)
DrawRobot(T, X, path_generator(A, freq, T), robot_params);

figure
hold on
plot(T, path_generator(A, freq, T), 'Color', 'r');
plot(T, X(:, 4), 'Color', 'b');
legend('Desired Trajectory', 'Robot Trajectory')

end



% Function to transform set of 
% states [x, y, theta, phi] -> [x1, x2, x3, x4]
% where:
% x1 = x
% x2 = (tan(phi) * cos(theta)^2) / l
% x3 = tan(theta)
% x4 = y
function tf = chain_form(x, y, theta, phi, params)
    l = params('l');
    tf = zeros(1, 4);
    
    tf(1) = x;
    tf(2) = (tan(phi) / l) * cos(theta)^3;
    tf(3) = tan(theta);
    tf(4) = y;
end