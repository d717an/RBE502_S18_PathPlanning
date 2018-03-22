% State variables
% x         position error of the center of gravity of the vehicle in the world's x axis
% y         position error of the center of gravity of the vehicle in the world's y axis
% theta     orientation error of the vehicle with respect to the world's x axis
% phi       steering angle between the front wheel and body axis
% dx        velocity of the center of gravity of the vehicle in the world's x axis
% dy        velocity of the center of gravity of the vehicle in the world's y axis
% dtheta    angular velocity of the vehicle with respect to world's x axis
% dphi      angular velocity of the steering angle

% Control States
% x1 = x
% x2 = (tan(phi) * cos(theta)^2) / l
% x3 = tan(theta)
% x4 = y

function dx = MobileRobot(T, X, xd, ud, params,k)
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