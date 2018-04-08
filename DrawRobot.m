function DrawRobot(t, states, desired_trajectory, params)
n_state = size(states, 1);

figure
hold on
plot(t, desired_trajectory, 'Color', 'r');
axis([0, t(n_state), -5, 5])

for i=1:n_state
    
    dt = t(1);
    if i > 1
        dt = t(i) - t(i-1);
    end
    
    x = states(i, 1);
    y = states(i, 4);
    theta = atan(states(i, 3));
    
    vehicle_direction_x = x + 0.5*cos(theta);
    vehicle_direction_y = y + 0.5*sin(theta);
    
    [p1, p2, p3, p4] = RobotChassis(x, y, theta, 0.3, 0.2);
    
    cog = plot(x, y, 'o', 'Color', 'b');
    vehicle_direction = line([x, vehicle_direction_x], [y, vehicle_direction_y]);
    
    l1 = line([p1(1), p2(1)], [p1(2), p2(2)]);
    l2 = line([p2(1), p3(1)], [p2(2), p3(2)]);
    l3 = line([p3(1), p4(1)], [p3(2), p4(2)]);
    l4 = line([p4(1), p1(1)], [p4(2), p1(2)]);
    
    pause(dt);
    if(i < size(t, 1) - 1)
        delete(cog);
        delete(vehicle_direction);
        
        delete(l1);
        delete(l2);
        delete(l3);
        delete(l4);
    end
end

end

function [p1, p2, p3, p4] = RobotChassis(cx, cy, theta, marx, mary)

T_C_O = [
            cos(theta), -sin(theta), cx;
            sin(theta),  cos(theta), cy;
                     0,           0,  1;
        ];

p1_C = [-marx; mary; 1];
p2_C = [marx; mary; 1];
p3_C = [marx; -mary; 1];
p4_C = [-marx; -mary; 1];

p1 = T_C_O * p1_C;
p2 = T_C_O * p2_C;
p3 = T_C_O * p3_C;
p4 = T_C_O * p4_C;

end