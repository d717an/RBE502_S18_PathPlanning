function generate_env()
clear;
clc;
close all;

figure 
hold on
axis equal
grid on
axis([0 500 0 500])

[rect_list, tri_list] = get_user_input();


end

% Function to add user-defined obstacles
% The only supported obstacles are triangles and rectangles
% The function return 2 lists, a list of rectangles and a list of rectangle
% Each item in the list is a list of lines that define the obstacle
% Specifically:
% - an item in the rectangle list is a list of 4 lines
% - an iten in the triangle list is a list of 3 lines
function [rect_list, tri_list] = get_user_input()

option_msg = 'Please choose an option (default rectangle):\n1. Add a rectangle\n2. Add a triangle\n3. Done\n=> ';

rect_list = [];
tri_list = [];

while 1
    option = input(option_msg);
    if option == 1
        disp('Add a rectangle');
        
        rect_msg_1 = 'Enter bottom left corner x coordinate => ';
        rect_msg_2 = 'Enter bottom left corner y coordinate => ';
        rect_msg_3 = 'Enter length in x => ';
        rect_msg_4 = 'Enter length in y => ';
        
        rect_x      = input(rect_msg_1);
        rect_y      = input(rect_msg_2);
        rect_width  = input(rect_msg_3);
        rect_height = input(rect_msg_4);
        
        [l1, l2, l3, l4] = draw_rect(rect_x, rect_y, rect_width, rect_height);
        rect_list = [rect_list; [l1, l2, l3, l4]];
        
    elseif option == 2
        disp('Add a triangle');
        
        tri_msg_1 = 'Enter bottom left corner x coordinate => ';
        tri_msg_2 = 'Enter bottom left corner y coordinate => ';
        tri_msg_3 = 'Enter bottom length  => ';
        tri_msg_4 = 'Enter left length => ';
        tri_msg_5 = 'Enter angle => ';
        
        tri_x           = input(tri_msg_1);
        tri_y           = input(tri_msg_2);
        tri_bot_len     = input(tri_msg_3);
        tri_left_len    = input(tri_msg_4);
        tri_angle       = input(tri_msg_5);
        
        [l1, l2, l3] = draw_triangle(tri_x, tri_y, tri_bot_len, tri_angle, tri_left_len);
        tri_list = [tri_list; [l1, l2, l3]];
    else
        break
    end
end

end

function [l1, l2, l3] = draw_triangle(origx, origy, bottom_side, angle, left_side)
% Function to draw a triangle given a bottom left corner, length of bottom
% side, left side and the angle between the two sides
l1 = line([origx, origx + bottom_side], [origy, origy], 'Color', 'r');
l2 = line([origx, origx + left_side * cos(angle)], [origy, origy + left_side * sin(angle)], 'Color', 'r');
l3 = line([origx + bottom_side, origx + left_side * cos(angle)], [origy, origy + left_side * sin(angle)], 'Color', 'r');
end

function [l1, l2, l3, l4] = draw_rect(startx, starty, lengthx, lengthy)
% Function to draw a rectangle given starting point x, y, width and length

l1 = line([startx, startx + lengthx], [starty, starty], 'Color', 'r');
l2 = line([startx, startx], [starty, starty + lengthy], 'Color', 'r');
l3 = line([startx, startx + lengthx], [starty + lengthy, starty + lengthy], 'Color', 'r');
l4 = line([startx + lengthx, startx + lengthx], [starty + lengthy, starty], 'Color', 'r');
end

