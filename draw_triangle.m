function [l1, l2, l3] = draw_triangle(origx, origy, bottom_side, angle, left_side)
% Function to draw a triangle given a bottom left corner, length of bottom
% side, left side and the angle between the two sides
l1 = line([origx, origx + bottom_side], [origy, origy], 'Color', 'r');
l2 = line([origx, origx + left_side * cos(angle)], [origy, origy + left_side * sin(angle)], 'Color', 'r');
l3 = line([origx + bottom_side, origx + left_side * cos(angle)], [origy, origy + left_side * sin(angle)], 'Color', 'r');

end