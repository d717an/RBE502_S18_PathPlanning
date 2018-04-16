function [l1, l2, l3, l4] = draw_rect(startx, starty, lengthx, lengthy)
% Function to draw a rectangle given starting point x, y, width and length

l1 = line([startx, startx + lengthx], [starty, starty], 'Color', 'r');
l2 = line([startx, startx], [starty, starty + lengthy], 'Color', 'r');
l3 = line([startx, startx + lengthx], [starty + lengthy, starty + lengthy], 'Color', 'r');
l4 = line([startx + lengthx, startx + lengthx], [starty + lengthy, starty], 'Color', 'r');
end