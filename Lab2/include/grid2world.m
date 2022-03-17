function [x, y] = grid2world(cell_x,cell_y, map_size)
%GRID2WORLD Converts grid coordinates to world coordinates
%INPUT:     cell_x -> x cell
%           cell_y -> y cell
%           size_map -> size of the map
%OUTPUT:    x -> x coordinate of world
%           y -> y coordinate of world
x = cell_x*4/map_size;
y = cell_y*4/map_size;

end
