function [cell_x, cell_y] = world2grid(x,y,size_map)
%WORLD2GRID Converts world coordinates to grid coordinates
%INPUT:     x -> x coordinate
%           y -> y coordinate
%           size_map -> size of the map
%OUTPUT:    cell_x -> row of cell
%           cell_y -> collumn of cell

cell_x = round(size_map*x/4);
cell_y = round(size_map*y/4);
end

