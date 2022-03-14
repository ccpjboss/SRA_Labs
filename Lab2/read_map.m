function [grid] = read_map(map_path)
%READ_MAP Summary of this function goes here
%   Detailed explanation goes here
map = imread(map_path);
map_g = im2gray(map);
grid = double(map_g == 0);
end

