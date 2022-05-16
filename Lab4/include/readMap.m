function [grid] = readMap(map_path)
        map = imread(map_path);
        map_g = im2gray(map);
        grid = double(map_g == 0);
    end