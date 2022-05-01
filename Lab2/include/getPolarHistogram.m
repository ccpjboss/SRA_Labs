function h_smooth = getPolarHistogram(world_y, y, world_x, x, theta, alpha, window_size)
    beta_cells = atan2((world_y-y),(world_x-x));
    beta_cells = beta_cells+2*pi*(beta_cells<0);

    figure(2)
    subplot(2,1,2)
    polarhistogram(beta_cells,"FaceColor",'red');

    dist_cells = sqrt((world_x-x).^2+(world_y-y).^2);
    dmax = sqrt(2) * (window_size-1)/2;
    a = dmax;
    b = 1;
    m = 1.*(a-b.*dist_cells);
    
    k = ceil(beta_cells/alpha);
    
    h = zeros(1,2*pi/alpha);

    for i=1:size(m,1)
        h(k(i)) = h(k(i)) + m(i);
    end

    L = 2; 
    h_length = size(h,2);       %get length of sector array
    h_padded = [zeros(1,L),h(1,:),zeros(1,L)];   %padding h with zeros on the ends to make average calculations
    hp_sum = zeros(1,h_length); %initialize array for summation of sector values
    div = ones(1,h_length);     %initialize divisor for each element
    weightArray = [1:L,L:-1:1];  %weighted array to determine coefficients for avg calc
    
    %Since the first & last L indicies will not have 2L+1 values to divide by
    %the divisor must be modified for the end indicies. 
    div = div*(2*L+1);          %divisor for calculating avg sector value
    div_mod = [L:-1:1, zeros(1,h_length-(2*L)), 1:L];    %modifier array
    div = div-div_mod;          %accounting for the missing end values
    
    %loop to calculate the sum of the weighted average for each sector
    for i=1+L:h_length+L
        hp_sum(i-L) = sum(h_padded((i-L):(i+L-1)).*weightArray);    %sector summing array
    end
    h_smooth = hp_sum./div; %calculate H'
end