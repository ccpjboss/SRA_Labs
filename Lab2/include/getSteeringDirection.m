function [go_theta] =getSteeringDirection(h_smooth, goal_pose, x, y,alpha)
    wrapN = @(x, N) (1 + mod(x-1, N));

    %Apply threshold
    h_smooth(h_smooth < 0.15) = 0;

    %Histogram of 1s and 0s
    Hp1 = zeros(1,size(h_smooth,2));
    for j = 1:size(h_smooth,2)
        if h_smooth(j) > 0
            Hp1(j) = 1;
        end
    end

    %Find target bin
    target_t = atan2(goal_pose(2)-y,goal_pose(1)-x);
    target_t = target_t + 2*pi*(target_t<0);
    k_target = ceil(target_t/alpha)
    if k_target == 0
            k_target = 1;
    end

    %Check if empty target bin
    if(Hp1(k_target) == 0)
        if (Hp1(wrapN(k_target-2,36)) == 0 && Hp1(wrapN(k_target-1,36)) == 0 && Hp1(wrapN(k_target+2,36)) == 0 && Hp1(wrapN(k_target+1,36)) == 0)
            go_theta = k_target*rad2deg(alpha);
            return
        end
    end

    %% !! CHANGE !!
    
    b1=(find(diff(Hp1)==-1)); %Find beginning of consecutive clear sectors
    b1 = b1 + 1;
    b2=(find(diff(Hp1)==1)); %Find end of consecutive clear sector
    
    if (Hp1(1) == 0)
        b1=[1,b1];
    end
    if (Hp1(end) == 0)
        b2=[b2,size(h_smooth,2)];
    end

    passages = [b1' b2'];

    dist = abs(passages-k_target); 
    [r,c] = find(dist==min(dist(:)));       
    chosenValley = passages(r,:);

    %The distance might be the same to two valleys, so we pick the larger one
    if size(chosenValley) > 1
        if chosenValley(1) == chosenValley(2)
            chosenValley = unique(chosenValley)';
            %If repeated valley, just take the unique value
        else
            larger = diff(chosenValley');
            [r,t] = max(larger);
            chosenValley = chosenValley(t,:);
            %Picks the larger valley
        end
    end

    smax = 10;
    
    if (abs(chosenValley(1) - chosenValley(end)) > smax)
        fprintf("WIDE VALLEY\n");
        sectors = chosenValley(1):chosenValley(end);
        [~,kn] = min(abs(sectors-k_target));
        go_theta = sectors(kn)*rad2deg(alpha)+0.5*rad2deg(alpha)*smax;
    else
        fprintf("NARROW VALLEY\n");
        go_theta = 0.5*(chosenValley(1)*rad2deg(alpha)+chosenValley(end)*rad2deg(alpha));
    end
end