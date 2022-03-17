function [go_theta] =getSteeringDirection(h_smooth, goal_pose, x, y,alpha)
    Hp1 = zeros(1,size(h_smooth,2));
    for j = 1:size(h_smooth,2)
        if h_smooth(j) > 0
            Hp1(j) = 1;
        end
    end
    
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

    k_target = atan2(goal_pose(2)-y,goal_pose(1)-x);
    k_target = k_target + 2*pi*(k_target<0);
    ktarg = ceil(k_target/alpha);
    if ktarg == 0
            ktarg = 1;
    end

    dist = abs(passages-ktarg); 
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

    smax = 5;

    if (abs(chosenValley(1) - chosenValley(2)) > smax)
        sectors = chosenValley(1):chosenValley(2);
        [~,kn] = min(abs(sectors-ktarg));
        go_theta = kn*rad2deg(alpha) + 0.5*rad2deg(alpha)*smax;
    else
        go_theta = 0.5*(chosenValley(1)*rad2deg(alpha)+chosenValley(2)*rad2deg(alpha));
    end
end