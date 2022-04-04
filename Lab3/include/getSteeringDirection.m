function [go_theta] =getSteeringDirection(h_smooth, goal_pose, x, y,alpha, theta)
    %Apply threshold
    h_smooth(h_smooth < 0.3) = 0;

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
    k_target = ceil(target_t/alpha);
    if k_target == 0
            k_target = 1;
    end
    
    dist = sqrt((goal_pose(1)-x)^2+(goal_pose(2)-y)^2);

    if (all(h_smooth == 0)) || dist < 0.25
        go_theta = k_target*rad2deg(alpha);
        return 
    end

    figure(2);
    subplot(2,1,1)
    hold on;
    bar(k_target,1);
    hold off

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
    for i = 1:size(passages,1)
        if (abs(passages(i,1)-passages(i,2)) > 3)
            real_passages(i,:) = passages(i,:);
        end
    end
            

    dist=min(mod((real_passages - k_target),36), mod((k_target - real_passages),36));
    if (dist(1) == dist(2))
        dist(2) = 100000;
    end
    [r,c] = find(dist==min(dist(:)),1);       
    chosenValley = real_passages(r,:)
    k_chossen = chosenValley(1,c);

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

    smax = 9;
    
    if (abs(chosenValley(1) - chosenValley(end)) > smax)
        fprintf("WIDE VALLEY\n");

        if (k_chossen >chosenValley(1))
            go_theta = k_chossen*rad2deg(alpha)-0.5*smax*rad2deg(alpha);
        else
            go_theta = k_chossen*rad2deg(alpha)+0.5*smax*rad2deg(alpha);
        end

    else
        fprintf("NARROW VALLEY\n");
        go_theta = 0.5*(chosenValley(1)*rad2deg(alpha)+chosenValley(end)*rad2deg(alpha));
    end
    figure(2);
    subplot(2,1,1)
    hold on 
    bar(go_theta/rad2deg(alpha),1);
    hold off
end