function [dead_reckoning] = getDeadReckoning(state_vector, right_ticks, left_ticks, baseline)
    Dk = (right_ticks + left_ticks) / 2;
    Dt = (right_ticks - left_ticks) / baseline;

    dead_reckoning = state_vector + [Dk * cos(state_vector(3) + (Dt / 2));
                                Dk * sin(state_vector(3) + (Dt / 2));
                                Dt];
end
