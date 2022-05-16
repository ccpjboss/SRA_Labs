function motionError = getMotionError(C_p,C_u,state_vector, sr, sl, b)
    %myFun - Description
    %
    % Syntax: motionError =getMotionErrormyF(input)
    %
    % Long description
    Dk = (sr + sl) / 2;
    Dt = (sr - sl) / b;

    J_p = [1 0 -Dk * sin(state_vector(3) + Dt / 2);
        0 1 Dk * cos(state_vector(3) + Dt / 2);
        0 0 1];

    J_u = [cos(state_vector(3) - (sl - sr) / (2 * b)) / 2 - (sin(state_vector(3) - (sl - sr) / (2 * b)) * (sl / 2 + sr / 2)) / (2 * b), cos(state_vector(3) - (sl - sr) / (2 * b)) / 2 + (sin(state_vector(3) - (sl - sr) / (2 * b)) * (sl / 2 + sr / 2)) / (2 * b);
    sin(state_vector(3) - (sl - sr) / (2 * b)) / 2 + (cos(state_vector(3) - (sl - sr) / (2 * b)) * (sl / 2 + sr / 2)) / (2 * b), sin(state_vector(3) - (sl - sr) / (2 * b)) / 2 - (cos(state_vector(3) - (sl - sr) / (2 * b)) * (sl / 2 + sr / 2)) / (2 * b);
    1 / b, -1 / b];

    motionError = J_p * C_p * J_p' + J_u * C_u * J_u';

end
