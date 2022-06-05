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

% A2 = [x + ((sr+sl)/2)*cos(state_vector(3)+((sr-sl)/b)/2);
%       y + ((sr+sl)/2)*sin(state_vector(3)+((sr-sl)/b)/2);
%       state_vector(3) + ((sr-sl)/b)]
%
% Ju = jacobian(A2,[sr,sl])
J_u = [cos(Dt/2 + state_vector(3)), -(Dk*sin(Dt/2 + state_vector(3)))/2;
    sin(Dt/2 + state_vector(3)),  (Dk*cos(Dt/2 + state_vector(3)))/2;
    0,                         1];

motionError = J_p * C_p * J_p' + J_u * C_u * J_u';

end
