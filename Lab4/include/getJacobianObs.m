function [J_g] = getJacobianObs(state_vector, xm, ym)
x = state_vector(1);
y = state_vector(2);

J_g = [(2*x - 2*xm)/(2*((x - xm)^2 + (y - ym)^2)^(1/2)), (2*y - 2*ym)/(2*((x - xm)^2 + (y - ym)^2)^(1/2)), 0];

end

