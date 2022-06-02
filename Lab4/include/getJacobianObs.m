function [J_g] = getJacobianObs(state_vector, xm, ym)
delta_x = 0.035;
x = state_vector(1);
y = state_vector(2);
theta = state_vector(3);

% J_g = [-(2*delta_x - 4*state_vector(1) + 2*xm + 2*delta_x*cos(state_vector(3)) - 4*state_vector(1)*cos(state_vector(3)) + 2*xm*cos(state_vector(3)) + 2*ym*sin(state_vector(3)))/(2*((xm - state_vector(1) + delta_x*cos(state_vector(3)) - state_vector(1)*cos(state_vector(3)) + state_vector(2)*sin(state_vector(3)))^2 + (state_vector(2) - ym - delta_x*sin(state_vector(3)) + state_vector(2)*cos(state_vector(3)) + state_vector(1)*sin(state_vector(3)))^2)^(1/2))  (4*state_vector(2) - 2*ym - 2*delta_x*sin(state_vector(3)) + 4*state_vector(2)*cos(state_vector(3)) - 2*ym*cos(state_vector(3)) + 2*xm*sin(state_vector(3)))/(2*((xm - state_vector(1) + delta_x*cos(state_vector(3)) - state_vector(1)*cos(state_vector(3)) + state_vector(2)*sin(state_vector(3)))^2 + (state_vector(2) - ym - delta_x*sin(state_vector(3)) + state_vector(2)*cos(state_vector(3)) + state_vector(1)*sin(state_vector(3)))^2)^(1/2)) (2*(state_vector(2)*cos(state_vector(3)) - delta_x*sin(state_vector(3)) + state_vector(1)*sin(state_vector(3)))*(xm - state_vector(1) + delta_x*cos(state_vector(3)) - state_vector(1)*cos(state_vector(3)) + state_vector(2)*sin(state_vector(3))) - 2*(delta_x*cos(state_vector(3)) - state_vector(1)*cos(state_vector(3)) + state_vector(2)*sin(state_vector(3)))*(state_vector(2) - ym - delta_x*sin(state_vector(3)) + state_vector(2)*cos(state_vector(3)) + state_vector(1)*sin(state_vector(3))))/(2*((xm - state_vector(1) + delta_x*cos(state_vector(3)) - state_vector(1)*cos(state_vector(3)) + state_vector(2)*sin(state_vector(3)))^2 + (state_vector(2) - ym - delta_x*sin(state_vector(3)) + state_vector(2)*cos(state_vector(3)) + state_vector(1)*sin(state_vector(3)))^2)^(1/2))];
J_g = [(2*x - 2*xm)/(2*((x - xm)^2 + (y - ym)^2)^(1/2)), (2*y - 2*ym)/(2*((x - xm)^2 + (y - ym)^2)^(1/2)), 0];

% J_g = [-(2*delta_x - 4*x + 2*xm + 2*delta_x*cos(theta) - 4*x*cos(theta) + 2*xm*cos(theta) + 2*ym*sin(theta))/(2*((xm - x + delta_x*cos(theta) - x*cos(theta) + y*sin(theta))^2 + (y - ym - delta_x*sin(theta) + y*cos(theta) + x*sin(theta))^2)^(1/2)), 
%     (4*y - 2*ym - 2*delta_x*sin(theta) + 4*y*cos(theta) - 2*ym*cos(theta) + 2*xm*sin(theta))/(2*((xm - x + delta_x*cos(theta) - x*cos(theta) + y*sin(theta))^2 + (y - ym - delta_x*sin(theta) + y*cos(theta) + x*sin(theta))^2)^(1/2)), 
%     (2*(y*cos(theta) - delta_x*sin(theta) + x*sin(theta))*(xm - x + delta_x*cos(theta) - x*cos(theta) + y*sin(theta)) - 2*(delta_x*cos(theta) - x*cos(theta) + y*sin(theta))*(y - ym - delta_x*sin(theta) + y*cos(theta) + x*sin(theta)))/(2*((xm - x + delta_x*cos(theta) - x*cos(theta) + y*sin(theta))^2 + (y - ym - delta_x*sin(theta) + y*cos(theta) + x*sin(theta))^2)^(1/2))];
% 
% J_g = J_g';
end

