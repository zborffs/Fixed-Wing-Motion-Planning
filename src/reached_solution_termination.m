function [value, isterminal, direction] = reached_solution_termination(t, state, p)
value      = (abs(state(AircraftStateIndex.x) - p.x_des) < p.abs_tol && abs(state(AircraftStateIndex.y) - p.y_des) < p.abs_tol && abs(state(AircraftStateIndex.z) - p.z_des) < p.abs_tol);
isterminal = 1;   % Stop the integration
direction  = 0;
end

