function out = dubins3d(t, state, p)
% implements numerically integrable 3D Dubins model without stearing rate
% state variables
% :param t: time vector
% :param x: model variables (x, y, z, theta, alpha)
% :returns: derivatives of state variables

% update the 'aircraft' position, heading, and angle of attack
p.aircraft.x = state(AircraftStateIndex.x);
p.aircraft.y = state(AircraftStateIndex.y);
p.aircraft.z = state(AircraftStateIndex.z);
p.aircraft.theta = state(AircraftStateIndex.theta);
p.aircraft.alpha = state(AircraftStateIndex.alpha);

% Call controller function to generate control signals
[u,v,w] = p.controller.gen_control(t, p.aircraft, p.environment, p.x_des, p.y_des, p.z_des);

% use control signals and current state variables to derive state variable 
% derivatives
out = [
    u * cos(state(AircraftStateIndex.theta)) * cos(state(AircraftStateIndex.alpha));
    u * sin(state(AircraftStateIndex.theta)) * cos(state(AircraftStateIndex.alpha));
    u * sin(state(AircraftStateIndex.alpha));
    u * tan(v);
    u * tan(w);
];

end

