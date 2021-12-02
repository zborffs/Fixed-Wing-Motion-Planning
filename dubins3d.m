function out = dubins3d(t, state, p)
% implements numerically integrable dubins3d model
%
% :param t: time vector
% :param x: model variables (x, y, z, theta, alpha)
% :returns: new configuration of dubins

p.aircraft.x = state(AircraftStateIndex.x);
p.aircraft.y = state(AircraftStateIndex.y);
p.aircraft.z = state(AircraftStateIndex.z);
p.aircraft.theta = state(AircraftStateIndex.theta);
p.aircraft.alpha = state(AircraftStateIndex.alpha);

[u,v,w] = p.controller.global_control(t, p.aircraft, p.environment, p.x_des, p.y_des, p.z_des);

out = [
    u * cos(state(AircraftStateIndex.theta)) * cos(state(AircraftStateIndex.alpha));
    u * sin(state(AircraftStateIndex.theta)) * cos(state(AircraftStateIndex.alpha));
    u * sin(state(AircraftStateIndex.alpha));
    v;
    w;
];

end

