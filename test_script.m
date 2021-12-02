% <ecarril2@terpmail.umd.edu>
% - Estefany Carrillo

% addpath("../trunk"); % to get staliro

aircraft = Aircraft();
p = struct('x_des', 300, 'y_des', 300, 'z_des', 20, 'aircraft', aircraft, 'environment', env);

% Model (3D Dubins)
model = @(t, state) dubins3d(t, state, p);

[t, out] = ode45(model,[0 20],[0; 0; 0; deg2rad(45); deg2rad(1)]);

x = out(:,1);
y = out(:,2);
z = out(:,3);
theta = out(:,4);
alpha = out(:,5);

% Initial Conditions
% - how should we start (x,y,z have hard bounds, but (u,v,w) have
% variable bounds

% Input Range
% - constraints on u,v,w
inpu

% cp_array
% - should empty set since input range is defined

% phi: formula to falsify in form of string
% - should be the no-go zones + that we eventually end up at the target
% -> NEVER


% preds: predicate mappings from atomic propositions in the formula to
% predicates over the state space or the output space of the model

% TotSimTime: total simulation time

% opt: options

% help staliro