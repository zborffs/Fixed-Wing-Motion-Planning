%% Create an "Environment" and "Aircraft" objects

% Generate a tree obstacle
tree_radius = 4;
tree_height = 40;
tree_x_pos = 10;
tree_y_pos = 10;
tree_z_pos = 0;
tree = create_tree(tree_x_pos, tree_y_pos, tree_z_pos, tree_radius, tree_height);

tree_radius = 4;
tree_height = 40;
tree_x_pos = 30;
tree_y_pos = 30;
tree_z_pos = 0;
tree2 = create_tree(tree_x_pos, tree_y_pos, tree_z_pos, tree_radius, tree_height);

tree_radius = 1;
tree_height = 40;
tree_x_pos = 55;
tree_y_pos = 45;
tree_z_pos = 0;
tree3 = create_tree(tree_x_pos, tree_y_pos, tree_z_pos, tree_radius, tree_height);

tree_radius = 2;
tree_height = 40;
tree_x_pos = 80;
tree_y_pos = 70;
tree_z_pos = 0;
tree4 = create_tree(tree_x_pos, tree_y_pos, tree_z_pos, tree_radius, tree_height);

tree_radius = 5;
tree_height = 40;
tree_x_pos = 55;
tree_y_pos = 25;
tree_z_pos = 0;
tree5 = create_tree(tree_x_pos, tree_y_pos, tree_z_pos, tree_radius, tree_height);

% generate a Forbidden Zone obstacle
forbidden_zone_length = 20;
forbidden_zone_width = 20;
forbidden_zone_height = 40; % -> will need to update later to make infinity
forbidden_x_pos = 50;
forbidden_y_pos = 50;
forbidden_z_pos = 0;
forbidden_zone = create_forbidden_zone(forbidden_zone_length, forbidden_zone_width, forbidden_zone_height, forbidden_x_pos, forbidden_y_pos, forbidden_z_pos);

ball = create_sphere(20, 2);
ball = translate_shape(ball, [10; 1; 25]);

ball2 = create_sphere(50, 8);
ball2 = translate_shape(ball2, [40; 30; 37]);

forbidden_zone_length = 40;
forbidden_zone_width = 1;
forbidden_zone_height = 40; % -> will need to update later to make infinity
forbidden_x_pos = -10;
forbidden_y_pos = 9;
forbidden_z_pos = 0;
forbidden_zone2 = create_forbidden_zone(forbidden_zone_length, forbidden_zone_width, forbidden_zone_height, forbidden_x_pos, forbidden_y_pos, forbidden_z_pos);

forbidden_zone_length = 60;
forbidden_zone_width = 1;
forbidden_zone_height = 40; % -> will need to update later to make infinity
forbidden_x_pos = -10;
forbidden_y_pos = -9;
forbidden_z_pos = 0;
forbidden_zone3 = create_forbidden_zone(forbidden_zone_length, forbidden_zone_width, forbidden_zone_height, forbidden_x_pos, forbidden_y_pos, forbidden_z_pos);

forbidden_zone_length = 1;
forbidden_zone_width = 30;
forbidden_zone_height = 40; % -> will need to update later to make infinity
forbidden_x_pos = -10+20;
forbidden_y_pos = 9;
forbidden_z_pos = 0;
forbidden_zone4 = create_forbidden_zone(forbidden_zone_length, forbidden_zone_width, forbidden_zone_height, forbidden_x_pos, forbidden_y_pos, forbidden_z_pos);

forbidden_zone_length = 1;
forbidden_zone_width = 30;
forbidden_zone_height = 40; % -> will need to update later to make infinity
forbidden_x_pos = -10+40;
forbidden_y_pos = 21;
forbidden_z_pos = 0;
forbidden_zone5 = create_forbidden_zone(forbidden_zone_length, forbidden_zone_width, forbidden_zone_height, forbidden_x_pos, forbidden_y_pos, forbidden_z_pos);

% Create the environment object, and add the obstacles to the environment
env = Environment();
% env.add_obstacle(forbidden_zone2);
% env.add_obstacle(forbidden_zone3);
% env.add_obstacle(forbidden_zone4);
% env.add_obstacle(forbidden_zone5);
% env.add_obstacle(forbidden_zone);
% env.add_obstacle(tree);
% env.add_obstacle(tree2);
% env.add_obstacle(tree3);
% env.add_obstacle(tree4);
% env.add_obstacle(tree5);
% env.add_obstacle(ball);
% env.add_obstacle(ball2);

% instantiate an Aircraft object
aircraft_x_pos = 0; % in meters
aircraft_y_pos = 0; % in meters
aircraft_z_pos = 10; % in meters
aircraft_heading = deg2rad(45); % in radians
aircraft_aoa = deg2rad(5); % angle of attack in radians
aircraft_detection_radius = 10; % in meters
aircraft_u_max = 5; % in m/s
aircraft_v_max = deg2rad(45); % in radians
aircraft_w_max = deg2rad(15); % in radians
aircraft_max_dist_to_obs = 3; % in meters
aircraft = Aircraft(aircraft_x_pos, aircraft_y_pos, aircraft_z_pos, ...
    aircraft_heading, aircraft_aoa, aircraft_detection_radius, ...
    aircraft_u_max, aircraft_v_max, aircraft_w_max, ...
    aircraft_max_dist_to_obs);

% draw the aircraft and environment
draw(aircraft, env);

% instantiate the "AircraftController" object with default constructor
controller = AircraftController();

%% Instantiate Simulation object and perform simulation
target_x_pos = 0; % in meters
target_y_pos = 0; % in meters
target_z_pos = 40; % in meters
target_abs_tol = 3; % in meters
sim = Simulation(env, aircraft, controller, ...
    [target_x_pos; target_y_pos; target_z_pos], target_abs_tol);

% simulate the system
tic; sim.simulate(); toc

% plot the simulation
sim.plot();

% animate the simulation
sim.animate();
