%% Create an "Environment" and "Aircraft" objects

% start by generating all the obstacles
tree_radius = 4;
tree_height = 40;
tree_x_pos = 10;
tree_y_pos = 10;
tree_z_pos = 0;
tree = create_tree(tree_x_pos, tree_y_pos, tree_z_pos, tree_radius, tree_height);


forbidden_zone_length = 100;
forbidden_zone_width = 100;
forbidden_zone_height = 30; % -> will need to update later to make infinity
forbidden_x_pos = 100;
forbidden_y_pos = 100;
forbidden_z_pos = 0;
forbidden_zone = create_forbidden_zone(forbidden_zone_length, forbidden_zone_width, forbidden_zone_height, forbidden_x_pos, forbidden_y_pos, forbidden_z_pos);

% Create the environment object by passing in the objects
env = Environment();
env.add_obstacle(forbidden_zone);
env.add_obstacle(tree);

% instantiate the "Aircraft" object with default constructor
aircraft = Aircraft();

% instantiate the "AircraftController" object with default constructor
controller = AircraftController();

%% Instantiate Simulation object and perform simulation
sim = Simulation(env, aircraft, controller, [300; 300; 20], 10);
sim.simulate();
sim.plot();
% sim.animate(); % -> not implemented yet!

out = sim.out;
x = out(:,1); y = out(:,2); z=out(:,3);theta=out(:,4);alpha=out(:,5);
my_time = 500;
aircraft.x = x(my_time); aircraft.y = y(my_time); aircraft.z = z(my_time); aircraft.theta = theta(my_time); aircraft.alpha = alpha(my_time);
[X,Y,Z,TF] = detectable_objects(aircraft, env);
plot_rays(X,Y,Z,TF);

in = find(TF);
Xin = X(in);
Yin = Y(in);
Zin = Z(in);

scatter3(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z, '.')

[az,el,r] = cart2sph(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z);
P1 = [az el r];
histogram(P1(:,1), 32);
[n_az1, bins_az] = histcounts(P1(:,1), 31);
[n_el1, bins_el] = histcounts(P1(:,2), 31);

% different
my_time = 61;
aircraft.x = x(my_time); aircraft.y = y(my_time); aircraft.z = z(my_time); aircraft.theta = theta(my_time); aircraft.alpha = alpha(my_time);
[X,Y,Z,TF] = detectable_objects(aircraft, env);
plot_rays(X,Y,Z,TF);

in = find(TF);
Xin = X(in);
Yin = Y(in);
Zin = Z(in);

scatter3(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z, '.')

[az,el,r] = cart2sph(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z);
P2 = [az el r];
histogram(P2(:,1), 32);
[n_az2, bins_az2] = histcounts(P2(:,1), 31);
[n_el2, bins_el2] = histcounts(P2(:,2), 31);
[n_r2, bins_r2] = histcounts(P2(:,3), 31);

plot([n_az1; n_az2]')
plot([n_el1; n_el2]')
plot(n_az2-n_az1); % identifies dangerous areas along thetas
plot(n_el2-n_el1); % identifies dangerous areas along alphas
sgn_az = sign(n_az2-n_az1);
sgn_el = sign(n_el2-n_el1);
stairs(sgn_az); ylim([-1.1 1.1]);
stairs(sgn_el); ylim([-1.1 1.1]);
indices_az = sgn_az > 0;
indices_el = sgn_el > 0;


indices_az2 = false(1,length(indices_az)+1);
for i = 1:length(indices_az)
    if indices_az(i)
        indices_az2(i:i+1) = true;
    end
end
angles_az = bins_az2(indices_az2);

indices_el2 = false(1,length(indices_el)+1);
for i = 1:length(indices_el)
    if indices_el(i)
        indices_el2(i:i+1) = true;
    end
end
angles_el = bins_el2(indices_el2);

middle_indexes = [];
rmin = [];
elmin = [];
azmin = [];

assert(all(size(el) == size(az)));
for i = 1:length(az)
    for j = 1:length(angles_el)-1
        for k = 1:length(angles_az)-1
            if az(i) >= angles_az(k) && az(i) <= angles_az(k+1) && el(i) >= angles_el(j) && el(i) <= angles_el(j+1)
                middle_indexes(end+1) = i;

                if isempty(rmin)
                    rmin = r(i);
                    elmin = el(i);
                    azmin = az(i);
                elseif r(i) < rmin
                    rmin = r(i);
                    elmin = el(i);
                    azmin = az(i);
                end
            end
        end
    end
end

middle_indexes = middle_indexes';
Pmiddle = [r(middle_indexes) az(middle_indexes) el(middle_indexes)];

bpoints = [0];

if length(Pmiddle) > 1
    for i = 2:length(Pmiddle)
        if abs(Pmiddle(i,2) - Pmiddle(i-1,2)) > 0.000001 || abs(Pmiddle(i,3) - Pmiddle(i-1,3)) > 0.000001
            bpoints(end+1) = i-1;
        end
    end
end
bpoints(end+1) = length(Pmiddle);

rpoints = {};
for i = 1:length(bpoints)-1
    rpoints{end+1} = Pmiddle(bpoints(i)+1:bpoints(i+1));
end
assert(all(length([rpoints{:}]) == length(Pmiddle)));


scatter3(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z, '.'); hold on;
[xmin, ymin, zmin] = sph2cart(azmin, elmin, rmin);
plot3([0 xmin], [0 ymin], [0 zmin], 'r-o');

% rdot, r, => u,v,w
