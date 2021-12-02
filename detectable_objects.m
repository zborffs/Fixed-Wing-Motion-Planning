function [x, y, z, tf] = detectable_objects(aircraft, env)

% 100 x 20 x 20 = 40,000
r = linspace(0, aircraft.detection_radius, 100);
theta = linspace(0, 2*pi, 50);
phi = linspace(0, pi, 21);
[theta,r,phi] = meshgrid(theta', r', phi'); % r must be 2nd!
r = r(:);
theta = theta(:);
phi = phi(:);

x = r .* cos(theta) .* sin(phi) + aircraft.x;
y = r .* sin(theta) .* sin(phi) + aircraft.y;
z = r .* cos(phi) + aircraft.z;

tf = false(length(phi),1);

for i = 1:length(env.obstacles)
    tf = tf | inShape(env.obstacles{i}, x, y, z) | z < 0;
end

for i = 1:length(tf)
    j = mod(i-1, 100)+1;
    if tf(i)
        tf(i:i+100-j) = true;
    end
end

end

