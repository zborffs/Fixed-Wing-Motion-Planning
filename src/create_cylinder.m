function cylander_shape = create_cylinder(n, r, h)

% make sure the discretization number, the radius, and height are positive
assert(r > 0);
assert(n > 0);
assert(h > 0);

% parameterize the a circle of radius 'r' by 't' sampled 'n' times
t = (pi/24:2*pi/n:2*pi)';
x = r * cos(t);
y = r * sin(t);

% create a height vector
z = ones(numel(x),1);

% repeat the x matrix
k = 5; % arbitrary value for the number of vertical meshes 

myheight = linspace(0, h, k);

xg = repmat(x, k, 1); 
yg = repmat(y, k, 1);
zg = z*(myheight); % notice that 0:0.25:1 is 48 x 5 matrix
zg = zg(:);

P = [xg yg zg];
P = unique(P, 'rows');
cylander_shape = alphaShape(P,1000); % 1000?

end

