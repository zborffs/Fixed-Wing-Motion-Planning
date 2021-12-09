function sphere_shape = create_sphere(n, r)
assert(r > 0);
assert(n > 0);

[x1, y1, z1] = sphere(n);
x1 = r * x1(:);
y1 = r * y1(:);
z1 = r * z1(:);
P = [x1 y1 z1];
P = unique(P, 'rows');

sphere_shape = alphaShape(P, r);

end

