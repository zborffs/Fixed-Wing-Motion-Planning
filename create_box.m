function box_shape = create_box(n, l, w, h)
assert(n > 0);
assert(l > 0);
assert(w > 0);
assert(h > 0);

[x,y,z] = meshgrid(linspace(0, l, n), linspace(0, w, n), linspace(0, h, n));

P = [x(:) y(:), z(:)];

box_shape = alphaShape(P, n^3);

end

