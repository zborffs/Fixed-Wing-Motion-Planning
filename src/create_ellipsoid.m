function ellipsoid_shape = create_ellipsoid(n, x, y, z, xr, yr, zr)

[x,y,z] = ellipsoid(x,y,z,xr,yr,zr,n);

P = [x(:) y(:) z(:)];
P = unique(P, 'rows');

ellipsoid_shape = alphaShape(P, n);


end

