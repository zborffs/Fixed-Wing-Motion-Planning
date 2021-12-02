% create a sphere
[x1, y1, z1] = sphere(50);
x1 = x1(:);
y1 = y1(:);
z1 = z1(:);
x2 = x1+1;
P = [x1 y1 z1];
P = unique(P,'rows');
P2 = [x2 y1 z1];
P2 = unique(P2, 'rows');

shp = alphaShape(P, 1);
plot(shp); hold on;
shp2 = alphaShape(P2, 1);
plot(shp2); hold off;

id1=inShape(shp2,x1,y1,z1); % points of shp in shp2
id2=inShape(shp,x2,y1,z1); % points of shp2 in shp
P3 = [[x1(id1); x2(id2)], [y1(id1); y1(id2)], [z1(id1); z1(id2)]];
P3 = unique(P3, 'rows');
shp3=alphaShape(P3, 1); % new shape representing intersection
plot(shp3);

V=volume(shp3); % volume of shape 3

% create a new shape!
[xg, yg, zg] = meshgrid(-1:0.25:1);
Pcube = [xg(:) yg(:), zg(:)];
shape = alphaShape(Pcube);
plot(shape); hold on; plot(shp2); hold off;

id1=inShape(shp2,xg,yg,zg); % points of shp in shp2
id2=inShape(shape,x2,y1,z1); % points of shp2 in shp
P3 = [[xg(id1); x2(id2)], [yg(id1); y1(id2)], [zg(id1); z1(id2)]];
P3 = unique(P3, 'rows');
shp3=alphaShape(P3, 1); % new shape representing intersection
plot(shp3);

V=volume(shp3); % volume of shape 3

% create a cylinder
cylinder_shape = create_cylinder(48, 2, 4);
plot(cylinder_shape); % make sure this has radius of 2 and height of 4

