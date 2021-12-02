function shp = create_tree(x,y,z,r,h)
shp = create_cylinder(100, r, h);
shp = translate_shape(shp, [x; y; z]);
end

