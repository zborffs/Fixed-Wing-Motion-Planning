function shp = create_forbidden_zone(l,w,h,x,y,z)
shp = create_box(10, l, w, h);
shp = translate_shape(shp, [x; y; z]);
end

