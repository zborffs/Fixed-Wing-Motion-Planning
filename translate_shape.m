function new_shp = translate_shape(shp, v)

    assert(all(size(v) == [3 1]));

    x = v(1);
    y = v(2);
    z = v(3);

    new_shp = shp;
    new_shp.Points(:,1) = new_shp.Points(:,1) + x;
    new_shp.Points(:,2) = new_shp.Points(:,2) + y;
    new_shp.Points(:,3) = new_shp.Points(:,3) + z;

end

