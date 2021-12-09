function is_safe = is_aircraft_safe(aircraft, environment)
% determines whether a the aircraft is in an safe or unsafe state with
% respect to the environment
% :param aircraft: 
% :param environment: 
% :return: 

is_safe = true;

aircraft_shape = aircraft.get_shape();
x1 = aircraft_shape.Points(:,1);
y1 = aircraft_shape.Points(:,2);
z1 = aircraft_shape.Points(:,3);

for i = 1:length(environment.obstacles)
    
    x2 = environment.obstacles{i}.Points(:,1);
    y2 = environment.obstacles{i}.Points(:,2);
    z2 = environment.obstacles{i}.Points(:,3);
    
    id1 = inShape(environment.obstacles{i}, x1, y1, z1);
    id2 = inShape(aircraft_shape, x2, y2, z2);
    
    P = [[x1(id1); x2(id2)], [y1(id1); y2(id2)], [z1(id1); z2(id2)]];
    P = unique(P, 'rows');
    
    shp = alphaShape(P,1);
    V = volume(shp);
    
    if V ~= 0
        is_safe = false;
        break;
    end
end

end

