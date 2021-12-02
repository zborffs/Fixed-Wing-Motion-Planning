classdef Aircraft
    %UNTITLED9 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % state variables
        x
        y
        z
        theta
        alpha

        % parameters
        detection_radius
        u_max
        v_max
        w_max
        
        % specification
        max_dist_to_obstacle
    end
    
    methods
        function obj = Aircraft()
            % state variables
            obj.x = 0;
            obj.y = 0;
            obj.z = 0;
            obj.theta = 0;
            obj.alpha = 0;

            % parameters
            obj.detection_radius = 10;
            obj.u_max = 5;
            obj.v_max = deg2rad(10); % rad/s
            obj.w_max = deg2rad(10); % rad/s

            % specification
            obj.max_dist_to_obstacle = 5;
        end
        
        function aircraft_shape = get_shape(obj)
            % 5 is based on specification
            aircraft_shape = create_sphere(50, obj.max_dist_to_obstacle);
            aircraft_shape = translate_shape(aircraft_shape, [obj.x;obj.y;obj.z]);
        end
    end
end

