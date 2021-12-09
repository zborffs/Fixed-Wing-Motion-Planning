% Todo:
% 1. clean up constructor argument list (make shorter)
% 2. add temporal safe state to the shape of the object

classdef Aircraft
    % Aircraft class maintains the aircraft state information for a given
    % sample of a simulation. It also maintains parameters related to the
    % detection radius, the bounds on the control signals, and finally the
    % specification related to the distance between objects
    
    properties
        % Aircraft State Variables
        x % longitudinal distance in meters
        y % latitudinal distance in meters
        z % altitude in meters
        theta % heading (angle w.r.t. x-axis) in radians
        alpha % angle-of-attack (angle w.r.t xy-plane) in radians

        % Aircraft Parameters
        detection_radius % max distance that we can detect obstacles in meters
        u_max % maximum forward velocity in m/s -> must be > 0
        v_max % maximum heading stearing angle in radians -> must be (-pi/2,pi/2) exclusive
        w_max % maximum angle-of-attack stearing angle in radians -> must be (-pi/2,pi/2) exclusive
        
        % Safety Specification
        max_dist_to_obstacle % spatially safe state distance spec in meters
    end
    
    methods
        function obj = Aircraft(x,y,z,theta,alpha,det_rad,u_max,v_max,w_max,max_dist_to_obs)
            % Constructs Aircraft object from user-defined initial values
            % :param x: longitudinal distance in meters
            % :param y: latitudinal distance in meters
            % :param z: altitude in meters
            % :param theta: heading (angle w.r.t. x-axis) in radians
            % :param alpha: angle-of-attack (angle w.r.t. xy-plane) in
            % radians
            % :param det_rad: detection radius in meters
            % :param u_max: maximum forward velocity in m/s (must be > 0)
            % :param v_max: maximum heading stearing angle in radians (must
            % be in range (-pi/2,pi/2) exclusive)
            % :param w_max: maximum angle-of-attack stearing angle in 
            % radians (must be in range (-pi/2,pi/2) exclusive)
            % :param max_dist_to_obs: spatially safe state distance spec in
            % meters
            % :return: an Aircraft instance
            
            % make sure the aircraft control bounds are as we expect
            assert(u_max > 0);
            assert(v_max > -pi/2 && v_max < pi/2);
            assert(w_max > -pi/2 && w_max < pi/2);
            
            % initialize state variables
            obj.x = x;
            obj.y = y;
            obj.z = z;
            obj.theta = theta;
            obj.alpha = alpha;

            % initialize parameters
            obj.detection_radius = det_rad;
            obj.u_max = u_max;
            obj.v_max = v_max; % rad/s
            obj.w_max = w_max; % rad/s

            % initialize specification
            obj.max_dist_to_obstacle = max_dist_to_obs;
        end
        
        function aircraft_shape = get_shape(obj)
            % get_shape returns the 'alphaShape' representation of the
            % aircraft, which is the union of a sphere of radius 
            % 'max_dist_to_obstacle' centered at the aircraft's position
            % and an ellipsoid whose x-axis radius is the distance safety
            % spec, y-axis radius is the maximum turning radius in the
            % xy-plane, and z-axis radius is the maximum turning radius
            % along the z-axis
            % :param obj: the Aircraft object calling this function
            % :return: an alphaShape representation of the aircraft
            
            % create the ellipsoid base object with radius of the safety-spec
            aircraft_shape = create_ellipsoid(20, obj.x, obj.y, obj.z, ...
                obj.max_dist_to_obstacle, ...
                max(obj.max_dist_to_obstacle, 2 / tan(obj.v_max)), ...
                max(obj.max_dist_to_obstacle, 2 / tan(obj.w_max)));
        end
    end
end

