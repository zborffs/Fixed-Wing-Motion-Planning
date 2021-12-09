% Todo:
% 1. Cleanup argument list (make shorter)
% 2. Figure out ideal min_safe_distance

classdef AircraftController < handle
    % AircraftController class maintains data and implments functions
    % responsible for computing control signals from the current state of
    % the aircraft and knowledge of the environment.

    properties
        % Global 'heading' controller coefficients (theta => v)
        theta_variables
        
        % Global 'angle of attack' controller coefficients (alpha => w)
        alpha_variables

        % previous time instant
        t_prev

        % Previous bincount (for detecting objects)
        bincount_prev

        % Previous minimum distance (for detecting objects
        rmin_prev

        % ideal distance from obstacle surface 
        % - (I think this should be slightly less than detection radius in "Aircraft")
        % - this might potentially be set by specification!
        min_safe_distance
    end
    
    methods
        function obj = AircraftController()
            % AircraftController is the default constructor for the
            % AircraftController class
            % :return: AircraftController Object

            obj.theta_variables = [0; 0; 0]; % initialize controller gains all to 0
            obj.alpha_variables = [0; 0; 0]; % initialize controller gains all to 0
            
            obj.t_prev = []; % initialize previous time as empty set
            obj.bincount_prev = []; % initialize previous bincount as empty set
            obj.rmin_prev = []; % initialize previous minimum radius as empty set

            obj.min_safe_distance = 7; % initialize the minimum safe distance as 8
        end

        function [u,v,w] = global_control(obj, T, aircraft, x_des, y_des, z_des)
            % global_control applies the global control law to the aircraft
            % whereby we simply point ourselves to the centroid of the
            % target region
            % :param obj: 'this' object
            % :param T: the time interval between this sample and the
            % previous sample (not fixed)
            % :param aircraft: Aircraft object (holds state info)
            % :param x_des: target x-position in meters
            % :param y_des: target y-position in meters
            % :param z_des: target z-position in meters
            % :return: control signals u (forward velocity in m/s), v
            % (heading derivative in rad/s) and w (angle-of-attack velocity
            % in rad/s)

            % time-optimal trajectory means getting there as soon as
            % possible, so go as fast as is possible for the aircraft to go
            % (needs to be changed to ensure that the max speed guarantees 
            % detecting object and avoiding it)
            u = aircraft.u_max;

            % the desired heading (theta) reference value
            theta_des = atan2(y_des - aircraft.y, x_des - aircraft.x);

            % the desired angle-of-attack (alpha) reference value
            alpha_des = atan2(z_des - aircraft.z, sqrt((x_des - aircraft.x)^2 + (y_des - aircraft.y)^2));

            % error signal for theta and alpha
%             theta_error = theta_des - aircraft.theta;
            theta_error = angdiff(aircraft.theta, theta_des); % interesting
            alpha_error = alpha_des - aircraft.alpha;
            
            % coefficients for VERY basic lead controller (s+2)/(s+1)
            zero_loc = 4; %4 zero_loc = 2 means zero is actually at -2
            pole_loc = 3; %3
            gain = 3; % 3
            A = 2 * (pole_loc + T); % 1 represents pole location
            B = 2 * (-pole_loc + T); % 1 represents pole location
            C = zero_loc + T; % 2 represents zero location
            D = (-zero_loc + T); % 2 represents zero location

            % update the coefficients for the theta controller
            obj.theta_variables(2) = obj.theta_variables(1);
            obj.theta_variables(1) = theta_error;

            % update the coefficients for the alpha controller
            obj.alpha_variables(2) = obj.alpha_variables(1);
            obj.alpha_variables(1) = alpha_error;

            % theta digitzed lead controller
            v = (gain * A * obj.theta_variables(1) + gain * B * obj.theta_variables(2) - D * obj.theta_variables(3)) / C;

            % alpha digitzed lead controller
            w = (A * obj.alpha_variables(1) + B * obj.alpha_variables(2) - D * obj.alpha_variables(3)) / C;

            % update the [k-1] term for the next iteration
            obj.theta_variables(3) = v;
            obj.alpha_variables(3) = w;
            
            % scaling to cancel nonlinearities
            v = atan(v / u);
            w = atan(w / u);
        end

        function [u,v,w] = reactive_control(obj, T, aircraft, x_des, y_des, z_des, rpoints)
            % applies reactive control law in case the plane is too close
            % to an obstacle (overrides the global control law to get to
            % the target
            % :param obj: 'this' object
            % :param T: time step between current sample and previous
            % sample (not fixed)
            % :param aircraft: Aircraft object (holds state)
            % :param x_des: target x-position in meters
            % :param y_des: target y-position in meters
            % :param z_des: target z-position in meters
            % :param rpoints: distance, azimuth, and elevation of obstacle
            % point to the UAV
            % :return: control signals u (forward velocity in m/s), v
            % (heading derivative in rad/s) and w (angle-of-attack velocity
            % in rad/s)

            u = zeros(length(rpoints),1);
            v = zeros(length(rpoints),1);
            w = zeros(length(rpoints),1);
            k = 1; % arbitrary variable saying how much 
            K = 1;
            
            for i = 1:length(rpoints)
                r = rpoints{i}(1);
                theta = rpoints{i}(2);
                phi = rpoints{i}(3);
                [x,y,z] = sph2cart(theta, phi, r);
                Fx = -k * x / r^2;
                Fy = -k * y / r^2;
                Fz = -k * z / r^2;
                [v(i),w(i),u(i)] = cart2sph(Fx, Fy, Fz);
            end
            
            F_goal = K * ([x_des;y_des;z_des] - [aircraft.x; aircraft.y; aircraft.z]);
            [v_goal, w_goal, u_goal] = cart2sph(F_goal(1), F_goal(2), F_goal(3));
            
            theta_des = mean([median(v), v_goal]); % theta setpoint
            alpha_des = mean([median(w), w_goal]); % alpha setpoint
%             sumU = sum(u);
%             theta_des = (median(v) * sumU + v_goal * u_goal) / (sumU + u_goal); % theta setpoint
%             alpha_des = (median(w) * sumU + w_goal * u_goal) / (sumU + u_goal); % alpha setpoint

            % error si10gnal for theta and alpha
%             theta_error = theta_des - aircraft.theta;
            theta_error = angdiff(aircraft.theta, theta_des);
            alpha_error = alpha_des - aircraft.alpha;
            
            % sum(u) + u_goal will tell you the magnitude of the 
            u_interm = (sum(u) + u_goal) * cos(theta_error) * cos(alpha_error);
            u = aircraft.u_max / (1+exp(-k * u_interm));

            % coefficients for VERY basic lead controller (s+2)/(s+1)
            zero_loc = 4; %4 zero_loc = 2 means zero is actually at -2
            pole_loc = 3; %3
            gain = 2; % 3
            A = 2 * (pole_loc + T); % 1 represents pole location
            B = 2 * (-pole_loc + T); % 1 represents pole location
            C = zero_loc + T; % 2 represents zero location
            D = (-zero_loc + T); % 2 represents zero location

            % update the coefficients for the theta controller
            obj.theta_variables(2) = obj.theta_variables(1);
            obj.theta_variables(1) = theta_error;

            % update the coefficients for the alpha controller
            obj.alpha_variables(2) = obj.alpha_variables(1);
            obj.alpha_variables(1) = alpha_error;

            % theta digitzed lead controller
            v = (gain * A * obj.theta_variables(1) + gain * B * obj.theta_variables(2) - D * obj.theta_variables(3)) / C;

            % alpha digitzed lead controller
            w = (A * obj.alpha_variables(1) + B * obj.alpha_variables(2) - D * obj.alpha_variables(3)) / C;

            % update the [k-1] term for the next iteration
            obj.theta_variables(3) = v;
            obj.alpha_variables(3) = w;
            
            % scaling to cancel nonlinearities
            v = atan(v / min(u, aircraft.u_max));
            w = atan(w / min(u, aircraft.u_max));
        end
        
        function [u,v,w] = gen_control(obj, t, aircraft, env, x_des, y_des, z_des)
            % generates the control signals given the current state of the
            % aircraft
            % :param obj: 'this' object
            % :param t: the current time instant in seconds (double, ex: 0.251 [sec])
            % :param aircraft: the aircraft object (aircraft state)
            % :param env: the environment object (obstacles)
            % :param x_des: desired x-position in meters (double, ex: 100)
            % :param y_des: desired y-position in meters (double, ex: -200)
            % :param z_des: desired z-position in meters (double, ex: 15.1)
            % :return: u (forward velocity in m/s), v (heading derivative
            % rad/s), w (angle-of-attack derivative rad/s)

            % Determine the direction and distance to the closest obstacle
            [rmin,~,~,rpoints] = obj.find_closest_danger(aircraft, env);


            % wait till we have at least 1 data point before we start
            % actuating anything, so if the t_prev member variable is still
            % the empty, set just skip all of the control law stuff
            if ~isempty(obj.t_prev)
                % we have at least 1 previous data point, so let's start
                % actuating the plane!

                T = t - obj.t_prev; % time difference (sampling time)

                % if we have data related to the distance to the closest
                % obstacle, then check to see whether we are too close to
                % any obstacles!
                if ~isempty(rpoints)
                    % if we are too close to the closest obstacle, then
                    % initialize the reactive controller
                    [u,v,w] = obj.reactive_control(T, aircraft, x_des, y_des, z_des, rpoints);
                else
                    % if we are NOT too close to an obstacle, then use the
                    % global controller
                    [u,v,w] = obj.global_control(T, aircraft, x_des, y_des, z_des);
                end
            else
                % set the nominal controls
                obj.t_prev = t;
                u = aircraft.u_max; % whatever
                v = 0;
                w = 0;
            end

            % update the previous minimum obstacle distance member variable
            obj.rmin_prev = rmin;
            
%             disp("t : " + t + ", u : " + u + ", v : " + v + ", w : " + w + ", x : " + aircraft.x + ", y : " + aircraft.y + ", z : " + aircraft.z);
            
            % make sure we only output our capacity
            u = max(min(aircraft.u_max, u), 0);
            v = max(min(aircraft.v_max, v), -aircraft.v_max);
            w = max(min(aircraft.w_max, w), -aircraft.w_max);
            
        end

        function [rmin, azmin, elmin, rpoints] = find_closest_danger(obj, aircraft, env)
            % finds the closest obstacle within 'detection_radius' meters
            % and returns its distance to the plane, its heading, and its
            % angle-of-attack
            % :param aircraft: Aircraft object (holds state info)
            % :param env: Environment object (holds obstalce info)
            % :return: rmin (distance to closest obstacle in meters), azmin
            % (heading of closest object in radians), and elmin
            % (angle-of-attack of closest object in radians), and rpoints,
            % the distance, azimuth, and elevation of all the points we can
            % see that are getting closer

            % Get the set of points around the object and check to see if
            % they intersect with an object (a la ray-tracing so anything
            % behind an obstacle is can't be seen)
            [X,Y,Z,TF] = detectable_objects(aircraft, env);


            % Find the subset of points that intersect or we can't see
            % behind
            in = find(TF);
            Xin = X(in);
            Yin = Y(in);
            Zin = Z(in);

            % Convert cartesian to spherical and place in single matrix
            [az,el,r] = cart2sph(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z);
            P2 = [az el r];

            % break down each coordinate down into frequency
            [n_az2, bins_az2] = histcounts(P2(:,1), 31);
            [n_el2, bins_el2] = histcounts(P2(:,2), 31);

            % if we don't know what those frequencies were from the
            % previous iteration, then just terminate, since its impossible
            % to infer where the objects are otherwise
            if ~isempty(obj.bincount_prev)
                % if we have information about the previous iteration's ray
                % trace, then grab the bincount from the previous iteration
                n_az1 = obj.bincount_prev(1,:);
                n_el1 = obj.bincount_prev(2,:);
                
                % Take the difference btw the traces to see what objects
                % are getting closer and which are moving away
                sgn_az = sign(n_az2-n_az1);
                sgn_el = sign(n_el2-n_el1);

                % Only get the indices of the objects that are getting
                % closer
                indices_az = sgn_az > 0;
                indices_el = sgn_el > 0;

                % Find the azimuth (heading) associated with the objects
                % that are getting closer
                indices_az2 = false(1,length(indices_az)+1);
                for i = 1:length(indices_az)
                    if indices_az(i)
                        indices_az2(i:i+1) = true;
                    end
                end
                angles_az = bins_az2(indices_az2);

                % Find the elevation (angle-of-attack) associated with the 
                % objects that are getting closer
                indices_el2 = false(1,length(indices_el)+1);
                for i = 1:length(indices_el)
                    if indices_el(i)
                        indices_el2(i:i+1) = true;
                    end
                end
                angles_el = bins_el2(indices_el2);

                % of the set of points that are getting closer, get the
                % closest point and return its distance, heading, and
                % elevation
                middle_indexes = [];
                rmin = [];
                elmin = [];
                azmin = [];

                assert(all(size(el) == size(az)));
                for i = 1:length(az)
                    for j = 1:length(angles_el)-1
                        for k = 1:length(angles_az)-1
                            if az(i) >= angles_az(k) && az(i) <= angles_az(k+1) && el(i) >= angles_el(j) && el(i) <= angles_el(j+1)
                                middle_indexes(end+1) = i;

                                if isempty(rmin)
                                    rmin = r(i);
                                    elmin = el(i);
                                    azmin = az(i);
                                elseif r(i) < rmin
                                    rmin = r(i);
                                    elmin = el(i);
                                    azmin = az(i);
                                end
                            end
                        end
                    end
                end

                % we might add this code later. this code will just get ALL
                % the individual TRACES rather than just the closest trace.
                % This might be relevant later if we notice the reactive
                % controller needs information about all the objects around
                % it, not just the closest object
                middle_indexes = middle_indexes';
                Pmiddle = [r(middle_indexes) az(middle_indexes) el(middle_indexes)];

                bpoints = [0];
                rpoints = {};

                if size(Pmiddle,1) > 1
                    for i = 2:size(Pmiddle,1)
                        if abs(Pmiddle(i,2) - Pmiddle(i-1,2)) > 0.000001 || abs(Pmiddle(i,3) - Pmiddle(i-1,3)) > 0.000001
                            bpoints(end+1) = i-1;
                        end
                    end
                    
                    bpoints(end+1) = size(Pmiddle,1);
                    

                    for i = 1:length(bpoints)-1
%                         rpoints{end+1} = Pmiddle(bpoints(i)+1:bpoints(i+1));
                        rpoints{end+1} = [Pmiddle(bpoints(i)+1,1); Pmiddle(bpoints(i)+1,2); Pmiddle(bpoints(i)+1,3)];
                    end

%                     assert(all(length([rpoints{:}]) == size(Pmiddle,1)));
                elseif size(Pmiddle,1) == 1
                    rpoints{end+1} = [Pmiddle(1); Pmiddle(2); Pmiddle(3)];
                end
                
            else
                % if we don't have reference ray traces to compare to, then
                % don't bother returning anything to the user about where
                % the objects are
                rmin = [];
                azmin = [];
                elmin = [];
                rpoints = {};
            end

            % update the bin count from this iteration's ray trace
            obj.bincount_prev = [n_az2; n_el2];
        end
    end
end

