classdef AircraftController < handle
    properties
        % theta->v controller stuff
        theta_variables
        
        % alpha->w controller stuff
        alpha_variables

        t_prev

        % reactive control
        
        % bincount
        bincount_prev
        
        % prev minimum distances
        rmin_prev
    end
    
    methods
        function obj = AircraftController()
            obj.theta_variables = [0; 0; 0];

            obj.alpha_variables = [0; 0; 0];
            
            obj.t_prev = [];

            obj.bincount_prev = [];
            obj.rmin_prev = [];
        end
        
        function [u,v,w] = global_control(obj, t, aircraft, env, x_des, y_des, z_des)
            u = 5;

            [rmin, thetamin, alphamin] = obj.find_closest_danger(t, aircraft, env);
                

            if ~isempty(obj.t_prev)
                theta_des = [];

                if ~isempty(rmin)
                    T = t - obj.t_prev;
                    rmin_dot = (rmin - obj.rmin_prev) / T;

                    disp("t : " + t + ", rmin : " + rmin + ", rmin_dot : " + rmin_dot + ", thetamin : " + rad2deg(thetamin))
                    
                    if rmin < 8
                        % take evasive action!
                        theta_hat = angdiff(thetamin, aircraft.theta);
                        
                        if theta_hat < 0
                            % implies aircraft.theta is further
                            % clockwise => turn right
                            theta_des = wrapToPi(thetamin - pi/2);
                        else
                            % implies aircraft.theta is further
                            % counter-clockwise => turn left
                            theta_des = wrapToPi(thetamin + pi/2);
                        end

                    end
                end


                if isempty(theta_des)
                    theta_des = atan2(y_des - aircraft.y, x_des - aircraft.x);
                end

                alpha_des = atan2(z_des - aircraft.z, sqrt((x_des - aircraft.x)^2 + (y_des - aircraft.y)^2));

                theta_error = theta_des - aircraft.theta;
                alpha_error = alpha_des - aircraft.alpha;

                T = t - obj.t_prev;

                A = 2 * (1 + T);
                B = 2 * (-1 + T);
                C = 2 + T;
                D = (-2 + T);

                obj.theta_variables(2) = obj.theta_variables(1);
                obj.theta_variables(1) = theta_error;

                obj.alpha_variables(2) = obj.alpha_variables(1);
                obj.alpha_variables(1) = alpha_error;


                v = (A * obj.theta_variables(1) + B * obj.theta_variables(2) - D * obj.theta_variables(3)) / C;
                w = (A * obj.alpha_variables(1) + B * obj.alpha_variables(2) - D * obj.alpha_variables(3)) / C;

                obj.theta_variables(3) = v;
                obj.alpha_variables(3) = w;
            else
                obj.t_prev = t;
                v = 0;
                w = 0;
            end

            obj.rmin_prev = rmin;
        end

        function [rmin, azmin, elmin] = find_closest_danger(obj, t, aircraft, env)
            [X,Y,Z,TF] = detectable_objects(aircraft, env);
            % get all the intersecting points
            in = find(TF);
            Xin = X(in);
            Yin = Y(in);
            Zin = Z(in);

            % convert cartesian to spherical and place in single matrix
            [az,el,r] = cart2sph(Xin-aircraft.x, Yin-aircraft.y, Zin-aircraft.z);
            P2 = [az el r];

            % break down each coordinate down into frequency
            [n_az2, bins_az2] = histcounts(P2(:,1), 31);
            [n_el2, bins_el2] = histcounts(P2(:,2), 31);

            if ~isempty(obj.bincount_prev)

                n_az1 = obj.bincount_prev(1,:);
                n_el1 = obj.bincount_prev(2,:);
                
                % get the sign of each component
                sgn_az = sign(n_az2-n_az1);
                sgn_el = sign(n_el2-n_el1);

                % only get positively signed components
                indices_az = sgn_az > 0;
                indices_el = sgn_el > 0;

                % find angle ranges where these occured (az)
                indices_az2 = false(1,length(indices_az)+1);
                for i = 1:length(indices_az)
                    if indices_az(i)
                        indices_az2(i:i+1) = true;
                    end
                end
                angles_az = bins_az2(indices_az2);

                % find angle ranges where these occured (el)
                indices_el2 = false(1,length(indices_el)+1);
                for i = 1:length(indices_el)
                    if indices_el(i)
                        indices_el2(i:i+1) = true;
                    end
                end
                angles_el = bins_el2(indices_el2);

%                 middle_indexes = [];
                rmin = [];
                elmin = [];
                azmin = [];

                assert(all(size(el) == size(az)));
                for i = 1:length(az)
                    for j = 1:length(angles_el)-1
                        for k = 1:length(angles_az)-1
                            if az(i) >= angles_az(k) && az(i) <= angles_az(k+1) && el(i) >= angles_el(j) && el(i) <= angles_el(j+1)
%                                 middle_indexes(end+1) = i;

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

%                 middle_indexes = middle_indexes';
%                 Pmiddle = [r(middle_indexes) az(middle_indexes) el(middle_indexes)];
% 
%                 bpoints = [0];
% 
%                 if length(Pmiddle) > 1
%                     for i = 2:length(Pmiddle)
%                         if abs(Pmiddle(i,2) - Pmiddle(i-1,2)) > 0.000001 || abs(Pmiddle(i,3) - Pmiddle(i-1,3)) > 0.000001
%                             bpoints(end+1) = i-1;
%                         end
%                     end
%                 end
%                 bpoints(end+1) = length(Pmiddle);
% 
%                 rpoints = {};
%                 for i = 1:length(bpoints)-1
%                     rpoints{end+1} = Pmiddle(bpoints(i)+1:bpoints(i+1));
%                 end
%                 assert(all(length([rpoints{:}]) == length(Pmiddle)));
            else
                rmin = [];
                thetamin = [];
                alphamin = [];
            end

            obj.bincount_prev = [n_az2; n_el2];
        end

    end
end

