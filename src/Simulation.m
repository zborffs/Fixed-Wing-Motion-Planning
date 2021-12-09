classdef Simulation < handle
    % Simulation class holds on to all the data required to perform a
    % simulation. Since simulations require several non-trivial objects
    % such as Aircraft, Environment, and AircraftController objects to
    % properly run, it makes sense to create an object that handles the
    % logic responsible for instantiating each of those uniformally. This
    % class also computes and holds onto important metrics related to
    % different simulations.
    
    properties
        env % Environment object (holds obstacle data)
        aircraft % Aircraft object (holds state data)
        controller % AircraftControl object (holds path-planning / control algorithms and data
        
        % Target region data
        x_des % target region's centroid's x-position in meters 
        y_des % target region's centroid's y-position in meters 
        z_des % target region's centroid's z-position in meters 
        abs_tol % target region's length, width, and height

        % Simulation output
        t % time vector
        out % the state variables at each time instant
        
        % S-Taliro output
        staliro_results % the results from an staliro simulation
    end
    
    methods
        function obj = Simulation(env, aircraft, controller, target_coordinates, abs_tol)
            % Constructs a Simulation object from user-defined objects
            % :param env: the environment (holds obstacle information)
            % :param aircraft: the aircraft (holds state information)
            % :param controller: contains motion-planning logic
            % :param target_coordinates: the target coordinates
            % :param abs_tol: the absolute tolerance of entering the target
            % coordinate
            % :return: an instance of Simulation object
            
            % initialize all objects with user-defined objects
            obj.env = env;
            obj.aircraft = aircraft;
            obj.controller = controller;
            obj.x_des = target_coordinates(1); 
            obj.y_des = target_coordinates(2); 
            obj.z_des = target_coordinates(3);
            obj.abs_tol = abs_tol;

            % we haven't performed a simulation yet, so these member
            % variables should be the empty-set
            obj.t = [];
            obj.out = [];
            obj.staliro_results = [];
        end
        
        function simulate(obj)
            % simulate simulates the aircraft object as its controlled by 
            % the controller object through the environment object
            % :param obj: the Simulation object that called this function

            
            % create a struct to hold the object information
            p = struct('x_des', obj.x_des, 'y_des', obj.y_des, 'z_des', obj.z_des, ... 
                        'abs_tol', obj.abs_tol, 'aircraft', obj.aircraft, ...
                        'controller', obj.controller, 'environment', obj.env);

            % create handler to make interface of dubins3d function match
            % that expected by the ode45 solver
            model = @(t, state) dubins3d(t, state, p);
            
            % add a 'reached_solution_termination' condition on the ODE
            % solver. This way, if our aircraft made it to the desired
            % location, then we exit the simulation
            rst =  @(t,state) reached_solution_termination(t, state, p);
            
            ode_options = odeset('Events', rst, 'AbsTol', 1e-3);

            % Run ode45 over the model for 100 seconds starting at the
            % aircraft's start position
            [t, out] = ode45(model,[0 50],[obj.aircraft.x; obj.aircraft.y; obj.aircraft.z; obj.aircraft.theta; obj.aircraft.alpha], ode_options);
            
            % store the outcome of the simulation
            obj.t = t;
            obj.out = out;
        end

        function animate(obj)
            % not really done yet.
            
            t = obj.t;
            x = obj.out(:,1);
            y = obj.out(:,2);
            z = obj.out(:,3);
            theta = obj.out(:,4);
            alpha = obj.out(:,5);

            assert(all(size(x) == size(y)));
            assert(all(size(x) == size(z)));
            assert(all(size(x) == size(theta)));
            assert(all(size(x) == size(alpha)));
            h = figure;
            axis tight % this ensures that getframe() returns a consistent size

            t_step_init = 0.1;
            t_step = t_step_init;
            i = 1;
            v = [];
            while i < size(t, 1) - 1
                t_step = t_step - (t(i+1) - t(i));
                if t_step < 0
                    v = [v; i];
                    t_step = t_step_init;
                end
                i = i + 1;
            end
            
            filename = 'animation.gif';
            for i = 1:length(v)
                obj.aircraft.x = x(v(i));
                obj.aircraft.y = y(v(i));
                obj.aircraft.z = z(v(i));
                obj.aircraft.theta = theta(v(i));
                obj.aircraft.alpha = alpha(v(i));

                [X, Y, Z, TF] = detectable_objects(obj.aircraft, obj.env);
                
                target_box = create_box(3, 2*obj.abs_tol, 2*obj.abs_tol, 2*obj.abs_tol);
                target_box = translate_shape(target_box, [obj.x_des-obj.abs_tol; obj.y_des-obj.abs_tol; obj.z_des-obj.abs_tol]);
                draw(obj.aircraft, obj.env); hold on;
                plot(target_box, 'FaceColor', 'green', 'FaceAlpha', 0.1);
                view([-11.6 26.6]);
                plt = plot3(x,y,z); hold off; title("Time: " + t(v(i)));

                drawnow
                frame = getframe(h);
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256); 
                % Write to the GIF File 
                if i == 1 
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', t_step_init);
                else 
                    imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', t_step_init);
                end 
            end
        end

        function plot(obj)
            % plots the simulation state variables over time and draws the
            % 3D trajectory of the aircraft through the environment
            
            % grab the state variables from the member variables
            x = obj.out(:,1);
            y = obj.out(:,2);
            z = obj.out(:,3);
            theta = obj.out(:,4);
            alpha = obj.out(:,5);
            
            % plot each of the state variables over time
            figure;
            subplot(5,1,1);
            plot(obj.t,x); grid on;
            subplot(5,1,2);
            plot(obj.t,y); grid on;
            subplot(5,1,3);
            plot(obj.t,z); grid on;
            subplot(5,1,4);
            plot(obj.t,theta); grid on;
            subplot(5,1,5);
            plot(obj.t,alpha); grid on;

            % draw the environment, the aircraft object, and the 3D
            % trajectory through the environment
            figure;
            target_box = create_box(3, 2*obj.abs_tol, 2*obj.abs_tol, 2*obj.abs_tol);
            target_box = translate_shape(target_box, [obj.x_des-obj.abs_tol; obj.y_des-obj.abs_tol; obj.z_des-obj.abs_tol]);
            draw(obj.aircraft, obj.env); hold on;
            plot3(x,y,z); 
            plot(target_box, 'FaceColor', 'green', 'FaceAlpha', 0.1);
            hold off;
        end

        function falsify(obj)
            % create a struct to hold the object information
            p = struct('x_des', obj.x_des, 'y_des', obj.y_des, 'z_des', obj.z_des, ... 
                        'abs_tol', obj.abs_tol, 'aircraft', obj.aircraft, ...
                        'controller', obj.controller, 'environment', obj.env);

            % create handler to make interface of dubins3d function match
            % that expected by the ode45 solver
            model = @(t, state) dubins3d(t, state, p);
            
            tspan = 50;
            X0 = [
                obj.aircraft.x - 5 obj.aircraft.x + 5;
                obj.aircraft.y - 5 obj.aircraft.y + 5;
                obj.aircraft.z - 5 obj.aircraft.z + 5;
                obj.aircraft.theta - deg2rad(180) obj.aircraft.theta + deg2rad(180);
                obj.aircraft.alpha - deg2rad(180) obj.aircraft.alpha + deg2rad(180)];
            cp_array = [];
            input_range = [];
            
            phi = '<>a';
            pred(1).str = 'a';
            pred(1).A = eye(3,5);
            pred(1).b = [obj.x_des + obj.abs_tol; obj.y_des + obj.abs_tol; obj.z_des + obj.abs_tol];
            opt = staliro_options();
            opt.runs = 1;
            
            obj.staliro_results = staliro(model,X0,input_range,cp_array,phi,pred,tspan,opt);
        end
    end
end

