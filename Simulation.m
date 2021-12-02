classdef Simulation < handle
    %UNTITLED18 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % 
        env
        aircraft
        controller
        
        % target region
        x_des
        y_des
        z_des
        abs_tol

        % output of sim
        t
        out
    end
    
    methods
        function obj = Simulation(env, aircraft, controller, des, abs_tol)
            obj.env = env;
            obj.aircraft = aircraft;
            obj.controller = controller;
            obj.x_des = des(1); obj.y_des = des(2); obj.z_des = des(3);
            obj.abs_tol = abs_tol;

            obj.t = [];
            obj.out = [];
        end
        
        function simulate(obj)
            p = struct('x_des', obj.x_des, 'y_des', obj.y_des, 'z_des', obj.z_des, 'abs_tol', obj.abs_tol, 'aircraft', obj.aircraft, 'controller', obj.controller, 'environment', obj.env);
            model = @(t, state) dubins3d(t, state, p);
            rst =  @(t,state) reached_solution_termination(t, state, p);
            ode_options = odeset('Events', rst);

            [t, out] = ode45(model,[0 3],[0; 0; 30; deg2rad(10); deg2rad(1)], ode_options);
            obj.t = t;
            obj.out = out;
        end

        function animate(obj)
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
            ylim([y(1) y(end)]);
            filename = 'animation.gif';
            for i = 1:length(x)
                obj.aircraft.x = x(i);
                obj.aircraft.y = y(i);
                obj.aircraft.z = z(i);
                obj.aircraft.theta = theta(i);
                obj.aircraft.alpha = alpha(i);

                [X, Y, Z, TF] = detectable_objects(obj.aircraft, obj.env);

                plt = plot_rays(X, Y, Z, TF); title("t = " + obj.t(i));
                drawnow
                frame = getframe(h); 
                im = frame2im(frame); 
                [imind,cm] = rgb2ind(im,256); 
                % Write to the GIF File 
                if i == 1 
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
                else 
                    imwrite(imind,cm,filename,'gif','WriteMode','append'); 
                end 
            end
        end

        function plot(obj)
            x = obj.out(:,1);
            y = obj.out(:,2);
            z = obj.out(:,3);
            theta = obj.out(:,4);
            alpha = obj.out(:,5);
            
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

            figure;
            draw(obj.aircraft, obj.env); hold on; plot3(x,y,z); hold off; 
        end
    end
end

