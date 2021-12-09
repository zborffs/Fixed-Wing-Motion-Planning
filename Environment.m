classdef Environment < handle
    % the Environment class maintains the obstacle data
    
    properties
        obstacles % list of obstacles as represented by meshes in Matlab
    end
    
    methods
        function obj = Environment()
            % default constructor for Environment object that creates an
            % empty obstacles list
            % :return: an Environment object
            
            obj.obstacles = {};
        end
        
        function add_obstacle(obj, obstacle)
            % adds an obstacle to the obstacle list
            % :param obstacle: obstacle being added to the list
            
            obj.obstacles{end+1} = obstacle;
        end
    end
end

