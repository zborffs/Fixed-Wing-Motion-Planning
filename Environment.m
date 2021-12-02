classdef Environment < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        obstacles % list of Obstacles        
    end
    
    methods
        function obj = Environment()
            obj.obstacles = {};
        end
        
        function add_obstacle(obj, obstacle)
            obj.obstacles{end+1} = obstacle;
        end
    end
end

