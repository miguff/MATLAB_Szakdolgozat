classdef RLEnv < handle
    properties
        Q_Table
        MapTable
        
    end
    methods
        function obj = RLEnv(row, col, depth)
            if nargin == 3
                obj.Q_Table = zeros(row, col, depth);
                obj.MapTable = obj.Q_Table;
            else
                obj.Q_Table = zeros(3, 3, 3);
                obj.MapTable = obj.Q_Table;
                
            end
        end

        function addObstacle = RLAddObstacle(obj,row1, row2, col1, col2,  depth1, depth2, MapOccupancy)
            obj.Q_Table(row1:row2, col1:col2, depth1:depth2) = -Inf;
            obj.MapTable(row1:row2, col1:col2, depth1:depth2) = 1;
            [xobstacle, yobstacle, zobstacle] = meshgrid(row1:row2, col1:col2, depth1:depth2);
            xyzObs = [xobstacle(:) yobstacle(:) zobstacle(:)];
            setOccupancy(MapOccupancy, xyzObs, 1)
        end

        function addEndState = RLAddEndSate(obj,row, col, depth, Value)
            obj.Q_Table(row, col, depth) = Value;
        end
    end


end


