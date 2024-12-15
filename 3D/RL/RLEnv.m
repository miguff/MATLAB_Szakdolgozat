classdef RLEnv < handle
    properties
        Q_Table
        MapTable
        
    end
    methods
        function self = RLEnv(row_Map, col, depth)
            if nargin == 3
                self.Q_Table = zeros(row_Map, col, depth);
                self.MapTable = self.Q_Table;
            elseif nargin == 1
                self.MapTable = row_Map;
                self.Q_Table = row_Map;
                self.Q_Table(self.Q_Table==1)=-100;
            else
                self.Q_Table = zeros(3, 3, 3);
                self.MapTable = self.Q_Table;
                
            end
        end

        function addObstacle = RLAddObstacle(self,row1, row2, col1, col2,  depth1, depth2, MapOccupancy)
            self.Q_Table(row1:row2, col1:col2, depth1:depth2) = -100;
            self.MapTable(row1:row2, col1:col2, depth1:depth2) = 1;
            [xobstacle, yobstacle, zobstacle] = meshgrid(row1:row2, col1:col2, depth1:depth2);
            xyzObs = [xobstacle(:) yobstacle(:) zobstacle(:)];
            setOccupancy(MapOccupancy, xyzObs, 1)
        end

        function addMap = RLAddExistingObstacle(self,ExistingOccupancyMapMatrix)
            self.MapTable = ExistingOccupancyMapMatrix;
            self.Q_Table = ExistingOccupancyMapMatrix;
            self.Q_Table(self.Q_Table==1)=-100;
        end

        function addEndState = RLAddEndSate(obj,row, col, depth, Value)
            obj.Q_Table(row, col, depth) = Value;
        end

        function ManhattanDevice = RLAddManhatten(self, endCoordinates)
            for i=1:size(self.Q_Table,1)
                for j=1:size(self.Q_Table,2)
                    for k=1:size(self.Q_Table,3)
                        cell = [j, i ,k];
                        if isequal(cell, endCoordinates) || self.Q_Table(j, i, k) == -100
                            continue
                        else
                            self.Q_Table(j, i, k) = self.Q_Table(j,i,k)+reciprocal_euclidean_distance_3d(self,cell, endCoordinates);
                        end
                    end
                end
            end
        end
        
        function reciprocal_distance = reciprocal_euclidean_distance_3d(self, cell1, cell2)
            %Kiszámoljuk a reciprokát egy 2 3 elemű sor vektor manhattan
            %távolságának a reciprokát
            dx = cell2(1) - cell1(1);
            dy = cell2(2) - cell1(2);
            dz = cell2(3) - cell1(3);

            euclidean_distance = sqrt(dx^2 + dy^2 + dz^2);
    
            reciprocal_distance = 1 / euclidean_distance;
        end


    end


end


