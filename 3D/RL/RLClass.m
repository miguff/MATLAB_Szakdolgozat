classdef RLClass < handle
    properties
        LearningRate {mustBeNumeric}
        Gamma {mustBeNumeric}
        epsilon {mustBeNumeric}
        EndState
        Q_Table
    end
    methods
        function self = RLClass(LR, Gamma, Epsilon, EndState, Q_Table)
            if nargin == 5
                self.LearningRate = LR;
                self.Gamma = Gamma;
                self.epsilon = Epsilon;
                self.EndState = EndState;
                self.Q_Table = Q_Table;
 
            else
                self.LearningRate = 0.001;
                self.Gamma = 0.9;
                self.epsilon = 0.7;
                self.EndState = [3 3 3];
                self.Q_Table = [];
                Error = "Nem adtál meg minden adatot, az alapokat fogom használni. Mindenképpen adj meg egy Q_Table-t külön"
            end
        end
        function [next_state, reward, done, ExistingData, goal] = lepes(self, obsx, obsy, obsz, AgentState,ExistingData)
            r = rand(1);
            if r < self.epsilon
                SurroundingMatrix = MatrixExtraction(self, obsx, obsy, obsz);
    
                maxValue = max(SurroundingMatrix(:));
                [row, col, page] = ind2sub(size(SurroundingMatrix), find(SurroundingMatrix == maxValue));
                MaxValueIndexes = [row col page];
                while true
                    randomRowIndex = randi(size(MaxValueIndexes, 1));
                    randomRow = MaxValueIndexes(randomRowIndex, :);
                    localCenter = CenterData(self, AgentState);
                    TransformationData = randomRow -localCenter;
                    AgentState = AgentState + TransformationData;
                
                    if AgentState(1) <= size(self.Q_Table, 1) && AgentState(2) <= size(self.Q_Table, 2) && AgentState(3) <= size(self.Q_Table, 3)
                        break                      
                    end
                end
        
                if isequal(AgentState, self.EndState)
                    next_state = AgentState;
                    reward = 100;
                    done = 1;
                    goal = true;
                elseif self.Q_Table(AgentState(1), AgentState(2), AgentState(3)) == -100
                    next_state = AgentState;
                    reward = -100;
                    done = -1;
                    goal = false;
                else
                    next_state = AgentState;
                    reward = -0.2;
                    done = 0;
                    goal = false;
                end
            ExistingData = [ExistingData;AgentState];
            else    
                SurroundingMatrix = MatrixExtraction(self, obsx, obsy, obsz);
                AgentState = RandomStep(self, AgentState, SurroundingMatrix);
                if isequal(AgentState, self.EndState)
                    next_state = AgentState;
                    reward = 100;
                    done = 1;
                    goal = true;
                elseif self.Q_Table(AgentState(1), AgentState(2), AgentState(3)) == -100
                    next_state = AgentState;
                    reward = -100;
                    done = -1;
                    goal = false;
                else
                    next_state = AgentState;
                    reward = -0.2;
                    done = 0;
                    goal = false;
                end
            ExistingData = [ExistingData;AgentState];
            end
        end
        
        function surroundingmatrix = MatrixExtraction(self, obsx, obsy, obsz)
            matrix_size = size(self.Q_Table);
            matrix = self.Q_Table;
        
            % Beállítjuk, hogy melyik az éppen megfelelő érték ami köré
            % falat akarunk vonni
            random_row = obsx;
            random_col = obsy;
            random_depth = obsz;
        
            % Mennyi körülötte lévő mezőre vagyunk kíváncsiak
            surrounding_size = 1;
                
            % Calculate the range for extraction
            row_range = max(1, random_row - surrounding_size):min(matrix_size(1), random_row + surrounding_size);
            col_range = max(1, random_col - surrounding_size):min(matrix_size(2), random_col + surrounding_size);
            depth_range = max(1, random_depth - surrounding_size):min(matrix_size(3), random_depth + surrounding_size);
        
            
            surroundingmatrix = matrix(row_range, col_range, depth_range);
        
        end
        
        
        function localCenter = CenterData(self, AgentState)
             %Azok az értékek ahol a bal kezdő sarokban van
            if AgentState(1) == 1 && AgentState(3)==1 && AgentState(2) == 1
                localCenter = [1 1 1];
            elseif AgentState(1) == 1 && AgentState(2) == 1 && AgentState(3)==2
                localCenter = [1 1 2];
            elseif AgentState(1) == 1 && AgentState(2) == 1 && AgentState(3)==3
                localCenter = [1 1 2];
            elseif AgentState(1) == 1 && AgentState(2) == 1 && AgentState(3)==4
                localCenter = [1 1 2];
            elseif AgentState(1) == 1 && AgentState(2) == 1 && AgentState(3)==5
                localCenter = [1 1 2];
            elseif AgentState(1) == 1 && AgentState(2) == 1 && AgentState(3)==6
                localCenter = [1 1 2];
        
            %Ahol az értékek a jobb szélső sarokban vannak
            elseif AgentState(1) == 1 && AgentState(3)==50 && AgentState(2) == 1
                localCenter = [1 2 1];
            elseif AgentState(1) == 1 && AgentState(2) == 50 && AgentState(3)==2
                localCenter = [1 2 2];
            elseif AgentState(1) == 1 && AgentState(2) == 50 && AgentState(3)==3
                localCenter = [1 2 2];
            elseif AgentState(1) == 1 && AgentState(2) == 50 && AgentState(3)==4
                localCenter = [1 2 2];
            elseif AgentState(1) == 1 && AgentState(2) == 50 && AgentState(3)==5
                localCenter = [1 2 2];
            elseif AgentState(1) == 1 && AgentState(2) == 50 && AgentState(3)==6
                localCenter = [1 2 2];


             %Ahol az értékek a bal alsó sarokban vannak
            elseif AgentState(1) == 50 && AgentState(3)==1 && AgentState(2) == 1
                localCenter = [2 1 1];
            elseif AgentState(1) == 50 && AgentState(2) == 1 && AgentState(3)==2
                localCenter = [2 1 2];
            elseif AgentState(1) == 50 && AgentState(2) == 1 && AgentState(3)==3
                localCenter = [2 1 2];
            elseif AgentState(1) == 50 && AgentState(2) == 1 && AgentState(3)==4
                localCenter = [2 1 2];
            elseif AgentState(1) == 50 && AgentState(2) == 1 && AgentState(3)==5
                localCenter = [2 1 2];
            elseif AgentState(1) == 50 && AgentState(2) == 1 && AgentState(3)==6
                localCenter = [2 1 2];



            %Amik az első emeleten a falon vannak
            elseif AgentState(1) == 1 && AgentState(3)==1
                localCenter = [1 2 1];
            elseif AgentState(2) == 1 && AgentState(3)==1
                localCenter = [2 1 1];
       


            %Amik a második emeleten a falon vannak
            elseif AgentState(2) == 1 && AgentState(3)==2
                localCenter = [2 1 2];
            elseif AgentState(1) == 1 && AgentState(3)==2
                localCenter = [1 2 2];
        
            %Amik a haramik emeleten a falon vannak
            elseif AgentState(2) == 1 && AgentState(3)==3
                localCenter = [2 1 2];
            elseif AgentState(1) == 1 && AgentState(3)==3
                localCenter = [1 2 2];
            
            %Amikor a 4. emeleti falon van
            elseif AgentState(1) == 1 && AgentState(3)==4
                localCenter = [1 2 2];
            elseif AgentState(2) == 1 && AgentState(3)==4
                localCenter = [2 1 2];
            
                
                %Amikor az 5. emeleti falon van
            elseif AgentState(1) == 1 && AgentState(3)==5
                localCenter = [1 2 2];
            elseif AgentState(2) == 1 && AgentState(3)==5
                localCenter = [2 1 2];

            %Amikor az 6. emeleti falon van
            elseif AgentState(1) == 1 && AgentState(3)==6
                localCenter = [1 2 2];
            elseif AgentState(2) == 1 && AgentState(3)==6
                localCenter = [2 1 2];

            %Amik nincsenek falon - Itt arra kell tenni az elsőnél ami a
            %max a max magassága a rendszernek
            elseif AgentState(3) == 1 
                localCenter = [2 2 1];
            else
                localCenter = [2 2 2];
            end
        end
        function AgentStateRandom = RandomStep(self, AgentState, SurroundingMatrix)
            rows = size(SurroundingMatrix, 1);
            cols = size(SurroundingMatrix, 2);
            depth = size(SurroundingMatrix, 3);
            given_value = [AgentState(1) AgentState(2) AgentState(3)];
            while true
                % Generate random indices
                random_row = randi(rows);
                random_col = randi(cols);
                random_depth = randi(depth);
        
                % Check if the randomly chosen index is not equal to the given value
                if ~isequal([random_row, random_col, random_depth], given_value)
                    % Display the randomly chosen cell index
                    randomRow = [random_row random_col random_depth];
                    localCenter = CenterData(self, AgentState);
                    TransformationData = randomRow -localCenter;
                    AgentStateRandom = AgentState + TransformationData;
                    if AgentStateRandom(1) <= size(self.Q_Table, 1) && AgentStateRandom(2) <= size(self.Q_Table, 2) && AgentStateRandom(3) <= size(self.Q_Table, 3)
                        break                      
                    end
                    
                end
            end
        end
    function [cost, Q_Table] = learn(self, obsx, obsy, obsz, reward, next_state)
        q_predict = self.Q_Table(obsx, obsy, obsz);
        q_target = reward + self.Gamma*self.Q_Table(next_state(1), next_state(2), next_state(3));
        self.Q_Table(obsx, obsy, obsz) = self.Q_Table(obsx, obsy, obsz) + self.LearningRate*(q_target-q_predict);
        cost = self.Q_Table(obsx, obsy, obsz);
        Q_Table = self.Q_Table;
    end

    end

end