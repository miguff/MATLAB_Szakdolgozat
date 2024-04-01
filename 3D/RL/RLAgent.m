classdef RLAgent < RLEnv
    properties
        InitRow
        InitCol
        InitDepth
        AgentState
    end
    methods
        function self = RLAgent(InitRow, InitCol, InitDepth)
            self.InitRow = InitRow;
            self.InitCol = InitCol;
            self.InitDepth = InitDepth;
            self.AgentState = [self.InitRow self.InitCol self.InitDepth];
        end
        
        function AgentState = RLReset(self)
            self.AgentState = [self.InitRow self.InitCol self.InitDepth];
        end


    end
end
