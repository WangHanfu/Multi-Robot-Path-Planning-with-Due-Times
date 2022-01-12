classdef MILPModel<handle
    
    properties
        AineqI
        AineqJ
        AineqV
        bineq
        
        AeqI
        AeqJ
        AeqV
        beq
        
        f
        
        AineqSize
        AeqSize
        AWidth
    end
    
    methods
        function obj = MILPModel(network,robotNum,objectiveSelect)
            switch objectiveSelect
                case 1 %makespan
                    constraint1 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint2 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint4 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint3.AI;
                    AineqJ = constraint3.AJ;
                    AineqV = constraint3.AV;
                    bineq = constraint3.b;
                    AineqSize = AineqSize + constraint3.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint4.AI];
                    AineqJ = [AineqJ; constraint4.AJ];
                    AineqV = [AineqV; constraint4.AV];
                    bineq = [bineq; constraint4.b];
                    AineqSize = AineqSize + constraint4.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint1.AI;
                    AeqJ = constraint1.AJ;
                    AeqV = constraint1.AV;
                    beq = constraint1.b;
                    AeqSize = AeqSize + constraint1.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint2.AI];
                    AeqJ = [AeqJ; constraint2.AJ];
                    AeqV = [AeqV; constraint2.AV];
                    beq = [beq; constraint2.b];
                    AeqSize = AeqSize + constraint2.Num;
                    
                    f = obj.getObjectiveMakespan(network,robotNum);
                    AWidth = network.ArcNum;
                    
                case 2 %total arrival time
                    constraint1 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint2 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint4 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint3.AI;
                    AineqJ = constraint3.AJ;
                    AineqV = constraint3.AV;
                    bineq = constraint3.b;
                    AineqSize = AineqSize + constraint3.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint4.AI];
                    AineqJ = [AineqJ; constraint4.AJ];
                    AineqV = [AineqV; constraint4.AV];
                    bineq = [bineq; constraint4.b];
                    AineqSize = AineqSize + constraint4.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint5.AI];
                    AineqJ = [AineqJ; constraint5.AJ];
                    AineqV = [AineqV; constraint5.AV];
                    bineq = [bineq; constraint5.b];
                    AineqSize = AineqSize + constraint5.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint1.AI;
                    AeqJ = constraint1.AJ;
                    AeqV = constraint1.AV;
                    beq = constraint1.b;
                    AeqSize = AeqSize + constraint1.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint2.AI];
                    AeqJ = [AeqJ; constraint2.AJ];
                    AeqV = [AeqV; constraint2.AV];
                    beq = [beq; constraint2.b];
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint6.AI];
                    AeqJ = [AeqJ; constraint6.AJ];
                    AeqV = [AeqV; constraint6.AV];
                    beq = [beq; constraint6.b];
                    AeqSize = AeqSize + constraint6.Num;
                    
                    f = obj.getObjectiveTotalTime(network,robotNum);
                    AWidth = network.ArcNum + network.T*robotNum;
                    
                case 5 %maximum lateness
                    constraint1 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint2 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint4 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    constraint7 = obj.getConstraintLateness(network,robotNum); %inequality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint3.AI;
                    AineqJ = constraint3.AJ;
                    AineqV = constraint3.AV;
                    bineq = constraint3.b;
                    AineqSize = AineqSize + constraint3.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint4.AI];
                    AineqJ = [AineqJ; constraint4.AJ];
                    AineqV = [AineqV; constraint4.AV];
                    bineq = [bineq; constraint4.b];
                    AineqSize = AineqSize + constraint4.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint5.AI];
                    AineqJ = [AineqJ; constraint5.AJ];
                    AineqV = [AineqV; constraint5.AV];
                    bineq = [bineq; constraint5.b];
                    AineqSize = AineqSize + constraint5.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint7.AI];
                    AineqJ = [AineqJ; constraint7.AJ];
                    AineqV = [AineqV; constraint7.AV];
                    bineq = [bineq; constraint7.b];
                    AineqSize = AineqSize + constraint7.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint1.AI;
                    AeqJ = constraint1.AJ;
                    AeqV = constraint1.AV;
                    beq = constraint1.b;
                    AeqSize = AeqSize + constraint1.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint2.AI];
                    AeqJ = [AeqJ; constraint2.AJ];
                    AeqV = [AeqV; constraint2.AV];
                    beq = [beq; constraint2.b];
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint6.AI];
                    AeqJ = [AeqJ; constraint6.AJ];
                    AeqV = [AeqV; constraint6.AV];
                    beq = [beq; constraint6.b];
                    AeqSize = AeqSize + constraint6.Num;
                    
                    f = obj.getObjectiveMaxLateness(network,robotNum);
                    AWidth = network.ArcNum + network.T*robotNum+1;
                    
                case 6 %total tardiness
                    constraint1 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint2 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint4 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint3.AI;
                    AineqJ = constraint3.AJ;
                    AineqV = constraint3.AV;
                    bineq = constraint3.b;
                    AineqSize = AineqSize + constraint3.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint4.AI];
                    AineqJ = [AineqJ; constraint4.AJ];
                    AineqV = [AineqV; constraint4.AV];
                    bineq = [bineq; constraint4.b];
                    AineqSize = AineqSize + constraint4.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint5.AI];
                    AineqJ = [AineqJ; constraint5.AJ];
                    AineqV = [AineqV; constraint5.AV];
                    bineq = [bineq; constraint5.b];
                    AineqSize = AineqSize + constraint5.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint1.AI;
                    AeqJ = constraint1.AJ;
                    AeqV = constraint1.AV;
                    beq = constraint1.b;
                    AeqSize = AeqSize + constraint1.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint2.AI];
                    AeqJ = [AeqJ; constraint2.AJ];
                    AeqV = [AeqV; constraint2.AV];
                    beq = [beq; constraint2.b];
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint6.AI];
                    AeqJ = [AeqJ; constraint6.AJ];
                    AeqV = [AeqV; constraint6.AV];
                    beq = [beq; constraint6.b];
                    AeqSize = AeqSize + constraint6.Num;
                    
                    f = obj.getObjectiveTotalTardiness(network,robotNum);
                    AWidth = network.ArcNum + network.T*robotNum;
                    
                case 7 %total unit penalties
                    constraint1 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint2 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint4 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint3.AI;
                    AineqJ = constraint3.AJ;
                    AineqV = constraint3.AV;
                    bineq = constraint3.b;
                    AineqSize = AineqSize + constraint3.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint4.AI];
                    AineqJ = [AineqJ; constraint4.AJ];
                    AineqV = [AineqV; constraint4.AV];
                    bineq = [bineq; constraint4.b];
                    AineqSize = AineqSize + constraint4.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint1.AI;
                    AeqJ = constraint1.AJ;
                    AeqV = constraint1.AV;
                    beq = constraint1.b;
                    AeqSize = AeqSize + constraint1.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint2.AI];
                    AeqJ = [AeqJ; constraint2.AJ];
                    AeqV = [AeqV; constraint2.AV];
                    beq = [beq; constraint2.b];
                    AeqSize = AeqSize + constraint2.Num;
                    
                    f = obj.getObjectiveTotalUP(network,robotNum);
                    AWidth = network.ArcNum;
            end
            
            obj.AineqI = AineqI;
            obj.AineqJ = AineqJ;
            obj.AineqV = AineqV;
            obj.bineq = bineq;
            
            obj.AeqI = AeqI;
            obj.AeqJ = AeqJ;
            obj.AeqV = AeqV;
            obj.beq = beq;
            
            obj.f = f;
            
            obj.AineqSize = AineqSize;
            obj.AeqSize = AeqSize;
            obj.AWidth = AWidth;
        end
        
        %% flow conservation constraints at all non-terminal nodes, equality, NodeNum*robotNum
        function constraint = getConstraintFlowConservationAtNonTerminalNodes(obj,network,robotNum)
            constNum = network.NodeNum-2*robotNum;
            AI = zeros(constNum*10,1);
            AJ = zeros(constNum*10,1);
            AV = zeros(constNum*10,1);
            b = zeros(constNum,1);
            
            nonTerminalIDs = 1:network.NodeNum;
            for i = [network.SourceVec; network.SinkVec]'
                nonTerminalIDs(nonTerminalIDs==i) = [];
            end
            
            vecSize = 0;
            constSize = 0;
            
            for i = nonTerminalIDs
                node = network.Nodes(i,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                outArcIDs = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                if ~isempty(inArcIDs) && ~isempty(outArcIDs)
                    constSize = constSize + 1;
                    for index = inArcIDs
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = index;
                        AV(vecSize,1) = 1;
                    end
                    
                    for index = outArcIDs
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = index;
                        AV(vecSize,1) = -1;
                    end
                elseif ~isempty(inArcIDs) && isempty(outArcIDs)
                    constSize = constSize + 1;
                    for index = inArcIDs
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = index;
                        AV(vecSize,1) = 1;
                    end
                    
                elseif isempty(inArcIDs) && ~isempty(outArcIDs)
                    constSize = constSize + 1;
                    for index = outArcIDs
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = index;
                        AV(vecSize,1) = 1;
                    end
                end
            end
            
            AI(vecSize + 1:end,:) = [];
            AJ(vecSize + 1:end,:) = [];
            AV(vecSize + 1:end,:) = [];
            b(constSize + 1:end,:) = [];
            
            constraint.Num = constSize;
            constraint.Type = 1;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
        end
        
        %% arc flow conservation constraints at terminal nodes,equality,2*robotNum
        function constraint = getConstraintFlowConservationAtTerminalNodes(obj,network,robotNum)
            AI = zeros(2*robotNum*5,1);
            AJ = zeros(2*robotNum*5,1);
            AV = zeros(2*robotNum*5,1);
            
            vecSize = 0;
            for i=1:robotNum
                nodeID = network.SourceVec(i,1);
                node = network.Nodes(nodeID,:);
                outArcIDs = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                for index = outArcIDs
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = i;
                    AJ(vecSize,1) = index;
                    AV(vecSize,1) = 1;
                end
            end
            
            for i = 1:robotNum
                nodeID = network.SinkVec(i,1);
                node = network.Nodes(nodeID,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                for index = inArcIDs
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = robotNum + i;
                    AJ(vecSize,1) = index;
                    AV(vecSize,1) = 1;
                end
            end
            
            AI(vecSize + 1:end,:) = [];
            AJ(vecSize + 1:end,:) = [];
            AV(vecSize + 1:end,:) = [];
            
            constraint.Num = 2*robotNum;
            constraint.Type = 1;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = ones(2*robotNum,1);
        end
        
        %% vertex conflict constraints,inequality,at most NodeNum
        function constraint = getConstraintVertexConflict(obj,network,robotNum)
            AI = zeros(network.NodeNum*5,1);
            AJ = zeros(network.NodeNum*5,1);
            AV = ones(network.NodeNum*5,1);
            b = ones(network.NodeNum,1);
            
            vecSize = 0;
            constSize = 0;
            for i=1:network.NodeNum
                node = network.Nodes(i,:);
                inArcIndices = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                if ~isempty(inArcIndices)
                    constSize = constSize + 1;
                    for index = inArcIndices
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = index;
                        AV(vecSize,1) = 1;
                    end
                end
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            b(constSize +1:end,:) = [];
            
            constraint.Num = constSize;
            constraint.Type = 0;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
        end
        
        %% edge conflict constraints. inequality,conflictNum
        function constraint = getConstraintEdgeConflict(obj,network,robotNum)
            
            AI = zeros(network.ConflictNum*2,1);
            AJ = zeros(network.ConflictNum*2,1);
            AV = ones(network.ConflictNum*2,1);
            b = ones(network.ConflictNum,1);
            
            vecSize = 0;
            for i=1:network.ConflictNum
                arcIndex1 = network.ConflictArcPair(i,1);
                arcIndex2 = network.ConflictArcPair(i,2);
                
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = arcIndex1;
                AV(vecSize,1) = 1;
                
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = arcIndex2;
                AV(vecSize,1) = 1;
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            
            constraint.Num = network.ConflictNum;
            constraint.Type = 0;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
        end
        
        %% ineualities. totally (T-1)*robotNum
        function constraint = getConstraintLogicOperation(obj,network,robotNum)
            
            AI = zeros((network.T-1)*robotNum*10,1);
            AJ = zeros((network.T-1)*robotNum*10,1);
            AV = zeros((network.T-1)*robotNum*10,1);
            b = zeros((network.T-1)*robotNum,1);
            
            vecSize = 0;
            constSize = 0;
            for t=2:network.T-1
                for i=1:robotNum
                    % find arcIndex
                    nodeID = network.Structure(network.GoalRCT(i,1),network.GoalRCT(i,2),t);
                    outArcIndices = network.OutArcCell{network.GoalRCT(i,1),network.GoalRCT(i,2),t-1};
                    
                    if ~isempty(outArcIndices)
                        for j = outArcIndices
                            if network.Arcs(j,2) == nodeID
                                arcIndex = j;
                                break;
                            end
                        end
                    end
                    
                    % first
                    constSize = constSize + 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+t*robotNum+i;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 0;
                    
                    %second
                    constSize = constSize + 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = arcIndex;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 0;
                    
                    %third
                    constSize = constSize + 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = -1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+t*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = arcIndex;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 1;
                end
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            b(constSize +1:end,:) = [];
            
            constraint.Num = constSize;
            constraint.Type = 0;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
            
        end
        
        %% equality, y_i^T is x_i^T
        function constraint = getConstraintLogicOperationEqual(obj,network,robotNum)
            AI = zeros(robotNum*2,1);
            AJ = zeros(robotNum*2,1);
            AV = ones(robotNum*2,1);
            b = zeros(robotNum,1);
            
            vecSize = 0;
            for i=1:robotNum
                % find arcIndex
                nodeID = network.Structure(network.GoalRCT(i,1),network.GoalRCT(i,2),network.T);
                outArcIndices = network.OutArcCell{network.GoalRCT(i,1),network.GoalRCT(i,2),network.T-1};
                
                if ~isempty(outArcIndices)
                    for j = outArcIndices
                        if network.Arcs(j,2) == nodeID
                            arcIndex = j;
                            break;
                        end
                    end
                end
                
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = network.ArcNum+(network.T-1)*robotNum+i;
                AV(vecSize,1) = 1;
                
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = arcIndex;
                AV(vecSize,1) = -1;
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            
            constraint.Num = robotNum;
            constraint.Type = 1;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
        end
        
        %% ineualities. totally (T-1)*robotNum
        function constraint = getConstraintStay(obj,network,robotNum)
            
            AI = zeros((network.T-1)*robotNum*10,1);
            AJ = zeros((network.T-1)*robotNum*10,1);
            AV = zeros((network.T-1)*robotNum*10,1);
            b = zeros((network.T-1)*robotNum,1);
            
            vecSize = 0;
            constSize = 0;
            for t=2:network.T-1
                for i=1:robotNum
                    % find arcIndex
                    nodeID = network.Structure(network.GoalRCT(i,1),network.GoalRCT(i,2),t);
                    outArcIndices = network.OutArcCell{network.GoalRCT(i,1),network.GoalRCT(i,2),t-1};
                    
                    if ~isempty(outArcIndices)
                        for j = outArcIndices
                            if network.Arcs(j,2) == nodeID
                                arcIndex = j;
                                break;
                            end
                        end
                    end
                    
                    % first
                    constSize = constSize + 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+t*robotNum+i;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 0;
                    
                    %second
                    constSize = constSize + 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = arcIndex;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 0;
                    
                    %third
                    constSize = constSize + 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = -1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcNum+t*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = arcIndex;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 1;
                end
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            b(constSize +1:end,:) = [];
            
            constraint.Num = constSize;
            constraint.Type = 0;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
            
        end
        
        %% constraint lateness, inequality
        function constraint = getConstraintLateness(obj,network,robotNum)
            AI = zeros(robotNum*2*network.T,1);
            AJ = zeros(robotNum*2*network.T,1);
            AV = ones(robotNum*2*network.T,1);
            b = zeros(robotNum,1);
            
            vecSize = 0;
            for i=1:robotNum
                for t=1:network.T
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = i;
                    AJ(vecSize,1) = network.ArcNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = -1;
                end
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = network.ArcNum + network.T*robotNum+1;
                AV(vecSize,1) = -1;
                
                b(i,1) = network.GoalRCT(i,3)-network.T-1;
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            
            constraint.Num = robotNum;
            constraint.Type = 0;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
        end
        
        %% objective: makespan
        function f = getObjectiveMakespan(obj,network,robotNum)
            f = zeros(1,network.ArcNum);
            for i=1:robotNum
                nodeID = network.SourceVec(i,1);
                node = network.Nodes(nodeID,:);
                outArcIDs = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                for index = outArcIDs
                    f(1,index)=1;
                end
            end
            
            for i=1:robotNum
                nodeID = network.SinkVec(i,1);
                node = network.Nodes(nodeID,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                for index = inArcIDs
                    f(1,index)=1;
                end
            end
            
        end
        
        %% objective : total arrival time
        function f = getObjectiveTotalTime(obj,network,robotNum)
            f = zeros(1,network.ArcNum+network.T*robotNum);
            f(1,network.ArcNum+1:end) = 1;
        end
        
        %% objective: maximum lateness
        function f = getObjectiveMaxLateness(obj,network,robotNum)
            f = zeros(1,network.ArcNum+network.T*robotNum+1);
            f(1,end) = 1;
        end
        
        %% objective: total tardiness
        function f = getObjectiveTotalTardiness(obj,network,robotNum)
            f = zeros(1,network.ArcNum+network.T*robotNum);
            for t=1:network.T
                for i=1:robotNum
                    if t <= network.GoalRCT(i,3)
                        val = 0;
                    else
                        val = 1;
                    end
                    f(1,network.ArcNum+(t-1)*robotNum+i) = val;
                end
            end
        end
        
        %% objective : sum of unit penalty
        function f = getObjectiveTotalUP(obj,network,robotNum)
            f = zeros(1,network.ArcNum);
            for i=1:robotNum
                node = network.GoalRCT(i,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                for index = inArcIDs
                    f(1,index) = 1;
                end
            end
        end
        
    end
end

