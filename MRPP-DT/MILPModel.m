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
                case 0 % makespan, default method
                    constraint1 = obj.getConstraintArcCapacity(network,robotNum); %inequality
                    constraint2 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    %constraint3 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintFlowConservationAtTerminalNodesForMakespanDefault(network,robotNum); %equality
                    constraint4 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint1.AI;
                    AineqJ = constraint1.AJ;
                    AineqV = constraint1.AV;
                    bineq = constraint1.b;
                    AineqSize = AineqSize + constraint1.Num;
                    
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
                    AeqI = constraint2.AI;
                    AeqJ = constraint2.AJ;
                    AeqV = constraint2.AV;
                    beq = constraint2.b;
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint3.AI];
                    AeqJ = [AeqJ; constraint3.AJ];
                    AeqV = [AeqV; constraint3.AV];
                    beq = [beq; constraint3.b];
                    AeqSize = AeqSize + constraint3.Num;
                    
                    f = obj.getObjectiveMakespanDefault(network,robotNum);
                    AWidth = network.ArcRobotPairNum;
                    
                case 1 %makespan, the first method
                    constraint1 = obj.getConstraintArcCapacity(network,robotNum); %inequality
                    constraint2 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    %constraint3 = obj.getConstraintFlowConservationAtTerminalNodesForMakespanDefault(network,robotNum); %equality
                    constraint4 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint7 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    constraint8 = obj.getConstraintMakespan(network,robotNum); %inequality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint1.AI;
                    AineqJ = constraint1.AJ;
                    AineqV = constraint1.AV;
                    bineq = constraint1.b;
                    AineqSize = AineqSize + constraint1.Num;
                    
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
                    
                    AineqI = [AineqI;AineqSize + constraint6.AI];
                    AineqJ = [AineqJ; constraint6.AJ];
                    AineqV = [AineqV; constraint6.AV];
                    bineq = [bineq; constraint6.b];
                    AineqSize = AineqSize + constraint6.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint8.AI];
                    AineqJ = [AineqJ; constraint8.AJ];
                    AineqV = [AineqV; constraint8.AV];
                    bineq = [bineq; constraint8.b];
                    AineqSize = AineqSize + constraint8.Num;
                                        
                    AeqSize = 0;
                    
                    AeqI = constraint2.AI;
                    AeqJ = constraint2.AJ;
                    AeqV = constraint2.AV;
                    beq = constraint2.b;
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint3.AI];
                    AeqJ = [AeqJ; constraint3.AJ];
                    AeqV = [AeqV; constraint3.AV];
                    beq = [beq; constraint3.b];
                    AeqSize = AeqSize + constraint3.Num;
                                                            
                    AeqI = [AeqI;AeqSize + constraint7.AI];
                    AeqJ = [AeqJ; constraint7.AJ];
                    AeqV = [AeqV; constraint7.AV];
                    beq = [beq; constraint7.b];
                    AeqSize = AeqSize + constraint7.Num;
                                                            
                    f = obj.getObjectiveMakespan(network,robotNum);
                    AWidth = network.ArcRobotPairNum + network.T*robotNum + 1;
                                    
                case 2 %total arrival time
                    constraint1 = obj.getConstraintArcCapacity(network,robotNum); %inequality
                    constraint2 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    %constraint3 = obj.getConstraintFlowConservationAtTerminalNodesForMakespan(network,robotNum); %equality
                    constraint4 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint7 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint1.AI;
                    AineqJ = constraint1.AJ;
                    AineqV = constraint1.AV;
                    bineq = constraint1.b;
                    AineqSize = AineqSize + constraint1.Num;
                    
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
                    
                    AineqI = [AineqI;AineqSize + constraint6.AI];
                    AineqJ = [AineqJ; constraint6.AJ];
                    AineqV = [AineqV; constraint6.AV];
                    bineq = [bineq; constraint6.b];
                    AineqSize = AineqSize + constraint6.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint2.AI;
                    AeqJ = constraint2.AJ;
                    AeqV = constraint2.AV;
                    beq = constraint2.b;
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint3.AI];
                    AeqJ = [AeqJ; constraint3.AJ];
                    AeqV = [AeqV; constraint3.AV];
                    beq = [beq; constraint3.b];
                    AeqSize = AeqSize + constraint3.Num;
                                                            
                    AeqI = [AeqI;AeqSize + constraint7.AI];
                    AeqJ = [AeqJ; constraint7.AJ];
                    AeqV = [AeqV; constraint7.AV];
                    beq = [beq; constraint7.b];
                    AeqSize = AeqSize + constraint7.Num;
                                                            
                    f = obj.getObjectiveTotalTime(network,robotNum);
                    AWidth = network.ArcRobotPairNum + network.T*robotNum;
                                    
                case 5 % maximum lateness
                    constraint1 = obj.getConstraintArcCapacity(network,robotNum); %inequality
                    constraint2 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    %constraint3 = obj.getConstraintFlowConservationAtTerminalNodesForMakespan(network,robotNum); %equality
                    constraint4 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint7 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    constraint8 = obj.getConstraintLateness(network,robotNum); %inequality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint1.AI;
                    AineqJ = constraint1.AJ;
                    AineqV = constraint1.AV;
                    bineq = constraint1.b;
                    AineqSize = AineqSize + constraint1.Num;
                    
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
                    
                    AineqI = [AineqI;AineqSize + constraint6.AI];
                    AineqJ = [AineqJ; constraint6.AJ];
                    AineqV = [AineqV; constraint6.AV];
                    bineq = [bineq; constraint6.b];
                    AineqSize = AineqSize + constraint6.Num;
                    
                    AineqI = [AineqI;AineqSize + constraint8.AI];
                    AineqJ = [AineqJ; constraint8.AJ];
                    AineqV = [AineqV; constraint8.AV];
                    bineq = [bineq; constraint8.b];
                    AineqSize = AineqSize + constraint8.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint2.AI;
                    AeqJ = constraint2.AJ;
                    AeqV = constraint2.AV;
                    beq = constraint2.b;
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint3.AI];
                    AeqJ = [AeqJ; constraint3.AJ];
                    AeqV = [AeqV; constraint3.AV];
                    beq = [beq; constraint3.b];
                    AeqSize = AeqSize + constraint3.Num;
                                                            
                    AeqI = [AeqI;AeqSize + constraint7.AI];
                    AeqJ = [AeqJ; constraint7.AJ];
                    AeqV = [AeqV; constraint7.AV];
                    beq = [beq; constraint7.b];
                    AeqSize = AeqSize + constraint7.Num;
                                                            
                    f = obj.getObjectiveMaxLateness(network,robotNum);
                    AWidth = network.ArcRobotPairNum + network.T*robotNum + 1;
                    
                case 6   % total tardiness
                    constraint1 = obj.getConstraintArcCapacity(network,robotNum); %inequality
                    constraint2 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    %constraint3 = obj.getConstraintFlowConservationAtTerminalNodesForMakespan(network,robotNum); %equality
                    constraint4 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                    constraint6 = obj.getConstraintLogicOperation(network,robotNum); %inequality
                    constraint7 = obj.getConstraintLogicOperationEqual(network,robotNum); %equality
                    
                    AineqSize = 0;
                    
                    AineqI = constraint1.AI;
                    AineqJ = constraint1.AJ;
                    AineqV = constraint1.AV;
                    bineq = constraint1.b;
                    AineqSize = AineqSize + constraint1.Num;
                    
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
                    
                    AineqI = [AineqI;AineqSize + constraint6.AI];
                    AineqJ = [AineqJ; constraint6.AJ];
                    AineqV = [AineqV; constraint6.AV];
                    bineq = [bineq; constraint6.b];
                    AineqSize = AineqSize + constraint6.Num;
                    
                    AeqSize = 0;
                    
                    AeqI = constraint2.AI;
                    AeqJ = constraint2.AJ;
                    AeqV = constraint2.AV;
                    beq = constraint2.b;
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint3.AI];
                    AeqJ = [AeqJ; constraint3.AJ];
                    AeqV = [AeqV; constraint3.AV];
                    beq = [beq; constraint3.b];
                    AeqSize = AeqSize + constraint3.Num;
                                                            
                    AeqI = [AeqI;AeqSize + constraint7.AI];
                    AeqJ = [AeqJ; constraint7.AJ];
                    AeqV = [AeqV; constraint7.AV];
                    beq = [beq; constraint7.b];
                    AeqSize = AeqSize + constraint7.Num;
                                                            
                    f = obj.getObjectiveTotalTardiness(network,robotNum);
                    AWidth = network.ArcRobotPairNum + network.T*robotNum;
                    
                case 7  % total penalties
                    constraint1 = obj.getConstraintArcCapacity(network,robotNum); %inequality
                    constraint2 = obj.getConstraintFlowConservationAtNonTerminalNodes(network,robotNum); %equality
                    constraint3 = obj.getConstraintFlowConservationAtTerminalNodes(network,robotNum); %equality
                    %constraint3 = obj.getConstraintFlowConservationAtTerminalNodesForMakespanDefault(network,robotNum); %equality
                    constraint4 = obj.getConstraintVertexConflict(network,robotNum); %inequality
                    constraint5 = obj.getConstraintEdgeConflict(network,robotNum); %inequality
                   
                    AineqSize = 0;
                    
                    AineqI = constraint1.AI;
                    AineqJ = constraint1.AJ;
                    AineqV = constraint1.AV;
                    bineq = constraint1.b;
                    AineqSize = AineqSize + constraint1.Num;
                    
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
                    
                    AeqI = constraint2.AI;
                    AeqJ = constraint2.AJ;
                    AeqV = constraint2.AV;
                    beq = constraint2.b;
                    AeqSize = AeqSize + constraint2.Num;
                    
                    AeqI = [AeqI;AeqSize + constraint3.AI];
                    AeqJ = [AeqJ; constraint3.AJ];
                    AeqV = [AeqV; constraint3.AV];
                    beq = [beq; constraint3.b];
                    AeqSize = AeqSize + constraint3.Num;
                                                                                                                        
                    f = obj.getObjectiveTotalUP(network,robotNum);
                    AWidth = network.ArcRobotPairNum;
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
        
        %% arc capacity constraints,inequality,ArcNum
        function constraint = getConstraintArcCapacity(obj,network,robotNum)
            constraint.Num = network.UsedArcNum;
            constraint.Type = 0;
            constraint.AI = network.ArcRobotPairinUsedArcs;
            constraint.AJ = (1:network.ArcRobotPairNum)';
            constraint.AV = ones(network.ArcRobotPairNum,1);
            constraint.b = ones(network.UsedArcNum,1);
        end
        
        %% flow conservation constraints at all non-terminal nodes, equality, NodeNum*robotNum
        function constraint = getConstraintFlowConservationAtNonTerminalNodes(obj,network,robotNum)
            constNum = (network.NodeNum-2*robotNum)*robotNum;
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
                %find all in and out arcs
                
                if  ~any(network.ReachableNodeBoolMat(i,:))
                    continue;
                end
                
                node = network.Nodes(i,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                outArcIDs = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                if ~isempty(inArcIDs) && ~isempty(outArcIDs)
                    for j = 1:robotNum
                        if network.ReachableNodeBoolMat(i,j) %% used by robot j
                            constSize = constSize + 1;
                            for index = inArcIDs
                                if network.ReachableArcBoolMat(index,j)
                                    vecSize = vecSize + 1;
                                    AI(vecSize,1) = constSize;
                                    AJ(vecSize,1) = network.ReachableArcMat(index,j);
                                    AV(vecSize,1) = 1;
                                end
                            end
                            
                            for index = outArcIDs
                                if network.ReachableArcBoolMat(index,j)
                                    vecSize = vecSize + 1;
                                    AI(vecSize,1) = constSize;
                                    AJ(vecSize,1) = network.ReachableArcMat(index,j);
                                    AV(vecSize,1) = -1;
                                end
                            end
                        end
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
                    if network.ReachableArcBoolMat(index,i)
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = i;
                        AJ(vecSize,1) = network.ReachableArcMat(index,i);
                        AV(vecSize,1) = 1;
                    end
                end
            end
            
            for i = 1:robotNum
                nodeID = network.SinkVec(i,1);
                node = network.Nodes(nodeID,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                for index = inArcIDs
                    if network.ReachableArcBoolMat(index,i)
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = robotNum + i;
                        AJ(vecSize,1) = network.ReachableArcMat(index,i);
                        AV(vecSize,1) = 1;
                    end
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
        
        %% arc flow conservation constraints at terminal nodes,equality,robotNum
        function constraint = getConstraintFlowConservationAtTerminalNodesForMakespanDefault(obj,network,robotNum)
            AI = zeros(2*robotNum*5,1);
            AJ = zeros(2*robotNum*5,1);
            AV = zeros(2*robotNum*5,1);
            
            vecSize = 0;
            for i=1:robotNum
                nodeID = network.SourceVec(i,1);
                node = network.Nodes(nodeID,:);
                outArcIDs = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                for index = outArcIDs
                    if network.ReachableArcBoolMat(index,i)
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = i;
                        AJ(vecSize,1) = network.ReachableArcMat(index,i);
                        AV(vecSize,1) = 1;
                    end
                end
                
                nodeID = network.SinkVec(i,1);
                node = network.Nodes(nodeID,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                for index = inArcIDs
                    if network.ReachableArcBoolMat(index,i)
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = i;
                        AJ(vecSize,1) = network.ReachableArcMat(index,i);
                        AV(vecSize,1) = -1;
                    end
                end
                
            end
            
            AI(vecSize + 1:end,:) = [];
            AJ(vecSize + 1:end,:) = [];
            AV(vecSize + 1:end,:) = [];
            
            constraint.Num = robotNum;
            constraint.Type = 1;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = zeros(robotNum,1);
        end
        
        %% vertex conflict constraints,inequality,at most NodeNum
        function constraint = getConstraintVertexConflict(obj,network,robotNum)
            AI = zeros(network.NodeNum*robotNum*5,1);
            AJ = zeros(network.NodeNum*robotNum*5,1);
            AV = ones(network.NodeNum*robotNum*5,1);
            b = ones(network.NodeNum,1);
            
            vecSize = 0;
            constSize = 0;
            for i=1:network.NodeNum
                
                if  ~any(network.ReachableNodeBoolMat(i,:))
                    continue;
                end
                
                %find all in arcs
                node = network.Nodes(i,:);
                outArcIndices = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                if ~isempty(outArcIndices)
                    constSize = constSize + 1;
                    for arcID = outArcIndices
                        for j = 1:robotNum
                            if network.ReachableArcBoolMat(arcID,j)
                                vecSize = vecSize + 1;
                                AI(vecSize,1) = constSize;
                                AJ(vecSize,1) = network.ReachableArcMat(arcID,j);
                                AV(vecSize,1) = 1;
                            end
                        end
                    end
                end
            end
            
            AI(vecSize+1:end,:) = [];
            AJ(vecSize+1:end,:) = [];
            AV(vecSize+1:end,:) = [];
            b(constSize +1:end,:) = [];
            
            constraint.Num = constSize;
            constraint.Type = 1;
            constraint.AI = AI;
            constraint.AJ = AJ;
            constraint.AV = AV;
            constraint.b = b;
        end
        
        %% edge conflict constraints. inequality,conflictNum
        function constraint = getConstraintEdgeConflict(obj,network,robotNum)
            
            AI = zeros(network.ConflictNum*robotNum*2,1);
            AJ = zeros(network.ConflictNum*robotNum*2,1);
            AV = ones(network.ConflictNum*robotNum*2,1);
            b = ones(network.ConflictNum,1);
            
            vecSize = 0;
            for i=1:network.ConflictNum
                arcIndex1 = network.ConflictArcPair(i,1);
                arcIndex2 = network.ConflictArcPair(i,2);
                
                for j=1:robotNum
                    if network.ReachableArcBoolMat(arcIndex1,j)
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = i;
                        AJ(vecSize,1) = network.ReachableArcMat(arcIndex1,j);
                        AV(vecSize,1) = 1;
                    end
                    if network.ReachableArcBoolMat(arcIndex2,j)
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = i;
                        AJ(vecSize,1) = network.ReachableArcMat(arcIndex2,j);
                        AV(vecSize,1) = 1;
                    end
                end
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
        
        %% constraint (6). ineualities. totally (T-1)*robotNum
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
                    AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = constSize;
                    AJ(vecSize,1) = network.ArcRobotPairNum+t*robotNum+i;
                    AV(vecSize,1) = -1;
                    
                    b(constSize,1) = 0;
                    
                    %second
                    constSize = constSize + 1;
                    if network.ReachableArcBoolMat(arcIndex,i) == true
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                        AV(vecSize,1) = 1;
                        
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ReachableArcMat(arcIndex,i);
                        AV(vecSize,1) = -1;
                    else
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                        AV(vecSize,1) = 1;
                    end
                    b(constSize,1) = 0;
                    
                    %third
                    constSize = constSize + 1;
                    if network.ReachableArcBoolMat(arcIndex,i) == true
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                        AV(vecSize,1) = -1;
                        
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ArcRobotPairNum+t*robotNum+i;
                        AV(vecSize,1) = 1;
                        
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ReachableArcMat(arcIndex,i);
                        AV(vecSize,1) = -1;
                    else
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                        AV(vecSize,1) = -1;
                        
                        vecSize = vecSize + 1;
                        AI(vecSize,1) = constSize;
                        AJ(vecSize,1) = network.ArcRobotPairNum+t*robotNum+i;
                        AV(vecSize,1) = 1;
                    end
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
        
        %% constraint (7). equality, y_i^T is x_i^T
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
                
                if network.ReachableArcBoolMat(arcIndex,i) == true
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = i;
                    AJ(vecSize,1) = network.ArcRobotPairNum+(network.T-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                    
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = i;
                    AJ(vecSize,1) = network.ReachableArcMat(arcIndex,i);
                    AV(vecSize,1) = -1;
                else
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = i;
                    AJ(vecSize,1) = network.ArcRobotPairNum+(network.T-1)*robotNum+i;
                    AV(vecSize,1) = 1;
                end
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
        
        % makespan objective
        function constraint = getConstraintMakespan(obj,network,robotNum)
            AI = zeros(robotNum*2*network.T,1);
            AJ = zeros(robotNum*2*network.T,1);
            AV = ones(robotNum*2*network.T,1);
            b = zeros(robotNum,1);
            
            vecSize = 0;
            for i=1:robotNum
                for t=1:network.T
                    vecSize = vecSize + 1;
                    AI(vecSize,1) = i;
                    AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = -1;                                                                         
                end
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = network.ArcRobotPairNum+network.T*robotNum+1;
                AV(vecSize,1) = -1; 
                b(i,1) = -network.T;
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
                    AJ(vecSize,1) = network.ArcRobotPairNum+(t-1)*robotNum+i;
                    AV(vecSize,1) = -1;                                                                          
                end
                vecSize = vecSize + 1;
                AI(vecSize,1) = i;
                AJ(vecSize,1) = network.ArcRobotPairNum + network.T*robotNum+1;
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
                                        
        %% objective : makespan,another realization
        function f = getObjectiveMakespanDefault(obj,network,robotNum)
            f = zeros(1,network.ArcRobotPairNum);
            for i=1:robotNum
                nodeID = network.SourceVec(i,1);
                node = network.Nodes(nodeID,:);
                outArcIDs = network.OutArcCell{node(1,1),node(1,2),node(1,3)};
                for index = outArcIDs
                    if network.ReachableArcBoolMat(index,i)
                        loc = network.ReachableArcMat(index,i);
                        f(1,loc) = 1;
                    end
                end
            end
        end
        
        %% objective: makespan
        function f = getObjectiveMakespan(obj,network,robotNum)
            f = zeros(1,network.ArcRobotPairNum+network.T*robotNum+1);
            f(1,end) = 1;
        end
        
        %% objective : total arrival time
        function f = getObjectiveTotalTime(obj,network,robotNum)
            f = zeros(1,network.ArcRobotPairNum+network.T*robotNum);
            f(1,network.ArcRobotPairNum+1:end) = 1;
        end
        
        %% objective: maximum lateness
        function f = getObjectiveMaxLateness(obj,network,robotNum)
            f = zeros(1,network.ArcRobotPairNum+network.T*robotNum+1);
            f(1,end) = 1;
        end
        
        %% objective: total tardiness
        function f = getObjectiveTotalTardiness(obj,network,robotNum)
            f = zeros(1,network.ArcRobotPairNum+network.T*robotNum);
            for t=1:network.T
                for i=1:robotNum
                    if t <= network.GoalRCT(i,3)
                        val = 0;
                    else
                        val = 1;
                    end
                    f(1,network.ArcRobotPairNum+(t-1)*robotNum+i) = val;
                end
            end
        end
        
        %% objective : sum of unit penalty
        function f = getObjectiveTotalUP(obj,network,robotNum)
            f = zeros(1,network.ArcRobotPairNum);
            for i=1:robotNum
                node = network.GoalRCT(i,:);
                inArcIDs = network.InArcCell{node(1,1),node(1,2),node(1,3)};
                for index = inArcIDs
                    if network.ReachableArcBoolMat(index,i)
                        loc = network.ReachableArcMat(index,i);
                        f(1,loc) = 1;
                    end
                end
            end
        end
        
    end
end

