classdef FlowNetwork<handle
    
    properties
        Map
        T
        Structure
        
        Nodes
        Arcs
        NodeNum
        ArcNum
        
        StartRCT
        GoalRCT
        
        OutArcCell
        InArcCell
        
        SourceVec
        SinkVec
        
        ConflictArcPair
        ConflictNum
        
        Neighbors = [1 0 1;-1 0 1;0 1 1;0 -1 1;0 0 1]; %4-connected neighbors
    end
    
    methods
        function obj = FlowNetwork(map,T,startRCT,goalRCT)
            obj.Map = map;
            obj.T = T;
            
            %% nodes in the network
            structure = zeros(map.Height,map.Width,T);
            nodes = zeros(map.VertexNum*T,3);
            nodeID = 0;
            for t=1:T
                for i=1:map.Height
                    for j=1:map.Width
                        if map.MapGrid(i,j) == 0
                            nodeID = nodeID + 1;
                            structure(i,j,t) = nodeID;
                            nodes(nodeID,:) = [i,j,t];
                        end
                    end
                end
            end
            
            %% arcs in the network: R-,R+,C-,C+,STALL
            arcs = zeros(map.VertexNum*T*5,2);
            outArcCell = cell(map.Height,map.Width,T);
            inArcCell = cell(map.Height,map.Width,T);
            arcID = 0;
            for t = 1:T-1
                for i=1:map.Height
                    for j=1:map.Width
                        if map.MapGrid(i,j) == 0
                            for k=1:5
                                newI = i + obj.Neighbors(k,1);
                                newJ = j + obj.Neighbors(k,2);
                                newT = t + 1;
                                if  newI >= 1 && newI <= map.Height && newJ >= 1 && newJ <= map.Width && structure(newI,newJ,newT) ~= 0
                                    newNodeID = structure(newI,newJ,newT);
                                    arcID = arcID + 1;
                                    arcs(arcID,:) = [structure(i,j,t) newNodeID];
                                    outArcCell{i,j,t} = [outArcCell{i,j,t},arcID];
                                    inArcCell{newI,newJ,newT} = [inArcCell{newI,newJ,newT},arcID];
                                end
                            end
                        end
                    end
                end
            end
            arcs(arcID+1:end,:) = [];
            
            robotNum = size(startRCT,1);
            sourceIDs = zeros(robotNum,1);
            sinkIDs = zeros(robotNum,1);
            for i=1:robotNum
                sourceIDs(i,1) = structure(startRCT(i,1),startRCT(i,2),startRCT(i,3));
                sinkIDs(i,1) = structure(goalRCT(i,1),goalRCT(i,2),T); %notice!
            end
            
            obj.Structure = structure;
            obj.Nodes = nodes;
            obj.Arcs = arcs;
            obj.NodeNum = nodeID;
            obj.ArcNum = arcID;
            
            obj.SourceVec = sourceIDs;
            obj.SinkVec = sinkIDs;
            
            obj.InArcCell = inArcCell;
            obj.OutArcCell = outArcCell;
            
            obj.StartRCT = startRCT;
            obj.GoalRCT = goalRCT;
            obj.getConflictArcs();
        end
        
        function getConflictArcs(obj)
            conflictArcPair = zeros(obj.ArcNum,2);
            conflictNum = 0;
            flagVec = zeros(obj.ArcNum,1);
            for i=1:obj.ArcNum
                if flagVec(i,1) == 1
                    continue;
                end
                
                %(u_t,v_{t+1}),(v_t,u_{t+1}) pair
                arc = obj.Arcs(i,:);
                nodeU = obj.Nodes(arc(1,1),:);
                nodeV = obj.Nodes(arc(1,2),:);
                
                if nodeU(1,1)==nodeV(1,1) && nodeU(1,2)==nodeV(1,2)
                    flagVec(i,1) = 1;
                    continue;
                end
                
                newNodeU = [nodeU(1,1:2),nodeV(1,3)];
                newNodeV = [nodeV(1,1:2),nodeU(1,3)];
                newIDU = obj.Structure(newNodeU(1,1),newNodeU(1,2),newNodeU(1,3));
                newIDV = obj.Structure(newNodeV(1,1),newNodeV(1,2),newNodeV(1,3));
                if newIDU==0 || newIDV==0
                    flagVec(i,1) = 1;
                    continue;
                end
                
                newArcIndices = obj.OutArcCell{newNodeV(1,1),newNodeV(1,2),newNodeV(1,3)};
                if ~isempty(newArcIndices)
                    for j = newArcIndices
                        arc2 = obj.Arcs(j,:);
                        if arc2(1,2) == newIDU
                            conflictNum = conflictNum + 1;
                            conflictArcPair(conflictNum,:) = [i j];
                            flagVec(i,1) = 1;
                            flagVec(j,1) = 1;
                            break;
                        end
                    end
                else
                    flagVec(i,1) = 1;
                end
            end
            conflictArcPair(conflictNum+1:end,:)=[];
            obj.ConflictArcPair = conflictArcPair;
            obj.ConflictNum = conflictNum;
        end

    end
end

