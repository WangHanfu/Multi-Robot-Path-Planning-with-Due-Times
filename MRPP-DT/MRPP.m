function solutionCandidates = MRPP(robotNum,map,startRCT,goalRCT,objectiveSelect,printInfo,inputT)
% objectiveSelect:0,1 = minMakespan, 2 = minSumArrivalTime
% objectiveSelect:3 = minMaxDistance, 4 = minSumDistance
% objectiveSelect:5 = minMaxLateness, 6 = minSumTardiness,7=minSumUnitPenalties
% output:optimal paths

%whether set suboptimal bound to accelerate optimization process 
setGap = true;
verbose = false;

%emperical value

if objectiveSelect==0 || objectiveSelect==7
    TSpan = 1;
else
    TSpan = 5; %emperical value
end
adaptiveParam = robotNum*0.2 + map.ObstacleProportion*20;

if inputT == 0
    Tmin = max(goalRCT(:,3));
else
    Tmin = inputT;
end

Tmax = Tmin + TSpan-1;
solutionCandidates = cell(TSpan,1);
solutionCount = 1;

for T=Tmin:Tmax
    if printInfo
        fprintf("T=%d\n",T);
    end
    tic;
    network = FlowNetwork(map,T,startRCT,goalRCT);
    milp = MILPModel(network,robotNum,objectiveSelect);
    Aineq=sparse(milp.AineqI,milp.AineqJ,milp.AineqV,milp.AineqSize,milp.AWidth);
    Aeq=sparse(milp.AeqI,milp.AeqJ,milp.AeqV,milp.AeqSize,milp.AWidth);
    bineq = milp.bineq;
    beq = milp.beq;
    f = milp.f;
        
    %% solving MILP equations by GUROBI
    model.obj = f;
    model.A = [Aineq; Aeq];%sparse
    varlen = milp.AWidth;
    model.vtype = repmat('B', varlen, 1);
    if objectiveSelect == 1
        model.vtype = [repmat('B', varlen-1, 1);'I'];
        model.lb = zeros(milp.AWidth,1);
        tempMat = ones(milp.AWidth,1);
        tempMat(end,1) = 1000000;
        model.ub = tempMat;
    elseif objectiveSelect == 5
        model.vtype = [repmat('B', varlen-1, 1);'I'];
        tempMat = zeros(milp.AWidth,1);
        tempMat(end,1) = -1000000;
        model.lb = tempMat;
        tempMat = ones(milp.AWidth,1);
        tempMat(end,1) = 1000000;
        model.ub = tempMat;
    else
        model.vtype = repmat('B', varlen, 1);
        model.lb = zeros(milp.AWidth,1);
        model.ub = ones(milp.AWidth,1);
    end
    model.rhs = full([bineq(:); beq(:)]); % rhs must be dense
    model.sense = [repmat('<',size(Aineq,1),1); repmat('=',size(Aeq,1),1)]; % constraints, equality or inequality
    %model.Threads = 8;
    if printInfo && verbose
        params.outputflag = 1;
    else
        params.outputflag = 0;
    end
    
    switch objectiveSelect
        case 0 %makespan
            model.modelsense = 'max'; % optimization
            %Note that you should always include a small tolerance in this value.
            %Without this, a bound that satisfies the intended termination criterion may not
            %actually lead to termination due to numerical round-off in the bound.
            model.BestObjStop = robotNum-0.1;
            model.BestBdStop = robotNum-0.1;
        case 1
            model.modelsense = 'min';
        case 2 % total time
            model.modelsense = 'max';
            params.timelimit = 600;
            %performance gap
            if setGap == true
                params.MIPGap = adaptiveParam/100;
            end
        case 3 %reserved for maximum of distance objective
            
        case 4 %reserved for total distance objective
            
        case 5 %maximum lateness
            model.modelsense = 'min';
            params.timelimit = 600;
            if setGap == true
                params.MIPGap = adaptiveParam/100;
            end
        case 6 % total tardiness
            model.modelsense = 'max';
            params.timelimit = 600;
            if setGap == true
                params.MIPGap = adaptiveParam/100;
            end
        case 7 %sum of unit penalties
            model.modelsense = 'max'; % optimization
            params.timelimit = 600;
            model.BestObjStop = robotNum-0.1;
            model.BestBdStop = robotNum-0.1;
    end
    
    %gurobi_write(model, 'mip1.lp');
    
    %-1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier, 3=concurrent, 4=deterministic concurrent, 5=deterministic concurrent simplex
    params.Method = -1;
    
    result = gurobi(model, params);
    
    %loop conditions
    if objectiveSelect == 0
        if  result.objval == robotNum
            allPathCell = cell(robotNum,1);
            currentSolution=result.x;
            for i = 1:network.ArcRobotPairNum
                if currentSolution(i,1)==1
                    arcID = network.ArcRobotPair(i,1);
                    robotID = network.ArcRobotPair(i,2);
                    allPathCell{robotID,1}=[allPathCell{robotID,1};network.Nodes(network.Arcs(arcID,2),:)];
                end
            end
            for i=1:robotNum
                allPathCell{i,1}=[startRCT(i,:);allPathCell{i,1}];
            end
            
            solution.T = T;
            solution.ComputeTime = toc;
            solution.AllPathCell = allPathCell;
            solution.ObjectiveValue = T; %when result.objval == robotNum, T is the optimal makespan
            solutionCandidates{solutionCount,1} = solution;
            break;
        else
            solution.T = T;
            solution.ComputeTime = toc;
            solution.AllPathCell = [];
            solution.ObjectiveValue = []; %when result.objval == robotNum, T is the optimal makespan
            solutionCandidates{solutionCount,1} = solution;
        end
    else
        if strcmp(result.status,'INFEASIBLE') || strcmp(result.status,'TIME_LIMIT')
            solution.T = T;
            solution.ComputeTime = toc;
            solution.AllPathCell = [];
            solution.ObjectiveValue = [];
            solutionCandidates{solutionCount,1} = solution;
            solutionCount = solutionCount +1;                        
        else
            allPathCell = cell(robotNum,1);
            currentSolution=result.x;
            for i = 1:network.ArcRobotPairNum
                if currentSolution(i,1)==1
                    arcID = network.ArcRobotPair(i,1);
                    robotID = network.ArcRobotPair(i,2);
                    allPathCell{robotID,1}=[allPathCell{robotID,1};network.Nodes(network.Arcs(arcID,2),:)];
                end
            end
            for i=1:robotNum
                allPathCell{i,1}=[startRCT(i,:);allPathCell{i,1}];
            end
            
            optimalValue = 0;
            switch objectiveSelect
                case 1
                    optimalValue = result.objval;
                case 2
                    optimalValue = robotNum*T-result.objval;
                case 5
                    optimalValue = result.objval;
                case 6
                    optimalValue = robotNum*T-sum(goalRCT(:,3))-result.objval;
                case 7
                    optimalValue = robotNum-result.objval;
            end
            
            solution.T = T;
            solution.ComputeTime = toc;
            solution.AllPathCell = allPathCell;
            solution.ObjectiveValue = optimalValue;
            solutionCandidates{solutionCount,1} = solution;
            solutionCount = solutionCount +1;
            if printInfo
                fprintf("optimal value = %d\n",optimalValue);
            end
        end
    end
end

end


