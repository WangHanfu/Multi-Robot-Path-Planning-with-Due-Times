clear;
clc;
objectiveSelect = 0;
printInfo = true;

load('instances20x20x0%.mat');
typeNum = size(InstanceSet,1);
typeNum = 6;
instanceNum = size(InstanceSet,2);
Results = cell(typeNum,instanceNum);
for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x0-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        tic;
        result.Solutions = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
        result.ComputeTime = toc;  
        Results{typeID,instanceID} = result;
        fprintf("Compute Time is %f\n",result.ComputeTime);
        save('Results20x20x0-makespan','Results');       
    end
end

load('instances20x20x10%.mat');
typeNum = size(InstanceSet,1);
typeNum = 6;
instanceNum = size(InstanceSet,2);
Results = cell(typeNum,instanceNum);
for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x10-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        tic;
        result.Solutions = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
        result.ComputeTime = toc;  
        Results{typeID,instanceID} = result;
        fprintf("Compute Time is %f\n",result.ComputeTime);
        save('Results20x20x10-makespan','Results');       
    end
end

load('instances20x20x20%.mat');
typeNum = size(InstanceSet,1);
typeNum = 6;
instanceNum = size(InstanceSet,2);
Results = cell(typeNum,instanceNum);
for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x20-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        tic;
        result.Solutions = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,0);                   
        result.ComputeTime = toc;  
        Results{typeID,instanceID} = result;
        fprintf("Compute Time is %f\n",result.ComputeTime);
        save('Results20x20x20-makespan','Results');       
    end
end
