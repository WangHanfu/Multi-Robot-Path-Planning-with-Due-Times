clear;
clc;
objectiveSelect = 6;
printInfo = true;

load('instances20x20x0%.mat');
typeNum = 6;
instanceNum = size(InstanceSet,2);
load('Results20x20x0-makespan.mat');
ResultsMS=Results;

Results = cell(typeNum,instanceNum);
for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x0-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        msResult = ResultsMS{typeID,instanceID}.Solutions;
        result = msResult{1,1};
        Results{typeID,instanceID} = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,result.T);                   
        save('Results20x20x0-tardiness','Results');       
    end
end
 
load('instances20x20x10%.mat');
typeNum = 6;
instanceNum = size(InstanceSet,2);
load('Results20x20x10-makespan.mat');
ResultsMS=Results;

Results = cell(typeNum,instanceNum);
for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x10-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        msResult = ResultsMS{typeID,instanceID}.Solutions;
        result = msResult{1,1};
        Results{typeID,instanceID} = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,result.T);                   
        save('Results20x20x10-tardiness','Results');       
    end
end

load('instances20x20x20%.mat');
typeNum = 6;
instanceNum = size(InstanceSet,2);
load('Results20x20x20-makespan.mat');
ResultsMS=Results;

Results = cell(typeNum,instanceNum);
for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x20-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        msResult = ResultsMS{typeID,instanceID}.Solutions;
        result = msResult{1,1};
        Results{typeID,instanceID} = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,instance.GoalRCT,objectiveSelect,printInfo,result.T);                   
        save('Results20x20x20-tardiness','Results');       
    end
end