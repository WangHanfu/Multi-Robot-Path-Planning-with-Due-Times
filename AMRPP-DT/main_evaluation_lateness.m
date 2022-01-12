clear;
clc;
objectiveSelect = 5;
printInfo = false;

load('instances20x20x0%.mat');
typeNum = size(InstanceSet,1);
instanceNum = size(InstanceSet,2);
Results = cell(typeNum,instanceNum);

for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x0-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        mat = instance.GoalRCT;
        mat(:,3) = 5;
        Results{typeID,instanceID} = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,mat,objectiveSelect,printInfo,0);                   
        save('AMRPP-DT-Results20x20x0-lateness','Results');       
    end
end
 
load('instances20x20x10%.mat');
typeNum = size(InstanceSet,1);
instanceNum = size(InstanceSet,2);
Results = cell(typeNum,instanceNum);

for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x10-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        mat = instance.GoalRCT;
        mat(:,3) = 5;
        Results{typeID,instanceID} = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,mat,objectiveSelect,printInfo,0);                   
        save('AMRPP-DT-Results20x20x10-lateness','Results');       
    end
end
 
load('instances20x20x20%.mat');
typeNum = size(InstanceSet,1);
instanceNum = size(InstanceSet,2);
Results = cell(typeNum,instanceNum);

for typeID = 1:typeNum
    for instanceID = 1:instanceNum
        instance = InstanceSet{typeID,instanceID};
        fprintf("Solving instance20x20x20-%d-%d with robot number %d\n",typeID,instanceID,instance.RobotNum);
        mat = instance.GoalRCT;
        mat(:,3) = 5;
        Results{typeID,instanceID} = MRPP(instance.RobotNum,instance.Map,instance.StartRCT,mat,objectiveSelect,printInfo,0);                   
        save('AMRPP-DT-Results20x20x20-lateness','Results');       
    end
end
 
