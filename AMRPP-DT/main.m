%% generate a random instance, plan paths, visualize paths, and record video.

clear;
clc;

objectiveSelect = 7;
intepolate = 5;
printInfo = true;

%% map parameters and create map
load('map20x20x0%');
robotNum=50;
locations = map.Vertices;
locationNum = map.VertexNum;

stations = zeros(2*robotNum,2);
taskIndex = 1;
vec = randperm(locationNum);
for i=1:robotNum*2
    stations(i,1:2)= locations(vec(i),:);
end

StartRobotStates = ones(robotNum,3);
GoalRobotStates = ones(robotNum,3);
StartRobotStates(:,1:2) = stations(1:robotNum,:);
GoalRobotStates(:,1:2) = stations(robotNum+1:end,:);

for i=1:robotNum    
    startID = vec(1,i);
    goalID = vec(1,i+robotNum);
    dueTime = map.DistMat(startID,goalID);
    dueTime = 5;
    %dueTime = map.DistMat(startID,goalID)+unidrnd(5);
    GoalRobotStates(i,3)=dueTime+1;
    
end

ColorMat=rand(robotNum,3);

%% simulation
simTime = 1;
CurrentRobotStates = zeros(robotNum,3);
NextRobotStates = zeros(robotNum,3);
TaskStatus = ones(robotNum,1);
ArrivalTime = zeros(robotNum,1);

tic;
solutionCandidates = MRPP(robotNum,map,StartRobotStates,GoalRobotStates,objectiveSelect,printInfo,max(GoalRobotStates(:,3)));               
toc;

bestObjValue = 100000; 
bestIndex = 0;
for i=1:size(solutionCandidates,1)
    solution = solutionCandidates{i,1};
    if ~isempty(solution)
        temp = solution.ObjectiveValue;
        if temp < bestObjValue
            bestObjValue = temp;
            bestIndex = i;
        end
    end
end

bestSolution = solutionCandidates{bestIndex,1};
allPathCell = bestSolution.AllPathCell;
bestObjVal = bestSolution.ObjectiveValue;

disp(bestObjVal);
disp(bestSolution.T);

for i=1:robotNum
    path = allPathCell{i,1};
    for j=size(path,1):-1:1
        tempA = path(j,1);
        tempB = path(j,2);
        if tempA==GoalRobotStates(i,1) && tempB == GoalRobotStates(i,2)
            ArrivalTime(i,1) = j;
        end
    end
end

sz=get(0,'screensize');
h=figure('outerposition',sz);
assignin('base','h',h); %in case of any callback errors.
hold on;
grid on;
set(gca,'xtick',-1:1:map.Width+1);
set(gca,'ytick',-1:1:map.Height+1);
axis equal;
axis([-1 map.Width+1 -1 map.Height+1]);
axis manual;
video = VideoWriter('simulation');
video.FrameRate=intepolate;
open(video);

%plotAll(map,StartRobotStates,GoalRobotStates,ColorMat);

for i=1:robotNum
    path = allPathCell{i,1};
    CurrentRobotStates(i,:)=path(1,1:3);
end

while(any(TaskStatus))
    for i=1:robotNum
        path = allPathCell{i,1};
        if size(path,1)>simTime
            NextRobotStates(i,:)=path(simTime+1,1:3);            
        end        
        for j=1:robotNum        
            if  NextRobotStates(i,1)==GoalRobotStates(j,1) &&  NextRobotStates(i,2)==GoalRobotStates(j,2)
                TaskStatus(j,1) = 0;
            end 
        end
    end
    %continuous simulation
    for i=0:intepolate-1
        tempRobotStates=CurrentRobotStates+i*(NextRobotStates-CurrentRobotStates)/intepolate;   
        plotAll(map,simTime,tempRobotStates,GoalRobotStates,ArrivalTime,ColorMat,objectiveSelect);        
        frame = getframe;
        writeVideo(video,frame);
        cla;
    end
    CurrentRobotStates=NextRobotStates;
    simTime = simTime+1;
end

for i = 1:intepolate+10
    plotAll(map,simTime,CurrentRobotStates,GoalRobotStates,ArrivalTime,ColorMat,objectiveSelect);
    pause(0.1);
    frame = getframe;
    writeVideo(video,frame);
    cla;
end

close(video);