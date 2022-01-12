function plotAll(map,simT,RobotStates,GoalStates,ArrivalTime,ColorMat,objectiveSelect)
robotMarkerSize = 18;
obstacleMarkerSize = 36;
fontSize = 13;

height = map.Height;
width = map.Width;
robotNum=size(RobotStates,1);

rectangle('Position', [0,0,width,height],'lineWidth',5);
hold on;
[I,J] = find(map.MapGrid==1);
tempMat = [I J];
obstacles = tempMat;
obstacles(:,1) = tempMat(:,2)-0.5;
obstacles(:,2) = height+1-tempMat(:,1)-0.5;

robotXY = RobotStates;
robotXY(:,1) = RobotStates(:,2)-0.5;
robotXY(:,2) = height+1-RobotStates(:,1)-0.5;

goalXY = GoalStates;
goalXY(:,1) = GoalStates(:,2)-0.5;
goalXY(:,2) = height+1-GoalStates(:,1)-0.5;

plot(obstacles(:,1),obstacles(:,2),'square','MarkerEdgeColor','k','MarkerFaceColor',[0 0 0],'MarkerSize',obstacleMarkerSize);
hold on;


for i=1:robotNum    
    plot(goalXY(i,1),goalXY(i,2),'o','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[1 1 1],'MarkerSize',robotMarkerSize,'LineWidth',2);    
    %txt=sprintf('d_{%d}=%d',i,goalXY(i,3)-1);
    %text(goalXY(i,1),goalXY(i,2)+0.6,txt,'FontWeight','Bold','FontSize',fontSize-4,'HorizontalAlignment','center','Color',[0 0 0]);        
    hold on;
end

for i=1:robotNum
    plot(robotXY(i,1),robotXY(i,2),'o','MarkerEdgeColor',ColorMat(i,:),'MarkerFaceColor',ColorMat(i,:),'MarkerSize',robotMarkerSize,'LineWidth',2);   
    if ArrivalTime(i,1)<=simT
                
        switch objectiveSelect
            case 0
                txt=sprintf('τ_{%d}=%d',i,ArrivalTime(i,1)-1);
            case 1
                txt=sprintf('τ_{%d}=%d',i,ArrivalTime(i,1)-1);
            case 2
                txt=sprintf('τ_{%d}=%d',i,ArrivalTime(i,1)-1);
            case 5
                txt=sprintf('L_{%d}=%d',i,ArrivalTime(i,1)-goalXY(i,3));
            case 6
                txt=sprintf('T_{%d}=%d',i,abs(ArrivalTime(i,1)-goalXY(i,3)));
            case 7
                if ArrivalTime(i,1)==goalXY(i,3)+1
                    temp = 1;
                else
                    temp = 0;
                end
                txt=sprintf('U_{%d}=%d',i,temp);
        end                
        %text(goalXY(i,1),goalXY(i,2)-0.6,txt,'FontWeight','Bold','FontSize',fontSize-4,'HorizontalAlignment','center','Color',ColorMat(i,:)); 
    end
    txt=sprintf('%d',i);
    text(robotXY(i,1),robotXY(i,2),txt,'FontWeight','Bold','FontSize',fontSize,'HorizontalAlignment','center','Color',[1 1 1]);        
    hold on;
end

end

