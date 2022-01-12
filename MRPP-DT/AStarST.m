function OptimalPath=AStarST(MapMat,startState,goalState,constraints)
%Algorithm: space-time A*
%Author:  Hanfucius
%Input:   Occupancy Map matrix (zero position:upper left), start and goal row-colum
%actions: 1=right,2=up,3=left,4=down,0=wait
[height,width] = size(MapMat);
Actions = [0 1 1;-1 0 1;0 -1 1;1 0 1;0 0 1];       %right,up,left,down,pause,five actions
%pre-assign memory
OPEN_COUNT=1;
MAX_OPEN_SIZE=200;
OPEN=zeros(MAX_OPEN_SIZE,11);

CLOSED_COUNT=1;
MAX_CLOSED_SIZE=200;
CLOSED=zeros(MAX_CLOSED_SIZE,11);

%Initialize start node with FValue and open first node.
dist=pdist2(startState,goalState,'cityblock');
Root(1,1:11)=[startState(1,1) startState(1,2) 0 0 0 0 dist 0 dist 0 1]; %current ST node,parent ST node, F,G,H values,actions,checkedFlag (1=unchecked,0=checked)
OPEN(1,:)=Root;

while ~isempty(find(OPEN(:,11)==1, 1))
    %best-first search and tie-breaking policy
    TEMPOPEN = OPEN(OPEN(:,11)==1,:);
    minFScore=min(TEMPOPEN(:,7));
    index = find(TEMPOPEN(:,7)==minFScore);
    energy=TEMPOPEN(index,10);
    [~,index2] = min(energy,[],1);
    currentNodeIndex=index(index2);
    currentNode=TEMPOPEN(currentNodeIndex,:);
    if all(currentNode(1,1:2)==goalState)
        break;
    end
    
    %Removing node from OpenList to ClosedList
    [~,temp]=ismember(currentNode,OPEN,'rows');  
    OPEN(temp,11) = 0;
    CLOSED(CLOSED_COUNT,:) = currentNode;
    CLOSED_COUNT=CLOSED_COUNT+1;    
    for i=1:5
        newNode=zeros(1,11);
        newNode(1,1:3)=currentNode(1,1:3)+Actions(i,:);
        %Ignore the out of borders and obstacles.
        if newNode(1,1)<1||newNode(1,1)>height||newNode(1,2)<1||newNode(1,2)>width||MapMat(newNode(1,1),newNode(1,2))==1
            continue;
        end
                
        %Ignore the nodes in CLOSED lists.
        if ismember(newNode(1,1:3),CLOSED(:,1:3),'rows') 
            continue;
        end
        
        %Ignore the nodes in constraints lists.including vertex and edge
        %conflicts.
        currentST(1,1:3)=currentNode(1,1:3);
        nextST(1,1:3)=newNode(1,1:3);
        edgeConflict1=currentST;
        edgeConflict2=nextST;
        edgeConflict1(1,3)=nextST(1,3);
        edgeConflict2(1,3)=currentST(1,3);
        
        if ~isempty(constraints) 
            if ismember(nextST,constraints,'rows') ||(ismember(edgeConflict1,constraints,'rows') && ismember(edgeConflict2,constraints,'rows'))            
                continue;
            end
        end
                
        %Discover a new node and update its tentative F,G,H values
        newNode(1,4:6)=currentNode(1,1:3);
        dist=pdist2(newNode(1,1:2),goalState,'cityblock');
        newNode(1,7:9)=[dist+currentNode(1,8)+1 currentNode(1,8)+1 dist];
        if  i==5
            newNode(1,10)=currentNode(1,10);
        else
            newNode(1,10)=currentNode(1,10)+1;
        end
        newNode(1,11)=1;
        [flag,index]=ismember(newNode(1,1:3),OPEN(:,1:3),'rows');
        if  flag==1     % if this node is already in the OPEN list
            if newNode(1,7)>OPEN(index,7)
                continue;
            else
                OPEN(index,:)=newNode;
            end
        else   % if this node is not in the OPEN list
            OPEN_COUNT=OPEN_COUNT+1;
            OPEN(OPEN_COUNT,:)=newNode;
        end
    end       
end

%backward reconstruct path
k=1;
OptimalPath=zeros(200,3);
while ~all(Root==currentNode)
    OptimalPath(k,:)=currentNode(1,1:3);
    [~,parentIndex]=ismember(currentNode(1,4:6),CLOSED(:,1:3),'rows');
    currentNode = CLOSED(parentIndex,:);
    k=k+1;
end
OptimalPath=OptimalPath(all(OptimalPath,2),:);
OptimalPath=[OptimalPath;Root(1,1:3)];
OptimalPath=flipud(OptimalPath);
end