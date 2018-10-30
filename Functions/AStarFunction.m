function [path] = AStarFunction(startCoords,goalCoords,nodes,obsList)

% The function AStarFunction uses the A* search algorithm to find a path
% from startCoords to goalCoords. The path connects the members of nodes
% and avoids the obstacles in obsList.
%
% The syntax is [path] = AStarFunction(startCoords,goalCoords,nodes,obsList).
%
% The function outputs are:
%   - nodes: 1xn struct with 4 fields. n is the number of nodes created.
%   The 4 fields are ID, Coords, Neighbours and Parent. ID is an integer,
%   the identification number of the node. Coords is a 2x1 vector
%   containing the x and y position of the node. Neighbours is a vector
%   containing the IDs of the neighbours of each node. Finally, Parent is
%   an array that contains the ID of the parent node. Only nodes opened
%   during A* get a parent assigned, so the field might be empty.
%
% The function arguments are:
%   - startCoords: 2x1 vector containing the x and y coordinates of the
%   starting position. It does not have to coincide with the position of a
%   node.
%   - goalCoords: 2x1 vector containing the x and y coordinates of the
%   goal position. It does not have to coincide with the position of a
%   node.
%   - nodes: 1xn struct with 4 fields. n is the number of nodes created.
%   The 4 fields are ID, Coords, Neighbours and Parent. ID is an integer,
%   the identification number of the node. Coords is a 2x1 vector
%   containing the x and y position of the node. Neighbours is a vector
%   containing the IDs of the neighbours of each node. Finally, Parent is
%   an array that contains the ID of the parent node. Only nodes opened
%   during A* get a parent assigned, so the field might be empty.
%   - obsList: 1x~ array containing the IDs of nodes marked as obstacles.


%% A* Set up

% Set up the beginning of the A* algorithm
startID=findNodeID(nodes,startCoords,obsList); %Find the ID of the starting node
curNode=nodes(startID); %Set the current node structure

goalID=findNodeID(nodes,goalCoords,obsList); %Find the ID of the goal node

closedList=[]; %Initialize the closed list
openList=[startID]; %Initialize the open list

feasiblePath=1; %Initialize the boolean variable feasiblePath

fCost=ones(numel(nodes),1)*100; %Initialize the fCost list
gCost=ones(numel(nodes),1); %Initialize the gCost list
hCost=zeros(numel(nodes),1); %Initialize the hCost list
for i=1:numel(nodes)
    hCost(i)=norm(nodes(goalID).Coords-nodes(i).Coords); %Fill the hCost list with the eucledian distance ot the goal
end

% Initialize the gCost,hCost and fCoat for the start node
gCost(startID,1)=0; %Is 0 because it is the start goal
hCost(startID,1)=hCost(startID,1); %Does not change
fCost(startID,1)=gCost(startID,1)+hCost(startID,1); %Sum of gCost and hCost


%% Main A* loop

while(curNode.ID~=goalID)

    curID=curNode.ID; %Get the ID of the current node
    
    openList=setdiff(openList,curID); %Remove the current node ID from the open list
    closedList=[closedList curID]; %Add the current node ID to the closed list
    
    if (curID==goalID) %Break the loop if current node is goal node
        continue
    end
    
    for n=1:1:numel(curNode.Neighbours) %For every neighbour of the current node
        neighbourID=curNode.Neighbours(n); %Get the ID of the neighbour
        temp_gCost=gCost(curID)+norm(curNode.Coords-nodes(neighbourID).Coords); %Calculate the gCost of the neighbour from the current node. It is temporal because if it is greater than   
        
        if (ismember(neighbourID,obsList)) %If the neighbour is in the obstacle list skip it
            continue
        elseif (((ismember(neighbourID,closedList))||(ismember(neighbourID,openList)))&&(temp_gCost>gCost(neighbourID))) %If the neighbour is (in the closed list or open list) and the gCost is larger (equivalent to fCost smaller) skip neighbour
            continue
        else %Else (((in closedList or openList) and smaller new cost) or not(in openList or in noList))
            openList=[openList neighbourID]; %Add the neighbour ot the open list
            gCost(neighbourID)=temp_gCost; %Update the gCost of the neighbour
            fCost(neighbourID)=gCost(neighbourID)+hCost(neighbourID); %Update the fCost of the neighbour
            nodes(neighbourID).Parent=curID; %Update the parent node of the neighbour
        end
    end
    % If the open list is empty, no feasible path, end the A*
    if (numel(openList)==0)
        feasiblePath=0;
        break
    end
    % Choose the node with the minimum fCost as the next current node
    if (ismember(goalID,openList)) %If goal node is in open List (was one of the neighbours)
        curNode=nodes(goalID); %Choose goal node as next current node
    else
        [minfCost,~] = min(fCost(openList)); %Find the minimum fCost value
        [minfCostID] = find(fCost(openList)==minfCost); %Find the ID of the node corresponding to the minimum fCost
        curNode = nodes(openList(minfCostID(1))); %Select the minimum cost node in the open list
    end
end


if (feasiblePath==0)
    path=[zeros(6,1) Inf*ones(6,1)];
    return
end


%% Path Extraction

%After the goal has been found, the path needs to be reconstructed by
%backtracking the parent nodeIDs until the startnode ID is found
nParent = nodes(goalID).Parent; %Get thegoal parent node
nodeIDPath = [goalID]; %Add the goal node ID to the node ID path
% Find the parents of the nodes recursively until the start node
while(nParent ~= startID)
    nodeIDPath = [nParent nodeIDPath];
    nParent = nodes(nParent).Parent;
end

% This array contains a list of nodeID path
nodePath = [startID nodeIDPath];

%Using the nodeID path, get the x and y coordinates of the path
path=zeros(6,size(nodePath,2));
for pIdx=1:1:(length(nodePath))
    path(1:2,pIdx) = nodes(nodePath(pIdx)).Coords; 
end

% Get an approximation for the heading psi for all nodes but the last
for pIdx=1:1:(length(nodePath)-1)
    error=nodes(nodePath(pIdx+1)).Coords-nodes(nodePath(pIdx)).Coords;
    path(3,pIdx) = atan2(error(2),error(1));
end

% Get an approximation for the heading psi for the final node
error=goalCoords-nodes(nodePath(end)).Coords;
path(3,end)=atan2(error(2),error(1));

end

