function [nodes] = getNodes(parkingDims,dx,dy,order)

% The function getNodes creates uniformely distributed nodes on a limited
% rectangular area.
%
% The syntax is [nodes] = getNodes(parkingDims,dx,dy,order).
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
%   - parkingDims: 1x4 array containing limits on x and y in the parking
%   area. The elements are doubles, and in the following order
%   [xMin,xMax,yMin,yMax].
%   - dx: Double, distance between node along the x axis.
%   - dy: Double, distance between node along the y axis.
%   - order: Integer, maximum amount of one-node steps that can be taken
%   between nodes that are still considered neighbours. Diagonal steps are
%   considered one-node steps.


%% Create the node coordinates

xVector=parkingDims(1):dx:parkingDims(2); %Vector containing the range of x valuse for the nodes
yVector=parkingDims(3):dy:parkingDims(4); %Vector containing the range of y valuse for the nodes
[X,Y]=ndgrid(xVector,yVector); %Matrices containing the coordinates x and y
X=X.'; %Flip the matrix X so the coordinates match the matrix positions
Y=flipud(Y.'); %Flip the matrix Y so the coordinates match the matrix positions


%% Create the nodes as structures. 

for i=numel(X):-1:1
    node.ID=i; %Node ID
    node.Coords=[X(i);Y(i)]; %Node coordinates
    node.Neighbours=neighbours(X,i,order); %Neighbours (reachable nodes)
    node.Parent=[]; %Node parent (tbd)
    nodes(i)=node; %Add node to output
end

end

