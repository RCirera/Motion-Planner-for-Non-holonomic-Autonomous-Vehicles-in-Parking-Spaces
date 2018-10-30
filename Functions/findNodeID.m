function [ID] = findNodeID(nodes,Coords,obsList)

% The function findNodeID finds the ID of the closest node in nodes to
% the point with coordinates Coords that is not listed in obsList.
%
% The syntax is [ID] = findNodeID(nodes,Coords,obsList).
%
% The function outputs are:
%   - ID: Identification number of the closest node to Coords.
%
% The function arguments are:
%   - nodes: 1xn struct with 4 fields. n is the number of nodes created.
%   The 4 fields are ID, Coords, Neighbours and Parent. ID is an integer,
%   the identification number of the node. Coords is a 2x1 vector
%   containing the x and y position of the node. Neighbours is a vector
%   containing the IDs of the neighbours of each node. Finally, Parent is
%   an array that contains the ID of the parent node. Only nodes opened
%   during A* get a parent assigned, so the field might be empty.
%   - Coords: 2x1 vector containing the x and y coordinates of a point.
%   - obsList: 1x~ array containing the IDs of nodes marked as obstacles.


%% Find the closest node

minError=Inf; %Define the current minimum error
minID=[]; %Define the ID of the current closest node

% For all the nodes
for n=1:numel(nodes)
    % Skip nodes that are in the obstacle list
    if (ismember(nodes(n).ID,obsList))
        continue
    end
    % Calculate the position error from Coords to the node n
    error=norm(Coords(1:2)-nodes(n).Coords);
    % If the error is smaller than the current minimum error
    if error<minError
        minError=error; %Update the minimum error
        minID=nodes(n).ID; %Update the ID of the current closest node
    end
end

% Define the output
ID=minID;

end