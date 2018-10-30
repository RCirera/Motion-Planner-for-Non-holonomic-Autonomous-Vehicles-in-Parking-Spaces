function [obsList] = obstacleIDs(nodes,obstacles,margin)

% The function obstacleIDs creates the obstacle list.
%
% The syntax is [obsList] = obstacleIDs(nodes,obstacles,margin)
%
% The function outputs are:
%   - obsList: 1x~ array containing the IDs of the nodes that are too close
%   to obstacles.
%
% The function arguments are:
%   - nodes: 1xn struct with 4 fields. n is the number of nodes created.
%   The 4 fields are ID, Coords, Neighbours and Parent. ID is an integer,
%   the identification number of the node. Coords is a 2x1 vector
%   containing the x and y position of the node. Neighbours is a vector
%   containing the IDs of the neighbours of each node. Finally, Parent is
%   an array that contains the ID of the parent node. Only nodes opened
%   during A* get a parent assigned, so the field might be empty.
%   - obstacles: ~x6 array that contains information on the environment
%   obstacles. Each row of the array is an obstacle. The first column
%   specifies the geometry of the obstacle: 1 for a rectangle, 2 for a
%   circle. The second and third columns specify the x and y location of
%   the center of the obstacle in meters. If the obstacle is of type 1, the
%   fourth column is the orientation of the obstacle in radians, measured
%   between the local and global x axis, positive counterclockwise. The
%   fifth and sixth columns of a type 1 obstacle are the thickness of the
%   obstacles along its local x and y axis respectively. For obstacles of
%   type 2, the fourth column indicates the radius. The other columns do
%   not indicate anything and can be leaved as zeros.
%   - margin: Double, distance from an obstacle perimeter up until wich a
%   node is still included in the obstacle list. 


%% Determine the obstacle list

obsList=[]; %Initiate the obstacle list

for n=1:numel(nodes) %For every node
    
    for obs=1:size(obstacles,1) %For every obstacle
        if obstacles(obs,1)==0 %Skip deleted obstacles
            continue
        
        elseif obstacles(obs,1)==1
            % Error vector between node and obstacle centre
            error=nodes(n).Coords-obstacles(obs,2:3).';
            % Define the rotation matrix
            R=[cos(-obstacles(obs,4)) -sin(-obstacles(obs,4));
               sin(-obstacles(obs,4))  cos(-obstacles(obs,4))];
            % Rotation into the frame of reference of the obstacle
            error=R*error;
            % Normalized distance wrt obstacles axis
            xDist=abs(error(1));
            yDist=abs(error(2));
            % Collision test
            if (xDist<(obstacles(obs,5)/2+margin) && yDist<(obstacles(obs,6)/2+margin)) %The node is in a collision
                obsList=[obsList nodes(n).ID]; %Add the node to the obstacle list
                break %Skip the rest of the obstacles and go to the next node
            end
            
        elseif obstacles(obs,1)==2
            % Error vector between node and obstacle centre
            error=norm(nodes(n).Coords-obstacles(obs,2:3).');
            if (error<(obstacles(obs,4)+margin)) %The node is in a collision
                obsList=[obsList nodes(n).ID]; %Add the node to the obstacle list
                break %Skip the rest of the obstacles and go to the next node
            end
        end
    end
end
                      
end

