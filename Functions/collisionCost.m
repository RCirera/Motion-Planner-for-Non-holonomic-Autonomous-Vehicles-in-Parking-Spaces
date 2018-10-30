function J=collisionCost(U,q0,obstacles,N,dT,dt,dims)

% The function collisionCost computes a collision cost for a motion
% determined by the initial state q0 and control history U.
%
% The syntax is J=collisionCost(U,q0,obstacles,N,dT,dt,dims)
%
% The function outputs are:
%   - J: Double, value of the cost function. The cost is the sum of the
%   individual cost of every obstacle.
%
% The funciton arguments are:
%   - U: 2xN array, control history.
%   - q0: 6x1 vector containing the initial state of the motion. The
%   first two rows contain the position of the center of the rear axis of
%   the vehicle on the plane in meters. The third row contains the
%   orientation of the vehicle in radians. The fourth is the vehicle
%   velocity in m/s. Finally, the fifth and sixth rows contain the steering
%   angle in rad and the acceleration in m/(s^2), respectively.
%   - obstacles: Array that contains information on the environment
%   obstacles.
%   - N: Integer. Number of time steps of the motion. The input remains
%   constant during a time step.
%   - dT: Double, size in seconds of each time step.
%   - dt: Double, size in seconds of the subdivisions of the simulation
%   time steps. The system equation is recomputed every dt seconds to
%   simulate the system. dT must be an integer multiple of dt.
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.


%% Initialization

% Obtain the states of the vehicle for every simulation step dt
q=vehicleSim_dt(U,q0,N,dT,dt,dims(1),dims(6),dims(7));

% Obtain the location of the points on the perimeter of the vehicle that
% will be checked for collision
qPerimeter=vehiclePerimeterPoints(q,dims);

% Create the array that will hold the individual objects cost
J_i=zeros(size(obstacles,1),1);


%% Calculate the cost that is incurred by every individual obstacle

% For every obstacle
for i=1:size(obstacles,1)
    
    % Skip deleted obstacles
    if obstacles(i,1)==0
        continue
    end
    
    % Skip obstacles too far from the path to possibly collide
    distance=q(1:2,:)-obstacles(i,2:3).'; %Vectors between the center of the obstacle and the states
    minDist=min(sqrt(sum(distance.*distance))); %Minimum euclidean distance between the center of the obstacle and the state
    colDist=norm(dims(2:3))+norm(obstacles(i,5:6))/2; %Collision safe distance
    
    if minDist>colDist
        continue
    end
    
    % Rectangular obstacles
    if obstacles(i,1)==1
        
        % Transform the states in q to the frame of reference of the
        % rectangular obstacle:
        
        % Translation
        q_rec=[qPerimeter(1:2,:)-obstacles(i,2:3).';
                         ones(1,size(qPerimeter,2))];
        % Define the rotation matrix
        R=[cos(-obstacles(i,4)) -sin(-obstacles(i,4)) 0;
           sin(-obstacles(i,4))  cos(-obstacles(i,4)) 0;
                              0                     0 1];
        % Rotation
        q_rec=R*q_rec;
        
        % Distances wrt axis normalized wrt size
        xDist_i=abs(q_rec(1,:)/((obstacles(i,5)/2)+dims(10)));
        yDist_i=abs(q_rec(2,:)/((obstacles(i,6)/2)+dims(10)));

        % Cost
        xCost_i=obsCost(xDist_i(xDist_i<=1 | yDist_i<=1));
        yCost_i=obsCost(yDist_i(xDist_i<=1 | yDist_i<=1));
        J_i(i,1)=sum(min(xCost_i,yCost_i));
        
    % Circular obstacles
    elseif obstacles(i,1)==2
        
        % Distances wrt circular obstacle normalized wrt radius
        dist_i=(sqrt(sum((q(1:2,:)-obstacles(i,2:3).').^2)))./(obstacles(i,4)+dims(10));
        
        % Cost
        J_i(i,1)=sum(obsCost(dist_i(dist_i<=1)));
    end
end

% Add the individual costs
J=sum(J_i);

end
