function J=costFunction(U,q0,WP,N,dT,dt,l,ta,td)

% The function costFunction computes a cost J, the minimization of which
% will result in a motion that attempts to follow a series of waypoints WP.
% The motion begins at initial state q0 and has control history U.
%
% The syntax is J=costFunction(U,q0,WP,N,dT,dt,l,ta,td)
%
% The function outputs are:
%   - J: Double, value of the cost function. The cost is the sum of the
%   individual cost of every WP. The individual cost of a WP is the minimum
%   of the norms of the errors of every state wrt said WP.
%
% The funciton arguments are:
%   - U: 2xN array, control history. The elements in the array are the
%   optimization variables.
%   - q0: 6x1 vector containing the initial state of the motion. The
%   first two rows contain the position of the center of the rear axis of
%   the vehicle on the plane in meters. The third row contains the
%   orientation of the vehicle in radians. The fourth is the vehicle
%   velocity in m/s. Finally, the fifth and sixth rows contain the steering
%   angle in rad and the acceleration in m/(s^2), respectively.
%   - WP: 7D vector array, waypoints to follow throughout the motion. Every
%   column corresponds to a waypoint. The first six rows of a column are
%   the states of the WP. The seventh row defines the type of WP. Type 1 WP
%   consider the error in all states but the steering angle. Type 2
%   consider error in position only. Type 3 consider position and
%   orientation error. 
%   - N: Integer. Number of time steps of the motion. The input remains
%   constant during a time step.
%   - dT: Double, size in seconds of each time step.
%   - dt: Double, size in seconds of the subdivisions of the simulation
%   time steps. The system equation is recomputed every dt seconds to
%   simulate the system. dT must be an integer multiple of dt.
%   - l: Double, vehicle's inter-axis distance.
%   - ta: Double, time constant of the first order dynamics of the
%   reference acceleration input of the vehicle model.
%   - td: Double, time constant of the first order dynamics of the
%   reference steering angle input of the vehicle model.

%% Initialization

% Determine the path of the vehicle for the given input history U
Q=vehicleSim(U,q0,N,dT,dt,l,ta,td);

%% Cost computation

% Calculate the error if WP i is of type 1
if WP(7,end)==1
    errorWP=Q([1:4 6],end)-WP([1:4 6],end);
    % Map the error to the interval [-pi pi]
    errorWP(3,:)=wrapToPi(errorWP(3,:));

% Calculate the error if WP i is of type 2
elseif WP(7,end)==2
    errorWP=Q(1:2,end)-WP(1:2,end);

% Calculate the error if WP i is of type 3
elseif WP(7,end)==3
    errorWP=Q(1:3,end)-WP(1:3,end);
    % Map the error to the interval [-pi pi]
    errorWP(3,:)=wrapToPi(errorWP(3,:));
    
else
    errorWP=0;
end
        
% Calculate the total cost as the norm
J=norm(errorWP);
        
end
