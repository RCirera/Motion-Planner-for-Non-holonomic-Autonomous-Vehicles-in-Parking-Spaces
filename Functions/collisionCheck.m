function collision=collisionCheck(U,q0,obstacles,N,dT,dt,dims)

% The function collisionCheck determines whether a collision occurs.
%
% The syntax is collision=collisionCheck(U,q0,obstacles,N,dT,dt,dims)
%
% The function outputs are:
%   - collision: boolean. 1 if a collision occurs, 0 otherwise.
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


%% Determine the occurence of a collision

J=collisionCost(U,q0,obstacles,N,dT,dt,dims); %Calculate the collision cost

% If the collision cost is larger than 0 a collisino occurs
if J>0
    collision=1;
else
    collision=0;
end

end