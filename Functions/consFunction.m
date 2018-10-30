function [c,ceq]=consFunction(U,q0,N,dT,dt,l,ta,td,comf)

% The function consFunction computes the nonlinear constraint matrices c
% and ceq to be used by fmincon.
%
% The syntax is [c,ceq]=consFunction(u,q0,qRef,N,dT,dt,l,ta,td)
%
% The function outputs are:
%   - c: nonlinear inequality matrix. fmincon optimizes a cost function
%   such that c<=0.
%   - ceq: nonlinear equality matrix. fmincon optimizes a cost function
%   such that ceq==0.
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
%   reference steering angle input of the vehicle model.%% Inequality Constraints
%   - comf: 6x1 vector containing maximum values for certain parameters to
%   ensure passenger comfort. In order, the elements are maximum
%   longitudinal acceleration, minimum longitudinal acceleration (maximum
%   deceleration/braking), maximum longitudinal jerk, maximum lateral
%   acceleration, maximum lateral jerk, maximum longitudinal velocity and
%   minimum longitudinal velocity. The units are m/s for velocities, m/s^2
%   for accelerations and m/s^3 for jerks. 

%% Initialization

% Simulate the motion with input U and initial state q0
Q=vehicleSim(U,q0,N,dT,dt,l,ta,td);

% Calculate the comfort paramters

v=Q(4,:); %Longitudinal velocity
delta=Q(5,:); %Steering angle
aLong=Q(6,:); %Longitudinal acceleration
jLong=(U(2,:)-Q(6,1:end-1))./ta; %Longitudinal jerk
aLat=(Q(4,:).^2)./(l./(tan(Q(5,:)))); %Lateral acceleration
jLat=diff(aLat)/dT; %Lateral jerk

%% Inequality constraints, matrix c

% Impose bounds on the final state error and comfort parameters
c=[ v-comf(6), -v+comf(7),... %Longitudinal velocity
    delta-comf(8), -delta+comf(9),... %Steering angle
    aLong-comf(1), -aLong+comf(2),... %Longitudinal acceleration 
    abs(jLong)-comf(3),... %Longitudinal jerk
    abs(aLat)-comf(4),... %Lateral acceleration
    abs(jLat)-comf(5)]; %Lateral jerk

%% Equality Constraints, matrix ceq

% No equality constraints
ceq=[];

end