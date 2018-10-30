function [c,ceq]=consFunctionObsCostU3(U,q0,N,dT,dt,qBounds,dims,obstacles,costBound,costfun)

% The function consFunctionObsU3 computes the nonlinear constraint matrices c
% and ceq to be used by fmincon.
%
% The syntax is
% [c,ceq]=consFunctionObsCostU3(U,q0,N,dT,dt,qBounds,dims,obstacles,costBound,costfun) 
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
%   - N: Integer. Number of time steps of the motion. The input remains
%   constant during a time step.
%   - dT: Double, size in seconds of each time step.
%   - dt: Double, size in seconds of the subdivisions of the simulation
%   time steps. The system equation is recomputed every dt seconds to
%   simulate the system. dT must be an integer multiple of dt.
%   - qBounds: 6x1 vector containing maximum values for certain parameters to
%   ensure passenger comfort. In order, the elements are maximum
%   longitudinal acceleration, minimum longitudinal acceleration (maximum
%   deceleration/braking), maximum longitudinal jerk, maximum lateral
%   acceleration, maximum lateral jerk, maximum longitudinal velocity and
%   minimum longitudinal velocity. The units are m/s for velocities, m/s^2
%   for accelerations and m/s^3 for jerks. 
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.
%   - obstacles: Array that contains information on the environment
%   obstacles.
%   - costBound: Objective bound for the cost function.
%   - costfun: Function handle of the cost funciton used together with this
%   constraint function in fmincon.


%% Initialization

% Simulate the motion with input U and initial state q0
Qdt=vehicleSim_dt(U,q0,N,dT,dt,dims(1),dims(6),dims(7));

% Calculate an obstacle collision cost
JObs=collisionCost(U,q0,obstacles,N,dT,dt,dims);

% Calculate the final cost
cost=costfun(U);

% Define a tolerance for the collision cost
obsTol=0.1;

% Calculate the comfort paramters
TdT=0:dT:dT*N;
Tdt=0:dt:dT*N;

v=Qdt(4,:);
delta=Qdt(5,:);
a=Qdt(6,:);

UNp1=[U zeros(2,1)];
aRefdT=timeseries(UNp1(2,:),TdT,'Name','aRef');
aRef=resample(aRefdT,Tdt,'zoh');
aRef=permute(aRef.Data,[3 2 1]).';

aLong=a; %Longitudinal acceleration
jLong=(aRef-a)./dims(6); %Longitudinal jerk
aLat=(v.^2)./(dims(1)./(tan(delta))); %Lateral acceleration
jLat=diff(aLat)/dt; %Lateral jerk


%% Inequality constraints, matrix c

% Impose bounds on the final state error and collision cost
c=[ JObs-obsTol... %Collision cost
    v-qBounds(6), -v+qBounds(7),... %Longitudinal velocity
    delta-qBounds(8), -delta+qBounds(9),... %Steering angle
    aLong-qBounds(1), -aLong+qBounds(2),... %Longitudinal acceleration 
    abs(jLong)-qBounds(3),... %Longitudinal jerk
    abs(aLat)-qBounds(4),... %Lateral acceleration
    abs(jLat)-qBounds(5),... %Lateral jerk
    cost-costBound]; %Final cost


%% Equality Constraints, matrix ceq

% No equality constraints
ceq=[];

end