function Q=vehicleSim_dt(U,q0,N,dT,dt,l,ta,td)

% The function vehicleSim_dt simulates the system with initial state q0 and
% control history U with a first order forward difference scheme. The
% output Q includes the vehicle states at the regular time steps and at the
% time step simulation subdivision. 
%
% The syntax is Q=vehicleSim_dt(U,q0,N,dT,dt,l,ta,td)
%
% The function outputs are:
%   - Q: 6x(N*(dT/dt)+1) array containing the state trajectory of the simulation.
%   Every column of Q contains the state of the vehicle at a given time
%   step subdivision.
%
% The function arguments are:
%   - U: 2xN array containing the control history of the simulation. The
%   first row contains the reference steering angle in radians. The second
%   row contains the reference acceleration in m/(s^2). Column k contains
%   the inputs for time step k.
%   - q0: 6x1 vector containing the initial state of the simulation. The
%   first two rows contain the position of the center of the rear axis of
%   the vehicle on the plane in meters. The third row contains the
%   orientation of the vehicle in radians. The fourth is the vehicle
%   velocity in m/s. Finally, the fifth and sixth rows contain the steering
%   angle in rad and the acceleration in m/(s^2), respectively.
%   - N: Integer. Number of time steps of the simulation. The input remains
%   constant during a time step.
%   - dT: Double, size in seconds of each simulation time step. The input
%   remains constant during a time step.
%   - dt: Double, size in seconds of the subdivisions of the simulation
%   time steps. The system equation is recomputed every dt seconds to
%   symulate the system. dT must be an integer multiple of dt.
%   - l: Double, vehicle's inter-axis distance.
%   - ta: Double, time constant of the first order dynamics of the
%   reference acceleration input of the vehicle model.
%   - td: Double, time constant of the first order dynamics of the
%   reference steering angle input of the vehicle model.

%% Initialization

Q=zeros(size(q0,1),N*round(dT/dt)+1); %Create the array to store the state trajectory Q
Q(:,1)=q0; %Substitute the initial state q0 in the first column of Q

%% Recursive state trajectory calculation

% Calculate the amount of time step subdivisions per time step
M=round(dT/dt); 

% For every time step
for k=1:N
    %Compute the system function values using a first order forward
    %difference scheme
    Q(:,2+(M*(k-1)):1+(M*(k)))=vehicleDT_dt(Q(:,1+(M*(k-1))),U(:,k),M,dt,l,ta,td);
end