function qkp1=vehicleDT(qk,uk,dT,dt,l,ta,td)

% The function vehicleDT simulates a time step of the system using a first
% order forward difference scheme.
%
% The syntax is qkp1=vehicleDT(qk,uk,Ts,dt,l,ta,td)
%
% The function outputs are:
%   - qkp1: 6x1 vector containing q(k+1), the state q at time step k+1.
%
% The function arguments are:
%   - qk: 6x1 vector containing the state of the simulation at time step k.
%   The first two rows contain the position of the center of the rear axis
%   of the vehicle on the plane in meters. The third row contains the
%   orientation of the vehicle in radians. The fourth is the vehicle
%   velocity in m/s. Finally, the fifth and sixth rows contain the steering
%   angle in rad and the acceleration in m/(s^2), respectively.
%   - uk: 2x1 vector containing the control history at time step k. The
%   first row contains the reference steering angle in radians. The second
%   row contains the reference acceleration in m/(s^2).
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

%% Recursive state trajectory calculation

% For every time step subdivision
for m=1:round(dT/dt)
    % Calculate the state values at the next subdivision with a first order
    % forward difference scheme
    qk=qk+dt*vehicleCT(qk,uk,l,ta,td);
end

% Pass the last time step subdivision state to the output
qkp1=qk;
end