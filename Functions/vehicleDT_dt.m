function Q=vehicleDT_dt(qk,uk,M,dt,l,ta,td)

% The function vehicleDT_dt simulates a time step of the system using a 
% first order forward difference scheme.
%
% The syntax is Q=vehicleDT_dt(qk,uk,M,dt,l,ta,td)
%
% The function outputs are:
%   - Q: 6xM vector containing the state q at the time step subdivisions of
%   time step k. 
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
%   - M: Integer, number of subdivisions of size dt in every time step.
%   - dt: Double, size in seconds of the subdivisions of the simulation
%   time steps. The system equation is recomputed every dt seconds to
%   symulate the system.
%   - l: Double, vehicle's inter-axis distance.
%   - ta: Double, time constant of the first order dynamics of the
%   reference acceleration input of the vehicle model.
%   - td: Double, time constant of the first order dynamics of the
%   reference steering angle input of the vehicle model.


%% Initialization 

Q=zeros(size(qk,1),M+1); %Create the array to store the output Q
Q(:,1)=qk; %Substitute the initial state qk in the first column of Q


%% Recursive state trajectory calculation

% For every time step subdivision
for k=1:M
    % Calculate the state values at the next subdivision with a first order
    % forward difference scheme
    Q(:,k+1)=Q(:,k)+dt*vehicleCT(Q(:,k),uk,l,ta,td);
end

% Remove the initial state qk from the output
Q=Q(:,2:end);

end