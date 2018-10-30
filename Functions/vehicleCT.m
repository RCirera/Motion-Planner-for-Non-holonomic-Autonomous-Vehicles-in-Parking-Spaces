function dqdt=vehicleCT(q,u,l,ta,td)

% The function vehicleCT calculates the state derivative of the vehicle
% model dqdt for a given state q and input u.
%
% The syntax is dqdt=vehicleCT(qu,l,ta,td)
%
% The function outputs are:
%   - dqdt: 6x1 vector containing q(k+1), the state q at time step k+1.
%
% The function arguments are:
%   - q: 6x1 vector containing the state of the simulation. The first two
%   rows contain the position of the center of the rear axis of the vehicle
%   on the plane in meters. The third row contains the orientation of the
%   vehicle in radians. The fourth is the vehicle velocity in m/s. Finally,
%   the fifth and sixth rows contain the steering angle in rad and the
%   acceleration in m/(s^2), respectively. 
%   - u: 2x1 vector containing the control input. The first row contains
%   the reference steering angle in radians. The second row contains the
%   reference acceleration in m/(s^2). 
%   - l: Double, vehicle's inter-axis distance.
%   - ta: Double, time constant of the first order dynamics of the
%   reference acceleration input of the vehicle model.
%   - td: Double, time constant of the first order dynamics of the
%   reference steering angle input of the vehicle model.


%% Calculate the state derivative

dqdt=[q(4)*cos(q(3)); %dx/dt
      q(4)*sin(q(3)); %dy/dt
      q(4)/l*tan(q(5)); %dpsi/dt
      q(6); %dv/dt
      (u(1)-q(5))/td; %ddelta/dt
      (u(2)-q(6))/ta]; %da/dt
  
end