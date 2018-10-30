function qPerimeter=vehiclePerimeterPoints(q,dims)

% The funciton qPerimeter calculates a series of points situated on the
% parimeter of a series of vehicle states.
%
% The syntax is qPerimeter=vehiclePerimeterPoints(q,dims)
%
% The function outputs are:
%   - qPerimeter: Matrix of column vectors that contain the positions of
%   points in the perimeter of the vehicle. 
%
% The funciton arguments are:
%   - q: 6x~ matrix containing states of the vehicle in the columns. The
%   first two rows contain the position of the center of the rear axis of
%   the vehicle on the plane in meters. The third row contains the
%   orientation of the vehicle in radians. The fourth is the vehicle
%   velocity in m/s. Finally, the fifth and sixth rows contain the steering
%   angle in rad and the acceleration in m/(s^2), respectively. 
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.
   

%% Calculate the perimeter point locations

% Determine the number of points along the length of the vehicle
lengthPoints=1+floor(dims(2)/dims(5));

% Determine the number of points along the width of the vehicle
widthPoints=1+floor(dims(3)/dims(5));

% Determine the distance between points along the length of the vehicle
dL=dims(2)/lengthPoints;

% Determine the distance between points along the width of the vehicle
dW=dims(3)/widthPoints;

% Determine the position of the points wrt the center of the rear axis
x=[0:dL:dims(2)]-dims(4);
y=[0:dW:dims(3)]-dims(3)/2;
statesU=[x;
         y(end)*ones(size(x))];
statesD=[x;
         y(1)*ones(size(x))];
statesL=[x(1)*ones(size(y(2:end-1)));
         y(2:end-1)];
statesR=[x(end)*ones(size(y(2:end-1)));
         y(2:end-1)];
states=[statesU fliplr(statesR) fliplr(statesD) statesL];

% Add a third dimension to the positions to perform the rotation
states=[states; ones(size(states(1,:)))];

% Create the matrix qPerimeter
qPerimeter=zeros(2,size(q,2)*size(states,2));

% For every vector in q find the translated and rotated positions
for j=1:size(q,2)  
    
    % Rotate the points around the rear axis centre by the orientation angle
    R=[cos(q(3,j)) -sin(q(3,j)) q(1,j);
       sin(q(3,j))  cos(q(3,j)) q(2,j);
                 0            0      1];
    statesi=R*states;
    
    % Add the perimeter points to qPerimeter
    qPerimeter(1:2,1+(j-1)*size(states,2):j*size(states,2))=statesi(1:2,:);
end

end