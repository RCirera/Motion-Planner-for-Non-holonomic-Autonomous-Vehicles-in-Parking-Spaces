function J=obsCost(x)

% The function obsCost computes the cost of a point situated at a
% normalized distance x from the center of an obstacle. The normalized
% distance is equal to the absolute value of the distance over the thicknes
% of the obstacle.
%
% The syntax is J=obsCost(x)
%
% The function outputs are:
%   - J: Double. Cost. Zero if the point is outside the obstacle (x>1). If 
%   the point is inside the obstacle (0<=x<=1) the cost is given by 
%   (-100*x+200).
%
% The funciton arguments are:
%   - x: Double. Normalized distance, larger or equal to 0.


%% Compute the cost
J=(x<=1).*(-100*x+200);

end

