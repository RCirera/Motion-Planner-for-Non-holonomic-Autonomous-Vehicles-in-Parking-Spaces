function [curveLength] = curveLength(Q)

% The function curveLength computes the length of the curve defined by the
% first two states of the vectors in array Q.
%
% The syntax is [curveLength] = curveLength(Q).
%
% The function outputs are:
%   - curveLength: Double. Length of the curve defined by the first two
%   states of the vectors in array Q.
%
% The function arguments are:
%   - Q: mxn array containing n vectors of m states.


%% Calculate the curve length

curveLength=sum(sqrt(sum(diff(Q(1:2,:),1,2).^2)));

end

