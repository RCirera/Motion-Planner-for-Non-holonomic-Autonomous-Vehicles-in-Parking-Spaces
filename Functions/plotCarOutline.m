function [] = plotCarOutline(q,dims,color,lineStyle)

% The function plotCarOutline plots the outline of a vehicle at state q.
%
% The syntax is [] = plotCarOutline(q,dims,color,lineStyle).
%
% There are no function outputs.
%
% The function arguments are:
%   - q: 6x1 vector containing the state of the vehicle.
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.
%   - color: String defining the color of the vehicle outline.
%   - lineStyle: String defining the style of the outline.


%% Plot the perimeter of the car at state q

xVertices=q(1)+dims(4)*[-1 -1 -1 -1]+dims(2)*[0 1 1 0];
yVertices=q(2)+dims(3)/2*[-1 -1 1 1];
per=patch(xVertices,yVertices,color);
per.FaceAlpha=0.1;
per.EdgeColor=color;
per.LineStyle=lineStyle;
if q(3)~=0
    rotate(per,[0 0 1],rad2deg(q(3)),[q(1:2);0]);
end

end

