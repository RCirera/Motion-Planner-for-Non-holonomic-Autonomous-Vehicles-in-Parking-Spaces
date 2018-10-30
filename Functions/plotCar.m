function [] = plotCar(q,dims,color,lineStyle)

% The function plotCar plots the schematic of a vehicle at state q.
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
per.FaceAlpha=0;
per.EdgeColor=color;
per.LineStyle=lineStyle;
if q(3)~=0
    rotate(per,[0 0 1],rad2deg(q(3)),[q(1:2);0]);
end

%% Plot the wheels of the car at state q

% Back left wheel
xVertices=q(1)+(dims(8)/2)*[-1 1 1 -1];
yVertices=q(2)+(dims(3)/2-dims(9)/2+(dims(9)/2)*[-1 -1 1 1]);
bl=patch(xVertices,yVertices,color);
bl.FaceAlpha=0;
bl.EdgeColor=color;
bl.LineStyle=lineStyle;
if q(3)~=0
    rotate(bl,[0 0 1],rad2deg(q(3)),[q(1:2);0]);
end


% Back right wheel
xVertices=q(1)+(dims(8)/2)*[-1 1 1 -1];
yVertices=q(2)-(dims(3)/2-dims(9)/2+(dims(9)/2)*[-1 -1 1 1]);
br=patch(xVertices,yVertices,color);
br.FaceAlpha=0;
br.EdgeColor=color;
br.LineStyle=lineStyle;
if q(3)~=0
    rotate(br,[0 0 1],rad2deg(q(3)),[q(1:2);0]);
end


% Front left weheel
xVertices=q(1)+dims(1)+(dims(8)/2)*[-1 1 1 -1];
yVertices=q(2)+(dims(3)/2-dims(9)/2+(dims(9)/2)*[-1 -1 1 1]);
fl=patch(xVertices,yVertices,color);
fl.FaceAlpha=0;
fl.EdgeColor=color;
fl.LineStyle=lineStyle;
if q(5)~=0
    rotate(fl,[0 0 1],rad2deg(q(5)),[q(1)+dims(1);q(2)+dims(3)/2-dims(9)/2;0]);
end
if q(3)~=0
    rotate(fl,[0 0 1],rad2deg(q(3)),[q(1:2);0]);
end

% Front right weheel
xVertices=q(1)+dims(1)+(dims(8)/2)*[-1 1 1 -1];
yVertices=q(2)-((dims(3)/2-dims(9)/2+(dims(9)/2)*[-1 -1 1 1]));
fr=patch(xVertices,yVertices,color);
fr.FaceAlpha=0;
fr.EdgeColor=color;
fr.LineStyle=lineStyle;
if q(5)~=0
    rotate(fr,[0 0 1],rad2deg(q(5)),[q(1)+dims(1);q(2)-dims(3)/2+dims(9)/2;0]);
end
if q(3)~=0
    rotate(fr,[0 0 1],rad2deg(q(3)),[q(1:2);0]);
end


end

