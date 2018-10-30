function []=plotPath(motion,WP,obstacles,dims,location,sampleRate)

% The function plotPath plots the outline of a vehicle along a motion.
%
% The syntax is []=plotPath(motion,WP,obstacles,dims,location,sampleRate).
%
% There are no function outputs.
%
% The function arguments are:
%   - motion: Time series collection containing the states and inputs of
%   the motion.
%   - WP: Waypoints to follow throughout the motion.
%   - obstacles: ~x6 array that contains information on the environment
%   obstacles. Each row of the array is an obstacle. The first column
%   specifies the geometry of the obstacle: 1 for a rectangle, 2 for a
%   circle. The second and third columns specify the x and y location of
%   the center of the obstacle in meters. If the obstacle is of type 1, the
%   fourth column is the orientation of the obstacle in radians, measured
%   between the local and global x axis, positive counterclockwise. The
%   fifth and sixth columns of a type 1 obstacle are the thickness of the
%   obstacles along its local x and y axis respectively. For obstacles of
%   type 2, the fourth column indicates the radius. The other columns do
%   not indicate anything and can be leaved as zeros.
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.
%   - location: String defining the desired location for the figure created
%   by the function.
%   - sampleRate: Double, rate of resampling of the timeseries cllection to
%   use in the figure.


%% Initialization

% Obtain the states from the time series collection
motionRS=resample(motion,0:sampleRate:(motion.Time(end)));
x=motionRS.x.Data(1,:);
y=motionRS.y.Data(1,:);
psi=motionRS.psi.Data(1,:);
v=motionRS.v.Data(1,:);
delta=motionRS.delta.Data(1,:);
a=motionRS.a.Data(1,:);
Q=[x;y;psi;v;delta;a];

% Create the figure in which to plot the path
figure() %Create a new figure
hold on %Set hold to on to overlap the different plots
grid on %Set the grid markings on
daspect([1 1 1]) %Set the aspect ratio so that x and y units have the same length
movegui(location) %Change the location of the figure


%% Plot the obstacles in grey
for i=1:size(obstacles,1)
    if obstacles(i,1)==0
        continue
    elseif obstacles(i,1)==1
        xVertices=obstacles(i,2)+obstacles(i,5)/2*[-1 1 1 -1];
        yVertices=obstacles(i,3)+obstacles(i,6)/2*[-1 -1 1 1];
        h=patch(xVertices,yVertices,0.75*[1 1 1]);
        rotate(h,[0;0;1],rad2deg(obstacles(i,4)),[obstacles(i,2:3) 0]);
    elseif obstacles(i,1)==2
        position=[obstacles(i,2)-obstacles(i,4) obstacles(i,3)-obstacles(i,4)...
                  obstacles(i,4)*2 obstacles(i,4)*2];
        curvature=[1 1];
        rectangle('Position',position,'Curvature',curvature,'FaceColor',0.75*[1 1 1]);
    end
end


%% Plot the path
for n=1:(size(Q,2))
    if Q(4,n)>=0
        plotCarOutline(Q(:,n),dims,'b','-')
    else
        plotCarOutline(Q(:,n),dims,'r','-')
    end
end

%% Plot the qRef in green
for wp=1:size(WP,2)
    if (WP(7,wp)==1)
        plotCarOutline(WP(1:6,wp),dims,'g','-')
    elseif (WP(7,wp)==2 || WP(7,wp)==3)
        plot(WP(1,wp),WP(2,wp),'go')
    end
end

% Update the figure
drawnow

end

