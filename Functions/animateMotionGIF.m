function []=animateMotionGIF(motion,qGoal,obstacles,dims,sampleRate,movieFileName,speedUp)

% The function animateMotionGIF generatesa a GIF of the motion.
%
% The syntax is
% []=animateMotionGIF(motion,qGoal,obstacles,dims,sampleRate,movieFileName,speedUp) 
%
% There are no function outputs.
%
% The function arguments are:
%   - motion: Time series collection containing the states and inputs of
%   the motion.
%   - qGoal: 6x1 vector of the goal state.
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
%   - sampleRate: Double, rate of resampling of the timeseries cllection to
%   use in the figure.
%   - movieFileName: String, name to be used in the GIF file.
%   - speedUp: Double, rate at which the GIF motion will be sped up with
%   respect to the real motion speed.


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

N=length(x);
M(N) = struct('cdata',[],'colormap',[]);

xLimits=[min(x)-7 max(x)+7];
yLimits=[min(y)-7 max(y)+7];

fprintf('Creating GIF...\n')

for k=1:1:N+10
    if k>N
        k=N;
    end
    
    %% Create the figure in which to plot the path
    f=figure(); %Create a new figure
    set(f,'Visible','off'); %Toggle the display of the figure off
    set(f,'InnerPosition',[0 0 1920 1080]); %Set the figures inner size
    hold on %Set hold to on to overlap the different plots
    daspect([1 1 1]) %Set the aspect ratio so that x and y units have the same length
    xlim(xLimits) %Set the x axis limits
    ylim(yLimits) %Set the y axis limits
    title(sprintf('Animated Motion (%.1f x Original Speed)',speedUp))

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
    hold on

    %% Plot the goal state in green
    plotCar(qGoal(:,end),dims,'g','-')
    hold on

    %% Plot the position of the vehicle along the path in blue
    if Q(4,k)>=0
        plotCar(Q(:,k),dims,'b','-');
    else
        plotCar(Q(:,k),dims,'r','-');
    end
    
    % Update the frame and save it
    drawnow
    frame = getframe(f);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if k == 1
        imwrite(imind,cm,movieFileName,'gif', 'Loopcount',inf,'DelayTime',sampleRate/speedUp);
    else
        imwrite(imind,cm,movieFileName,'gif','WriteMode','append','DelayTime',sampleRate/speedUp);
    end

    hold off
    close(f)

    if rem(k,floor(N/20))==0
        fprintf('Creating GIF... %.2f%% done.\n',k/N*100)
    end
end

% Print the progress information
fprintf('Creating GIF... %.2f%% done.\n',100)
fprintf('GIF saved to file %s.\n',movieFileName)

end


