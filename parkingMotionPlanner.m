% Author: Ricard Cirera Rocosa
% Affiliation: MSc. student at the Delft Center for Systems and Control
% department of the faculty of Mechanical, Maritime and Materials
% Engineering at the Delft University of Technology.  
% email: ricardcirera@gmail.com
% July 2018; Last revision: 06-August-2018

%% Begin the script

clear all
close all
clc
runTimeStartTime=tic; %Start the timer for the complete run time


%% Define the script parameters

% Vehicle model constants: SEAT Ibiza (delMax=deg2rad(37))
l=2.564; %Inter-axis distance
vl=4.059; %Vehicle length
vw=1.942; %Vehicle width (including rear view mirrors)
ve=0.699; %Distance from vehicle back bumper to back axis
vcol=0.1; %Minimum distance between points on the perimeter of the vehicle used to check for collisions
ta=2; %Time constant of the acceleration actuator
td=2; %Time constant of the steering actuator
wd=0.6; %Wheel diameter in meters
ww=0.2; %Wheel width in meters
obsMargin=0.1; %Minimum margin to maintain at all times between the vehicle and obstacles both in x and y directions
dims=[l,vl,vw,ve,vcol,ta,td,wd,ww,obsMargin].'; %Group the model parameters into an array

% Constraint function bounds for states and dynamic paramters 
aLongMax=1.0; %Maximum longitudinal acceleration in m/s^2
aLongMin=-aLongMax; %Minimum longitudinal acceleration (braking) in m/s^2
jLongMax=0.7; %Maximum longitudinal jerk in m/s^3
aLatMax=0.8; %Maximum lateral acceleration in m/s^2
jLatMax=0.3; %Maximum lateral jerk in m/s^3
vMax=5; %Maximum forward longitudinal velocity
vMin=-vMax; %Maximum backward longitudinal velocity
delMax=deg2rad(37); %Maximum steering angle
delMin=-delMax; %Minimum steering angle
qBounds=[aLongMax aLongMin jLongMax aLatMax jLatMax vMax vMin delMax delMin].'; %Group the parameters into an array
navqBounds=qBounds; %Specific bounds for the navigation motion
manqBounds=qBounds; %Specific bounds for the manoeuvre motion
manqBounds(6)=0; %Limit the manoeuvre motion to be backwards

% Set the upper and lower bounds for the vehicle model inputs
aRefMax=1.5; %Maximum reference acceleration
aRefMin=-aRefMax; %Minimum reference acceleration
delRefMax=deg2rad(45); %Maximum reference steering angle
delRefMin=-delRefMax; %Minimum reference steering angle
uBounds=[aRefMax aRefMin delRefMax delRefMin]; %Group the bounds into an array

% Parking parameters
pw=2.5; %Parking spot width
pl=5.0; %Parking spot length
pLine=3; %Number of spots continuously next to each other facing the ssame way
pTot=8*pLine; %Total number of parking spaces
parkingSpot=16; %Set the goal parking spot
lw=5.5; %Parking lane width

% A* parameters
dx=0.5; %Distance between nodes along the x axis
dy=dx; %Distance between nodes along the y axis
order=1; %Max node jumps between a node and its reachable nodes. A diagonal jump is equivalent in distance to a jump along the axis
obsNodeMargin=(lw-norm([dx dy]))/2; %Node margin wrt obstacles
navWPType=2; %Type of WPs for the navigation (type2 considers x and y for the cost)

% Manoeuvre WP parameters
dMan=dx; %Distance between manoeuvre points (before WP selection)
manWPType=3; %Type of WPs for the manoeuvre (type3 considers x, y and psi for the cost)

% Time parameters
dT=0.5; %Size of the input time steps
dt=0.1; %Size of the simulation time steps

% Navigation motion planning parameters
navdN=6; %Amount of input time steps per WP during navigation
navMMin=2; %Minimum number of WP considered at a time
navWPSteps=8; %Only 1 in every WPStepsNav WP are used for the navigation planning
navMMax=4; %Maximum number of WP considered at a time
navDispSteps=0; %Toggle the plotting of the local planning steps
navDispOptSteps=0; %Toggle the plotting of the individual optimization steps
navDetailedPlanning=0; %Toggle the use of dt as periodicity for constraint check
navCostBound=0.01; %Cost bound used in the navigation planning
algorithm='sqp'; %Set the algorithm to use in fmincon
navOptions = optimoptions(  'fmincon',... %Solver
                            'Algorithm',algorithm,... %Algorithm
                            'Display','off',... %Display type
                            'ObjectiveLimit',navCostBound,... %Cost function acceptable limit (sqp)
                            'OptimalityTolerance',1e-2); %1st order optimality tolerance
                            
% Manoeuvre motion planning parameters
mandN=4; %Amount of input time steps per WP during manouvering
manMMin=2; %Minimum number of WP considered at a time
manWPSteps=3; %Only 1 in every WPStepsMan WP are used for the manoeuvre planning
manMMax=7; %Maximum number of WP considered at a time
manDispSteps=0; %Toggle the plotting of the local planning steps
manDispOptSteps=0; %Toggle the plotting of the individual optimization steps
manDetailedPlanning=0; %Toggle the use of dt as periodicity for constraint check
manCostBound=0.01; %Cost bound used in the manoeuvre planning
algorithm='sqp'; %Set the algorithm to use in fmincon
manOptions = optimoptions(  'fmincon',... %Solver
                            'Algorithm',algorithm,... %Algorithm
                            'Display','off',... %Display type
                            'ObjectiveLimit',manCostBound,... %Cost function acceptable limit (sqp)
                            'OptimalityTolerance',1e-2); %1st order optimality tolerance

% Final Result Display Options
% Path plot parameters
showPath=1; %Toggle the creation of the path plot
savePath=0; %Toggle the saving of the path plot
pathBaseFilename=sprintf('pathParkingSpot%i',parkingSpot); %Name of the path plot file
pathFullFileName = fullfile(pwd,'Images\Paths',pathBaseFilename); %Full path plot file address

% Motion plot parameters
showMotion=1; %Toggle the creation of the motion plot
saveMotion=0; %Toggle the saving of the motion plot
motionBaseFilename=sprintf('motionParkingSpot%i',parkingSpot); %Name of the motion plot file
motionFullFileName = fullfile(pwd,'Images\Motions',motionBaseFilename); %Full motion plot file address

% Animation parameters
animate=1; %Toggle the creation of a GIF animation. WARNING: It is a time consuming task
GIFFileName=sprintf('GIFParkingSpot%i.gif',parkingSpot); %Name of the GIF file
GIFFullFileName=fullfile(pwd,'GIFs',GIFFileName); %Full GIF file address
speedUp=4; %The GIF will play speedUp times faster than the actual motion


%% Parameter checks and warnings

% Vehicle geometry constants 
rho=l/tan(delMax); %Minimum turn radius of the point representation of the vehicle

% Lane width parameter
rhoMaxFront=sqrt((rho+vw/2)^2+(vl-ve)^2); %Maximum turn radius of any point in the front of the vehicle (including obstacle distance margins)
rhoMaxBack=sqrt((rho+vw/2)^2+ve^2); %Maximum turn radius of any point in the back of the vehicle (including obstacle distance margins)
rhoMax=max(rhoMaxFront,rhoMaxBack); %Maximum turn radius of any point in the vehicle (including obstacle distance margins)
rhoMin=rho-vw/2; %Minimum turn radius of any point of the vehicle (including obstacle distance margins)
lwMinTurn=rhoMax-rhoMin*cos(pi/4)+2*obsMargin; %Minimum lane width required to turn around a corner

h1=(real(sqrt(rhoMin^2-(rhoMin-(pw-vw-2*obsMargin)/2)^2))); %Distance along the y axis from the entry of the parkign spot to a state from which the vehicle can exit the spot with a continuous minimum radius turn without colliding with the inside neighbour parking spot
h2=sign(rho+pw/2-obsMargin-rhoMaxBack)*(real(sqrt(rhoMaxBack^2-(rho+vw/2+(pw-vw-2*obsMargin)/2)^2))); %Distance along the y axis from the entry of the parkign spot to a state from which the vehicle can exit the spot with a continuous minimum radius turn without colliding with the outside neighbour parking spot
if (h1~=0 && h2~=0)
    h=min(h1,h2);
elseif h1==0
    h=h2;
elseif h2==0
    h=h1;
end
h=h-obsMargin; %Add the obstacle margin distance to h
lwMinManBack=rhoMaxFront-h+obsMargin; %Minimum lane width required to back into the parking spot in one motion
lwMin=max(lwMinManBack,lwMinTurn); %Minimum lane width required

% Lane width check
if lwMin-2*obsMargin>lw
    warning('Lane width, lw = %.2f, is smaller than the minimum required lane width to avoid obstacle collisions, lwMin = %.2f.',lw,lwMin-2*obsMargin);
elseif lwMin>lw
    warning('Lane width, lw = %.2f, is smaller than the minimum required lane width to respect obstacle margin constraints, lwMin = %.2f.',lw,lwMin);
elseif lw-lwMin<0.1
    warning('Lane width, lw = %.2f, and minimum required lane width to respect obstacle margin constraints, lwMin = %.2f, are close. This might result in obstacle margin constraint violation.',lw,lwMin);
end

% Parking spot length check
if pl<vl
    warning('The vehicle length, vl = %.2f, is larger than the parking spot length, pl = %.2f. This will result in collisions in the planned motion.',vl,pl)
elseif pl<vl+2*obsMargin
    warning('The vehicle length plus obstacle margin constraints, vl+2*obsMargin = %.2f, is larger than the parking spot length, pl = %.2f. This will result in obstacle margin constraint violation.',vl+2*obsMargin,pl)
elseif pl-vl+2*obsMargin<0.1
    warning('The vehicle length plus obstacle margin constraints, vl+2*obsMargin = %.2f, is close to the parking spot length, pl = %.2f. This might result in obstacle margin constraint violation.',vl+2*obsMargin,pl)
end

% Parking spot width check
if pw<vw
    warning('The vehicle width, vw = %.2f, is larger than the parking spot width, pw = %.2f. This will result in collisions in the planned motion.',vw,pw)
elseif pw<vw+2*obsMargin
    warning('The vehicle width plus obstacle margin constraints, vw+2*obsMargin = %.2f, is larger than the parking spot width, pw = %.2f. This will result in obstacle margin constraint violation.',vw+2*obsMargin,pw)
elseif pw-vw+2*obsMargin<0.1
    warning('The vehicle width plus obstacle margin constraints, vw+2*obsMargin = %.2f, is close to the parking spot width, pw = %.2f. This might result in obstacle margin constraint violation.',vw+2*obsMargin,pw)
end


%% Set up the parking environment

% Get the obstacles array, the parking entry coordinates and the parking spot coordinates
[obstacles,startCoords,parkingCoords,parkingDims]=parkingObstacles(pw,pl,lw,pTot,dims);
        
% Set the initial state
q0=[startCoords;zeros(3,1)];

% Get the parking spot goal states
qGoal=[parkingCoords(1:3,parkingSpot);
       zeros(3,1)];


%% Determine the parking manoeuvre WP

% Determine states of the turning section of the parking manoeuvre
manObstacles=obstacles; %Create a copy of obstacles for use only in this section
manObstacles(parkingSpot,1)=0; %Mark the goal parking spot free
rhoMan=l/tan(0.9*delMax); %Calculate the manoeuvre turn radius
delPsi=0:(dMan/rhoMan):(pi/2); %Manoeuvre orientation steps
xOffset=(rhoMan)*(+1-cos(delPsi)); %Calculate the corresponding offset along the x axis for every step on orientation
yOffset=rhoMan*sin(delPsi)-h+pl/2+vl/2-ve; %Calculate the correspondinf offset along the y axis for every step on orientation

% Create the turning states for a manoeuvre that starts with orientation
% on the first or fourth quadrant
qGoalAStar1=[qGoal(1)+xOffset; 
             qGoal(2)+yOffset*sign(sin(qGoal(3)));
             qGoal(3)-delPsi*sign(sin(qGoal(3)))];
         
% Create the turning states for a manoeuvre that starts with orientation
% on the second or third quadrant
qGoalAStar2=[qGoal(1)-xOffset; 
             qGoal(2)+yOffset*sign(sin(qGoal(3)));
             qGoal(3)+delPsi*sign(sin(qGoal(3)))];

% Keep only the states of qGoalAStar1 that are free from collision
manWP1=[]; %Create the array that will store the WP
for k=1:size(qGoalAStar1,2)
    % Check for collision
    collision1=collisionCheck(zeros(2,1),[qGoalAStar1(:,k);0;0;0],manObstacles,1,dT,dt,dims);
    % If there is no collision
    if (collision1==0)
        manWP1(:,end+1)=qGoalAStar1(:,k); %Add the state to the WP array
    else
        break %Stop adding WPs if a collision is detected
    end
end
manWP1=fliplr(manWP1);

% Keep only the states of qGoalAStar2 that are free from collision
manWP2=[]; %Create the array that will store the WP
for k=1:size(qGoalAStar2,2)
    % Check for collision
    collision2=collisionCheck(zeros(2,1),[qGoalAStar2(:,k);0;0;0],manObstacles,1,dT,dt,dims);
    % If there is no collision
    if (collision2==0)
        manWP2(:,end+1)=qGoalAStar2(:,k); %Add the state to the WP array
    else
        break %Stop adding WPs if a collision is detected
    end
end
manWP2=fliplr(manWP2);

% Add WPs between the end of the turn section of the manoeuvre and the final
% parking state qGoal
for y=manWP1(2,end):(-dy*sign(sin(qGoal(3)))):qGoal(2)
    manWP1(1:3,end+1)=[qGoal(1);y;qGoal(3)];
    manWP2(1:3,end+1)=[qGoal(1);y;qGoal(3)];
end

% Add the final parking state qGoal to the WP arrays
manWP1(1:3,end+1)=qGoal(1:3);
manWP2(1:3,end+1)=qGoal(1:3);

%Add the missing states and the WP types
manWP1=[manWP1;zeros(3,size(manWP1,2));manWPType*ones(1,size(manWP1,2)-1) 1]; 
manWP2=[manWP2;zeros(3,size(manWP2,2));manWPType*ones(1,size(manWP2,2)-1) 1]; 


%% Determine the parking navigation WP corresponding to manWP1

% Set the goal state
qnav1=manWP1(1:3,1);

% Add virtual obstacles for manWP1 to parkingObst
R1=rotz(rad2deg(qnav1(3))); %Define a rotation matrix
qObsBack1=R1*[rho/2;0;0]+qnav1(1:3); %Obstacle in front of the nav goal state
qObsLeft1=R1*[0;rho;rhoMin]+[qnav1(1:2);0]; %Obstacle left of the nav goal state
qObsRight1=R1*[0;-rho;rhoMin]+[qnav1(1:2);0]; %Obstacle right of the nav goal state
parkingObs1=[1 qObsBack1.' rho vw+2*rho;
             2 qObsLeft1.' 0 0;
             2 qObsRight1.' 0 0];

% Initialise A*
[nodes] = getNodes(parkingDims,dx,dy,order); %Create the nodes
obsListLarge=obstacleIDs(nodes,obstacles,obsNodeMargin); %Get the obstacle node list for parking spots and boundaries with margin obsNodemargin
obsListSmall=obstacleIDs(nodes,obstacles,vw/2+obsMargin); %Get the obstacle node list for parking spots and boundaries with a smaller margin
obsListMan1Large=obstacleIDs(nodes,parkingObs1,vw/2+norm([dx dy])); %Get the obstacle node list for the virtual obstacles with at least half the vehicle width as margin
obsListMan1Small=obstacleIDs(nodes,parkingObs1,0); %Get the obstacle node list for the virtual obstacles with margin 0
obsList1=union(union(obsListSmall,obsListMan1Small),setdiff(obsListLarge,intersect(setdiff(obsListLarge,obsListSmall),setdiff(obsListMan1Large,obsListMan1Small)))); %Combine the different obstacle node lists

% Find a path from the entrance to qnav1 using A*
path1 = AStarFunction(q0(1:2),qnav1(1:2),nodes,obsList1); 

% Convert the path into WPs
navWP1=[path1 [qnav1;zeros(3,1)];navWPType*ones(1,size(path1,2)) 1];


%% Determine the path from q0 manWP2

% Set the goal state
qnav2=manWP2(1:3,1);

% Add virtual obstacles for manWP1 to parkingObst
R2=rotz(rad2deg(qnav2(3))); %Define a rotation matrix
qObsBack2=R2*[rho/2;0;0]+qnav2(1:3); %Obstacle in front of the nav goal state
qObsLeft2=R2*[0;rho;rhoMin]+[qnav2(1:2);0]; %Obstacle left of the nav goal state
qObsRight2=R2*[0;-rho;rhoMin]+[qnav2(1:2);0]; %Obstacle right of the nav goal state
parkingObs2=[1 qObsBack2.' rho vw+2*rho;
             2 qObsLeft2.' 0 0;
             2 qObsRight2.' 0 0];

% Initialise A*
obsListMan2Large=obstacleIDs(nodes,parkingObs2,vw/2+norm([dx dy])); %Get the obstacle node list for the virtual obstacles with at least half the vehicle width as margin
obsListMan2Small=obstacleIDs(nodes,parkingObs2,0); %Get the obstacle node list for the virtual obstacles with margin 0
obsList2=union(union(obsListSmall,obsListMan2Small),setdiff(obsListLarge,intersect(setdiff(obsListLarge,obsListSmall),setdiff(obsListMan2Large,obsListMan2Small)))); %Combine the different obstacle node lists

% Find a path from the entrance to qnav1
path2 = AStarFunction(q0(1:2),qnav2(1:2),nodes,obsList2); %Determine the path using A*

% Convert the path into WPs
navWP2=[path2 [qnav2;zeros(3,1)];navWPType*ones(1,size(path2,2)) 1];


%% Determine the navigation and maneuvre WPs

% Calculate the minimum path distance
minLength1=curveLength([q0(1:3) qnav1]);
minLength2=curveLength([q0(1:3) qnav2]);

% Calculate the path distance
length1=curveLength(path1);
length2=curveLength(path2);

% Stop the script if no feasible WPs are found
if (length1==Inf && length2==Inf)
    warning('A* could not find a feasible path. The geometrcal variables of the parking and vehicle or the obstacle margin might need to be changed')
    return
end

% Select the shortest feasible path
if length1<=length2
    if length1>=minLength1
        navWP=[navWP1(:,1:navWPSteps:end-1) navWP1(:,end)]; %Resample the WPs
        manWP=[manWP1(:,1:manWPSteps:end-1) manWP1(:,end)]; %Resample the WPs 
        
    elseif length2>=minLength2
        navWP=[navWP2(:,1:navWPSteps:end-1) navWP2(:,end)]; %Resample the WPs
        manWP=[manWP2(:,1:manWPSteps:end-1) manWP2(:,end)]; %Resample the WPs
    end
    
else
    if length2>=minLength2
        navWP=[navWP2(:,1:navWPSteps:end-1) navWP2(:,end)]; %Resample the WPs
        manWP=[manWP2(:,1:manWPSteps:end-1) manWP2(:,end)]; %Resample the WPs
        
    elseif length1>=minLength1
        navWP=[navWP1(:,1:navWPSteps:end-1) navWP1(:,end)]; %Resample the WPs
        manWP=[manWP1(:,1:manWPSteps:end-1) manWP1(:,end)]; %Resample the WPs
    end 
end


%% Plan the navigation motion

% Determine the total number of steps
navN=navdN*size(navWP,2);

% Set the initial contorl history guess
navU0 = [zeros(1,navN);zeros(1,navN)];
       
% Follow the WP
[navQ,navU]=motionPlanner(q0,navU0,navWP,obstacles,dims,uBounds,navqBounds,navdN,dT,dt,navMMin,navMMax,navOptions,navDetailedPlanning,navCostBound,navDispSteps,navDispOptSteps);


%% Manoeuvre Planning

% Determine the total number of steps
manN=mandN*size(manWP,2);

% Set the initial state
manq0=navQ(:,end);

% Set the initial contorl history guess
manU0 = [zeros(1,manN);zeros(1,manN)];

% Plan the manoeuvre
[manQ,manU]=motionPlanner(manq0,manU0,manWP,manObstacles,dims,uBounds,manqBounds,mandN,dT,dt,manMMin,manMMax,manOptions,manDetailedPlanning,manCostBound,manDispSteps,manDispOptSteps);


%% Joint Motion

% Concatenate the control history
U=[navU manU];

% Determine the state trajectory
N=size(U,2); %Determine the number of input steps
Qdt=vehicleSim_dt(U,q0,N,dT,dt,l,ta,td); %Obtain the states at every simulation step
Q=vehicleSim(U,q0,N,dT,dt,l,ta,td); %Obtain the states at every input step

% Create timeseries for the states and inputs
TdT=0:dT:dT*N;
Tdt=0:dt:dT*N;

x=timeseries(Qdt(1,:),Tdt,'Name','x');
y=timeseries(Qdt(2,:),Tdt,'Name','y');
psi=timeseries(Qdt(3,:),Tdt,'Name','psi');
v=timeseries(Qdt(4,:),Tdt,'Name','v');
delta=timeseries(Qdt(5,:),Tdt,'Name','delta');
a=timeseries(Qdt(6,:),Tdt,'Name','a');

UNp1=[U zeros(2,1)];
deltaRefdT=timeseries(UNp1(1,:),TdT,'Name','deltaRef');
aRefdT=timeseries(UNp1(2,:),TdT,'Name','aRef');

deltaRef=resample(deltaRefdT,Tdt,'zoh');
aRef=resample(aRefdT,Tdt,'zoh');

% Join the timeseries into a timeseries collection
motion=tscollection({x y psi v delta a deltaRef aRef},'Name','Motion');


%% Joint motion results

% Run time
runTime=toc(runTimeStartTime);

% Get the motion time
motionTime=motion.Time(end);

% Get the motion length
motionLength=curveLength(Qdt);

% Check the obstacle distance constraint for th eresulting motion
obsCons=collisionCheck(U,q0,manObstacles,N,dT,dt,dims);
if obsCons==0
    obsConsBool='True';
else
    warning('Obstacle distance constraints not satisfied')
    obsConsBool='False';
end

% Check the resulting motion for obstacle collisions
dimsCollision=dims;
dimsCollision(10)=0;
collision=collisionCheck(U,q0,manObstacles,N,dT,dt,dimsCollision);
if collision==1
    warning('Collision Detected')
    collisionBool='True';
else
    collisionBool='False';
end

% Final cost
finalCost=costFunction(U,q0,[navWP manWP],N,dT,dt,dims(1),dims(6),dims(7));

% Final state error
error=abs(Q(1:6,end)-qGoal);
error(3)=wrapToPi(error(3));

% Print results
fprintf('\nParking spot %i:\n\tRun time\t= %.2f s.\tMotion duration\t= %.2f s.\n\tPath length\t= %.2f m.\tFinal cost\t\t= %.2e.\n',parkingSpot,runTime,motion.Time(end),curveLength(Qdt),finalCost)
fprintf('\tCollision with obstacles: %s.\n',collisionBool)
fprintf('\tDistance to obstacles constraint satisfied: %s.\n',obsConsBool)
fprintf('\tFinal state error in absolut values:\n\tx\t\t=\t%.2e m\n\ty\t\t= \t%.2e m\n\tpsi\t\t=\t%.2e rad (%.2e deg)\n\tv\t\t= \t%.2e m/s\n \tdelta\t=\t%.2e rad (%.2e deg)\n\ta\t\t=\t%.2e m/s^2\n',error(1:3),rad2deg(error(3)),error(4:5),rad2deg(error(5)),error(6))

% Check vMax constraint compliance
if ~(max(v.Data)<=qBounds(6)+1e-3) 
    warning('Parking spot %i. Max velocity surpassed by %.2e m/s.\n',parkingSpot,max(v.Data)-qBounds(6))
end
% Check vMin constraint compliance
if ~(min(v.Data)+1e-3>=qBounds(7)) 
    warning('Parking spot %i. Min velocity surpassed by %.2e m/s.\n',parkingSpot,min(v.Data)-qBounds(7))
end
% Check deltaMax constraint compliance
if ~(max(delta.Data)<=qBounds(8)+1e-3) 
    warning('Parking spot %i. Max steering angle surpassed by %.2e rad.\n',parkingSpot,max(delta.Data)-qBounds(8))
end
% Check deltaMin constraint compliance
if ~(min(delta.Data)+1e-3>=qBounds(9)) 
    warning('Parking spot %i. Min steering angle surpassed by %.2e rad.\n',parkingSpot,min(delta.Data)-qBounds(9))
end
% Check aMax constraint compliance
if ~(max(a.Data)<=qBounds(1)+1e-3)
    warning('Parking spot %i. Max longitudinal acceleration surpassed by %.2e m/s^2.\n',parkingSpot,max(a.Data)-qBounds(1))
end
% Check aMin constraint compliance
if ~(min(a.Data)+1e-3>=qBounds(2))
    warning('Parking spot %i. Min longitudinal deceleration surpassed by %.2e m/s^2.\n',parkingSpot,min(a.Data)-qBounds(2))
end
% Check jLong constraint compliance
jLong=(aRef.Data-a.Data)./dims(6); %Compute the longitudinal jerk
if ~(max(abs(jLong))<=qBounds(3)+1e-3)
    warning('Parking spot %i. Max longitudinal jerk surpassed by %.2e m/s^3.\n',parkingSpot,max(abs(jLong))-qBounds(3))
end
% Check aLat constraint compliance
aLat=(v.Data.^2)./(dims(1)./(tan(delta.Data))); %Compute the lateral acceleration
if ~(max(abs(aLat))<=qBounds(4)+1e-3)
    warning('Parking spot %i. Max lateral acceleration surpassed by %.2e m/s^2.\n',parkingSpot,max(abs(aLat))-qBounds(4))
end
% Check jLat constraint compliance
jLat=diff(aLat)/dt; %Compute the lateral jerk
if ~(max(abs(jLat))<=qBounds(5)+1e-3)
    warning('Parking spot %i. Max lateral jerk surpassed by %.2e m/s^3.\n',parkingSpot,max(abs(jLat))-qBounds(5))
end

% Display the path plot
if showPath
    plotPath(motion,[navWP manWP],manObstacles,dims,'southwest',dT)
    % Save the path plot
    if savePath
        saveas(gcf,pathFullFileName,'fig')
    end
end

% Display the motion plot
if showMotion
    plotMotion(motion,uBounds,qBounds,dims,'southwest',dT)
    % Save the motion plot
    if saveMotion
        saveas(gcf,motionFullFileName,'fig')
    end
end

% Animate the motion
if animate
    animateMotionGIF(motion,qGoal,manObstacles,dims,dT/2,GIFFullFileName,speedUp)
end


