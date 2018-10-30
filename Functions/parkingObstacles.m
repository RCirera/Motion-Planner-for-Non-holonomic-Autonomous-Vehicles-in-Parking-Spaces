function [obstacles,entranceCoords,parkingCoords,parkingDims] = parkingObstacles(pw,pl,lw,pTot,dims)

% The function parkingObstacles defines and creates the parking
% environment.
%
% The syntax is [obstacles,entranceCoords,parkingCoords,parkingDims] = parkingObstacles(pw,pl,lw,lw,pTot,dims)
%
% The function outputs are:
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
%   - entranceCoords: 3x1 vector containing the coordinates of the entrance
%   to the parking. The first and second rows contain the x and y states.
%   The third contains the orientation. The orientation faces into the
%   parking.  
%   - parkingCoords: 3xp array containing the coordinates of the parking p
%   spot states. Each column contains the coordinates of one spot. There
%   are p spots in the parking. The first and second rows contain the x and
%   y states. The third contains the orientation. 
%   - parkingDims: 1x4 array containing limits on x and y in the parking
%   area. The elements are doubles, and in the following order
%   [xMin,xMax,yMin,yMax].
%
% The function arguments are:
%   - pw: Double, parking spot width.
%   - pl: Double, parking spot length.
%   - lw: Double, parking lane width.
%   - pTot: Integer, number of parking spots in the parking. This
%   parrticular parking is organized in 4 blocks with 2 opposite facing
%   sections each. The total number of parking spots must therefore be a
%   multiple of 8.
%   - dims: Array containing information on geometric properties of the
%   vehicle for which the motion is calculated.


%% Initialization

pNum=pTot/8; %Number of parking spots per section

phi=deg2rad(90); %Rotation angle of the parking spaces (between 0 and 90deg)

pc=[pl/2;pw/2]; %Define parking space centre location (with angle=0)

cg=[pl/2-dims(2)/2+dims(4);pw/2]; %Define vehicle cg location (with angle=0)

% Determine the inter-spot distance. The geometric properties determine
% wether the spots are in parallel or battery configuration
if pl*sin(phi)>=pw*cos(phi)
    px=pw/sin(phi); %Parking spot length along the x axis in battery configuration
else
    px=pl/cos(phi); %Parking spot length along the x axis in parallel configuration
end

py=(pl*sin(phi)+pw*cos(phi)); %parking spot length along the y axis

pax=(pNum-1)*px+pw*sin(phi)+pl*cos(phi); %Parking area total length along x axis

pay=py*2; %Parking area total length along y axis


%% Create the parking obstacles and parking spots

% Create Parking Area 1, top

R1=[cos(phi) -sin(phi); %Rotation matrix for phi
    sin(phi)  cos(phi)];

pc1top=R1*pc+[lw/2+pw*sin(phi);lw/2+py]; %Coordinates of the centre of the first parking spot on the top line of parking area 1

obs1top=ones(pNum,1)*[1 pc1top.' phi pl pw]; %Create the parked car obstacles on the top line of parking area 1
obs1top(:,2)=obs1top(:,2)+(0:1:pNum-1).'*px; %Set the individual spot obstacles x locations

% Determine the cg locations for parking area 1, top
cg1top=R1*cg+[lw/2+pw*sin(phi);lw/2+py]; %x and y coordinates of the cg of the vehicle on the first parking spot on the top line of parking area 1
vehicle1top=ones(pNum,1)*[cg1top.' phi]; %Create the parked car cg states on the top line of parking area 1
vehicle1top(:,1)=vehicle1top(:,1)+(0:1:pNum-1).'*px; %Set the individual parking spots x locations

% Create Parking Area 1, bottom
R1=[cos(-phi) -sin(-phi); %Rotation matrix for -phi
    sin(-phi)  cos(-phi)];

pc1bot=R1*([1;-1].*pc)+[lw/2+pw*sin(phi);lw/2+py]; %Coordinates of the centre of the first parking spot on the bottom line of parking area 1

obs1bot=ones(pNum,1)*[1 pc1bot.' -phi pl pw]; %Create the parked car obstacles on the bottom line of parking area 1
obs1bot(:,2)=obs1bot(:,2)+(0:1:pNum-1).'*px; %Set the individual spot obstacles x locations

obs1=[obs1top;obs1bot]; %Join obstacles top and bottom rows

% Determine the cg locations for parking area 1, bottom
cg1bot=R1*([1;-1].*cg)+[lw/2+pw*sin(phi);lw/2+py]; %x and y coordinates of the cg of the vehicle on the first parking spot on the bottom line of parking area 1
vehicle1bot=ones(pNum,1)*[cg1bot.' -phi]; %Create the parked vehicle cg states on the bottom line of parking area 1
vehicle1bot(:,1)=vehicle1bot(:,1)+(0:1:pNum-1).'*px; %Set the individual parking spots x locations

vehicle1=[vehicle1top;vehicle1bot]; %Join top and bottom row parked vehicle cg information

% Create Parking Area 2
obs2=obs1;
obs2(:,2)=obs2(:,2)-pax-lw;

% Determine vehicle states on parking spots of parking area 2
vehicle2=vehicle1;
vehicle2(:,1)=vehicle2(:,1)-pax-lw;

% Create Parking Area 3
obs3=obs2;
obs3(:,3)=obs3(:,3)-pay-lw;

% Determine vehicle states on parking spots of parking area 3
vehicle3=vehicle2;
vehicle3(:,2)=vehicle3(:,2)-pay-lw;

% Create Parking Area 4
obs4=obs1;
obs4(:,3)=obs4(:,3)-pay-lw;

% Determine vehicle states on parking spots of parking area 2
vehicle4=vehicle1;
vehicle4(:,2)=vehicle4(:,2)-pay-lw;

% Create Parking Boundaries
obsWidth=5;
obs5=[1 0   lw/2+py*2+lw+obsWidth/2  0 lw+2*lw+2*pax obsWidth; %Top
      1 -((lw/2+pax+lw)/2+lw/4) -(lw/2+py*2+lw+obsWidth/2) 0 (pax+lw) obsWidth; %Bottom left
      1  ((lw/2+pax+lw)/2+lw/4) -(lw/2+py*2+lw+obsWidth/2) 0 (pax+lw) obsWidth; %Bottom right
      1   lw/2+pax+lw+obsWidth/2  0 0 obsWidth 2*obsWidth+lw+2*lw+2*pay; %Right
      1 -(lw/2+pax+lw+obsWidth/2) 0 0 obsWidth 2*obsWidth+lw+2*lw+2*pay]; %Left
  
% Entire parking area
obstacles=[obs1;obs2;obs3;obs4;obs5]; %All the obstacles

entranceCoords=[0;-(lw/2+pay+lw+obsWidth/2+dims(2)/2);pi/2]; %Coordinates of the entrance to the parking

parkingCoords=[vehicle1;vehicle2;vehicle3;vehicle4].'; %State coordinates of the parking spots

% Parking dimensions
xMax=lw/2+pax+lw+obsWidth; %Maximum value of x in the map
xMin=-xMax; %Minimum value of x in the map
yMax=lw/2+pay+lw+obsWidth; %Maximum value of y in the map
yMin=-yMax; %Minimum value of y in the map
parkingDims=[xMin,xMax,yMin,yMax];

end

