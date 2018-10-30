function [Q,U]=motionPlanner(q0,U0,WP,obstacles,dims,uBounds,comf,dN,dT,dt,MMin,MMax,options,detailedPlanning,costBound,dispSteps,dispOptSteps)

% The function motionPlanner calculates a motion with state trajectory Q and
% control history U that results from tracking a series of waypoints WP by
% minimizing a series of cost functions.
%
% The syntax is [Q,U,motion,cost,stepTimes,stepTypes]=motionPlanner(q0,U0,WP,obstacles,dims,uBounds,comf,dN,dT,dt,MMin,MMax,options,dispResults,detailedPlanning,costBound,dispSteps,dispOptSteps)

% The function outputs are:
%   - Q: 6x(N+1) array containing the state trajectory of the calculated
%   motion. N is the number of time steps of the motion.
%   - U: 2xN array containing the control history of the calculated motion.
%
% The function arguments are:
%   - q0: 6x1 vector containing the initial state of the motion to be
%   planned.
%   - U0: 2xN array containing the initial control history guess.
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
%   - uBounds: 2x2 array containing the maximum and minimum values allowed
%   for the control history inputs.
%   - comf: 7x1 vector containing maximum values for certain parameters to
%   ensure passenger comfort. In order, the elements are maximum
%   longitudinal acceleration, minimum longitudinal acceleration (maximum
%   deceleration/braking), maximum longitudinal jerk, maximum lateral
%   acceleration, maximum lateral jerk, maximum longitudinal velocity and
%   minimum longitudinal velocity. The units are m/s for velocities, m/s^2
%   for accelerations and m/s^3 for jerks. 
%   - dN: Integer, number of time steps per WP allowed for the motion.
%   - dT: Double, size in seconds of each time step.
%   - dt: Double, size in seconds of the subdivisions of the simulation
%   time steps. The system equation is recomputed every dt seconds to
%   simulate the system. dT must be an integer multiple of dt.
%   - MMin: Integer, minimum number of WP considered at a time.
%   - MMax: Integer, maximum number of WP considered at a time.
%   - options: Optimization options.
%   - detailedPlanning: Boolean, toggle the use of detailed planning on or
%   off. When detailedPlanning is 1, th eplanner considers the constraints
%   at every simulation step. When detailedPlanning is 0, the planner
%   considers the constraints at every input time step only.
%   - costBound: Double, value of the cost function that the planner aims
%   to achieve. 
%   - dispSteps: Boolean, toggles the display of the resulting path every
%   time that fmincon is run.
%   - dispOptSteps: Boolean, toggles the display of the paths resulting
%   from the intermediate minimization steps of fmincon.


%% Initialization

% Determine the total number of time steps
N=dN*size(WP,2);

% Minimum number of WP considered at a time
M=min(MMin,size(WP,2));

% Set the upper and lower bounds for the optimization variables
UB = [uBounds(3)*ones(1,N); uBounds(1)*ones(1,N)];
LB = [uBounds(4)*ones(1,N); uBounds(2)*ones(1,N)];


%% Find the motion

% Set the control history
U=U0; 

% Beginning at WP number M and until the last WP
for k=M:1:size(WP,2)
    S=M;
    feasible=0;
    motionPlanned=0;
    % While the current local motion planned is not feasible and S is not
    % too large
    while (~feasible && S<=min(MMax,k))
        
        % Determine the state trajectory
        Q=vehicleSim(U,q0,N,dT,dt,dims(1),dims(6),dims(7));

        % Determine the initial state for the local planning
        q0Local=Q(:,((k-S)*dN)+1);
        
        %% Compute U1

        % Define the cost function
        costfun=@(u) costFunction(u,q0Local,WP(:,(k-S)+1:k),dN*S,dT,dt,dims(1),dims(6),dims(7));

        % Define the constraint function, without obstacles
        if detailedPlanning==1
            consfun=@(u) consFunction_dt(u,q0Local,dN*S,dT,dt,dims,comf);
        else
            consfun=@(u) consFunction(u,q0Local,dN*S,dT,dt,dims(1),dims(6),dims(7),comf);
        end
        
        % Set a mid-optimization plot function if required
        if dispOptSteps==1
            outfcn=@(x,optimValues,state) midOptPlot(x,optimValues,state,q0Local,WP(:,(k-S)+1:k),obstacles,dims,dT,dt,'south');
            options=optimoptions(options,'OutputFcn',outfcn);
        end
        
        % Optimize using fmincon
        U1=U;
        [U1(:,(k-S)*dN+1:dN*k),cost1]=fmincon(costfun,U(:,(k-S)*dN+1:dN*k),[],[],[],[],LB(:,(k-S)*dN+1:dN*k),UB(:,(k-S)*dN+1:dN*k),consfun,options);
        U=U1;
        
        % Check for collisions on the motion
        collision1=collisionCheck(U1(:,1:dN*k),q0,obstacles,k*dN,dT,dt,dims);

        % Display the path if requested
        if dispSteps==1
            Q=vehicleSim(U1(:,1:dN*k),q0,dN*k,dT,dt,dims(1),dims(6),dims(7));
            plotPathQ(Q,WP(:,(k-S)+1:k),obstacles,dims,'northwest')
        end

        % If the cost is larger than the required bound, increase the horizon backwards    
        if cost1>costBound
            % Go to the next value of S
            S=S+1;
            continue
        % If the cost is within bounds and there is no collision
        elseif (collision1==0)
            % Go to the next value of k, go to next iteration of for loop
            motionPlanned=1;
            break
        % If the cost is within bounds but there is a collision, mark the
        % local motion as feasible and move on to the next steps of the
        % local planning to try to find a motion avoiding the collision
        else
            feasible=1;
        end

        %% If a collision occurs, try using the next WP as goal
        if (k~=size(WP,2)) %Only if the current goal WP is not the last one
            
            % Determine the state trajectory
            Q=vehicleSim(U,q0,N,dT,dt,dims(1),dims(6),dims(7));

            % Determine the initial state for the local planning
            q0Local=Q(:,((k-S)*dN)+1);

            % Cost function
            costfun=@(u) costFunction(u,q0Local,WP(:,(k-S)+1:(k+1)),dN*(S+1),dT,dt,dims(1),dims(6),dims(7));

            % Define the constraint function, without obstacles
            if detailedPlanning==1
                consfun=@(u) consFunction_dt(u,q0Local,dN*(S+1),dT,dt,dims,comf);
            else
                consfun=@(u) consFunction(u,q0Local,dN*(S+1),dT,dt,dims(1),dims(6),dims(7),comf);
            end
            
            % Set a mid-optimization plot function if required
            if dispOptSteps==1
                outfcn=@(x,optimValues,state) midOptPlot(x,optimValues,state,q0Local,WP(:,(k-S)+1:(k+1)),obstacles,dims,dT,dt,'south');
                options=optimoptions(options,'OutputFcn',outfcn);
            end

            % Optimize using fmincon
            U2=U;
            [U2(:,(k-S)*dN+1:dN*(k+1)),cost2]=fmincon(costfun,U(:,(k-S)*dN+1:dN*(k+1)),[],[],[],[],LB(:,(k-S)*dN+1:dN*(k+1)),UB(:,(k-S)*dN+1:dN*(k+1)),consfun,options);
          
            % Check for collisions on the motion
            collision2=collisionCheck(U2(:,1:dN*(k)),q0,obstacles,(k)*dN,dT,dt,dims);

            % Display the path if requested
            if dispSteps==1
                Q=vehicleSim(U2(:,1:dN*(k+1)),q0,dN*(k+1),dT,dt,dims(1),dims(6),dims(7));
                plotPathQ(Q,WP(:,(k-S)+1:(k+1)),obstacles,dims,'north')
            end
            
            % If the cost is below bounds and there is no collision, move
            % on to the next local planning step
            if (cost2<=costBound && collision2==0)
                U=U2;
                motionPlanned=1;
                break
            end
        end
    end
    
    % If a succesful local motion has been planned, move on to the next
    % local planning step
    if motionPlanned==1
        continue
    end
    
    %% If necessary, plan with obstacles
    
    % Set the current value of S
    S0=S;
    
    % For every value of S from the current until the maximum allowed
    for S=S0:min(MMax,k)
    
        % Determine the state trajectory
        Q=vehicleSim(U,q0,N,dT,dt,dims(1),dims(6),dims(7));
        
        % Determine the initial state for the local planning
        q0Local=Q(:,((k-S)*dN)+1);

        % Define the cost function
        costfun=@(u) costFunction(u,q0Local,WP(:,(k-S)+1:k),dN*S,dT,dt,dims(1),dims(6),dims(7));

        % Define the constraint function with obstacles
        if detailedPlanning==1
            consfun=@(u) consFunctionObs_dt(u,q0Local,dN*S,dT,dt,comf,dims,obstacles,costBound,costfun);
        else
            consfun=@(u) consFunctionObs(u,q0Local,dN*S,dT,dt,comf,dims,obstacles,costBound,costfun);
        end
        
        % Set a mid-optimization plot function if required
        if dispOptSteps==1
            outfcn=@(x,optimValues,state) midOptPlot(x,optimValues,state,q0Local,WP(:,(k-S)+1:k),obstacles,dims,dT,dt,'south');
            options=optimoptions(options,'OutputFcn',outfcn);
        end
        
        % Optimize with fmincon
        U3=U;
        [U3(:,(k-S)*dN+1:dN*k),cost3]=fmincon(costfun,U(:,(k-S)*dN+1:dN*k),[],[],[],[],LB(:,(k-S)*dN+1:dN*k),UB(:,(k-S)*dN+1:dN*k),consfun,options);
        
        % Check for collisions
        collision3=collisionCheck(U3(:,1:dN*(k)),q0,obstacles,(k)*dN,dT,dt,dims);

        % Display the path if requested
        if dispSteps==1
            Q=vehicleSim(U3(:,1:dN*k),q0,dN*k,dT,dt,dims(1),dims(6),dims(7));
            plotPathQ(Q,WP(:,(k-S)+1:k),obstacles,dims,'southwest')
        end

        % If a good motion is found, continue to next value of k
        if (cost3<=costBound && collision3==0)
            U=U3;
            break
        end
    end
end

% Determine the state trajectory
Q=vehicleSim(U,q0,N,dT,dt,dims(1),dims(6),dims(7));        

end
