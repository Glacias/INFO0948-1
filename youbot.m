%%% This code must be run on Matlab 2019b or higher
% Made by Simon BERNARD and Ivan KLAPKA
function youbot()
%% Initiate the connection to the simulator.
close all
disp('Program started');
% Use the following line if you had to recompile remoteApi
%vrep = remApi('remoteApi', 'extApi.h');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

% If you get an error like:
%   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
% Make sure your code is within a function! You cannot call V-REP from a script.

if id < 0
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connection whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% This will only work in "continuous remote API server service".
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

% Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
% The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the
% file focused/youbot_arm.m).
h = youbot_init(vrep, id);
h = youbot_hokuyo_init(vrep, h);

% Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or
% the robot pose with the simx_opmode_buffer option.
pause(.2);

%% Youbot constants
% The time step the simulator is using (your code should run close to it).
timestep = .05;

% Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

%% Initial values
disp('Starting robot');
youbotWidth = 0.4; % m
youbotLength = 0.6; % m

% Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels.
% They are adapted at each iteration by the code.
forwBackVel = 0; % Move straight ahead.
rightVel = 0; % Go sideways.
rotateRightVel = 0; % Rotate.

% Set the arm to its starting configuration.
res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object.
vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error.

for i = 1:5
    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end

res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);

% Get figure for ploting the map
fig = figure;

% Get the initial position and orientation of the robot.
[res, youbotStartingPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
[res, youbotStartingEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
youbotStartingEuler(3) = 0; % Align with gps for orientation of the map

%%% Odometry
[~ , wh1_a_prev] = vrep.simxGetJointPosition(id, h.wheelJoints(1), vrep.simx_opmode_buffer);
[~ , wh2_a_prev] = vrep.simxGetJointPosition(id, h.wheelJoints(2), vrep.simx_opmode_buffer);
[~ , wh3_a_prev] = vrep.simxGetJointPosition(id, h.wheelJoints(3), vrep.simx_opmode_buffer);
[~ , wh4_a_prev] = vrep.simxGetJointPosition(id, h.wheelJoints(4), vrep.simx_opmode_buffer);

wh_radius = 0.05; % Radius of the wheels

% First position estimation [x, y, theta]
youbotCurrPosEst = [0; 0; 0]; % Starting pos for odometry

%%%%% Initialise the maps
displayMap = true;
sizeOfWorldMap = [16 16];
worldResolution = 0.2; %m per cell

%%% Occupancy map
% Create the map with twice the size to be sure it is large enough
occMap = occupancyMap(sizeOfWorldMap(1)*2, sizeOfWorldMap(2)*2, 1/worldResolution);
% Set the map (0,0) as the staring position of the robot
occMap.GridOriginInLocal = -[diff(occMap.XWorldLimits) diff(occMap.YWorldLimits)]/2;

%%% Obstacle map
obstMat = checkOccupancy(occMap);
inflatedObstMat = checkOccupancy(occMap);

% Set the positions the robot is on as explored
[X, Y] = meshgrid(-youbotWidth:worldResolution:youbotWidth, -youbotLength:worldResolution:youbotLength);
gridInitPosX = reshape(X, 1, []); % Make a vector of the matrix X.
gridInitPosY = reshape(Y, 1, []);
setOccupancy(occMap, [gridInitPosX; gridInitPosY].', 0);

% Sensor max range
hokuyoMaxRange = 5;
% Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
% see, by selecting the points within this mesh that are within the visibility range.
[X, Y] = meshgrid(-hokuyoMaxRange:worldResolution:hokuyoMaxRange, -hokuyoMaxRange:worldResolution:hokuyoMaxRange);
sensorGridX = reshape(X, 1, []); % Make a vector of the matrix X.
sensorGridY = reshape(Y, 1, []);
allowed_sensor_update = true;

% Initialise array for saving points
itt_max = 10000;
nb_points = 684;
saved_info_ok = false([1 itt_max]);
saved_pts = zeros(itt_max,3,nb_points);
saved_contacts = zeros(itt_max,nb_points);
saved_inpoly = zeros(itt_max, length(sensorGridX));
saved_youbotMapPos = zeros(itt_max, 3);
saved_youbotMapEuler = zeros(itt_max, 3);

% Creation of the graph for path planning
currentlyInPath = false;
lookingInReach = false;
inRecovery = false;
faceGoal = false;
path = [0 0];
posNextInReach = [0, 0];
iPath = 1;

% Order for stopping
must_stop = false;
must_stop_gps = false;

% Gps update number
nbGpsUpdate = 1;

% Make sure everything is settled before we start.
pause(2);
total_time = 0;
previous_time_gps = 0;
elapsed = 0;
itnb = 0;
sv_ind = 1;
occMapImp = copy(occMap);

% Milestone
exploration_milestone = true;
plotGrasp = false;

% Manipulation milestone
init_manip = true;
inTravel = false;
facePoint = false;
goalPlanning = false;

% Initialise the state machine.
fprintf('Starting the exploration\n');

%% Start the simulation
while true
    %% Start of loop
    tic % See end of loop to see why it's useful.
    
    % Test for lost connection
    if vrep.simxGetConnectionId(id) == -1
        error('Lost connection to remote API.');
    end
    
    %% Get the orientation of the robot.
    [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Translate to map position
    % We suppose that the robot only travel on the X and Y axis and not on the Z axis
    youbotMapEuler1 = youbotEuler(1) - youbotStartingEuler(1);
    youbotMapEuler2 = youbotEuler(2) - youbotStartingEuler(2);
    youbotMapEuler3 = youbotEuler(3) - youbotStartingEuler(3);
    
    %% Odometry
    % Get angle of rotation of the wheels
    [~ , wh1_a] = vrep.simxGetJointPosition(id, h.wheelJoints(1), vrep.simx_opmode_buffer);
    [~ , wh2_a] = vrep.simxGetJointPosition(id, h.wheelJoints(2), vrep.simx_opmode_buffer);
    [~ , wh3_a] = vrep.simxGetJointPosition(id, h.wheelJoints(3), vrep.simx_opmode_buffer);
    [~ , wh4_a] = vrep.simxGetJointPosition(id, h.wheelJoints(4), vrep.simx_opmode_buffer);
    
    % Get the distance travelled bewteen previous mesurements
    wh1_d = angdiff(wh1_a,wh1_a_prev) * wh_radius;
    wh2_d = angdiff(wh2_a,wh2_a_prev) * wh_radius;
    wh3_d = angdiff(wh3_a,wh3_a_prev) * wh_radius;
    wh4_d = angdiff(wh4_a,wh4_a_prev) * wh_radius;
    
    % Update previous mesurements
    wh1_a_prev = wh1_a;
    wh2_a_prev = wh2_a;
    wh3_a_prev = wh3_a;
    wh4_a_prev = wh4_a;
    
    % Compute forward, lateral and rotational motion
    forwBack_d = -0.248 * (wh1_d + wh2_d + wh3_d + wh4_d);
    leftRigth_d = 0.238 * (- wh1_d + wh2_d - wh3_d + wh4_d);
    rotRight_a = 0.6439 * (wh1_d + wh2_d - wh3_d - wh4_d);
    
    % Compute velocities
    ts = elapsed;
    if elapsed < 0.05
        ts = 0.05;
    end
    forwBack_v = forwBack_d/ts;
    leftRigth_v = leftRigth_d/ts;
    rotRight_v =  rotRight_a/ts;
    
    %% Velocity Model
    % Thresold for considering a velocity null or not
    treshold_v = 0.00001;
    treshold_w = 0.00001;
    
    % Diagonal
    if abs(leftRigth_v) > treshold_v && abs(forwBack_v) > treshold_v
        v = sqrt(forwBack_v^2 + leftRigth_v^2)* sign(forwBack_v);
        theta = youbotCurrPosEst(3) - atan(leftRigth_v/forwBack_v);
        
        % Lateral
    elseif abs(leftRigth_v) > treshold_v
        v = leftRigth_v;
        theta = youbotCurrPosEst(3) - pi/2;
        
        % Forward or not moving
    else
        v = forwBack_v;
        theta = youbotCurrPosEst(3);
    end
    
    % Compute position from velocity and angle
    % With rotation
    if abs(rotRight_v) > treshold_w
        r = v/rotRight_v;
        d_odo = [- r*sin(theta) + r*sin(theta + rotRight_v*ts);
            r*cos(theta) - r*cos(theta + rotRight_v*ts);
            rotRight_v*ts];
        youbotCurrPosEst = youbotCurrPosEst + d_odo;
        
        % Without rotation
    else
        d_odo = [v*ts*cos(theta);
            v*ts*sin(theta);
            rotRight_v*ts];
        youbotCurrPosEst = youbotCurrPosEst + d_odo;
    end
    
    youbotMapPos = [youbotCurrPosEst(1) youbotCurrPosEst(2) 0];
    youbotCurrPosEst(3) = youbotMapEuler3 + pi/2;
    
    % Update from gps
    if (total_time >= (previous_time_gps + 60)) && allowed_sensor_update
        % If the robot is not moving
        if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
            fprintf("Received gps coordinates\n");
            
            %% Get the position and the orientation of the robot.
            [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % Get gps info
            rot = [cos(youbotStartingEuler(3)) -sin(youbotStartingEuler(3)) 0; sin(youbotStartingEuler(3)) cos(youbotStartingEuler(3)) 0 ; 0 0 1];
            gps = (youbotPos - youbotStartingPos) * rot;
            gpsEuler3 = youbotMapEuler3 + pi/2;
            
            % Compute error made
            err_odo = youbotMapPos - [gps(1) gps(2) 0];
            err_odo_it = err_odo/(sv_ind-1);
            
            % Do the correction
            i = 2;
            while i < sv_ind
                if saved_info_ok(i)
                    % Restore position
                    youbotMapPos = saved_youbotMapPos(i,:) - (err_odo_it * (i-1));
                    youbotMapEuler1 = saved_youbotMapEuler(i,1);
                    youbotMapEuler2 = saved_youbotMapEuler(i,2);
                    youbotMapEuler3 = saved_youbotMapEuler(i,3);
                    
                    % Create the transform to the restored position
                    trfStartingPos = transl(youbotMapPos) * trotx(youbotMapEuler1) * troty(youbotMapEuler2) * trotz(youbotMapEuler3);
                    
                    % Restored inpoly
                    in = logical(saved_inpoly(i,:));
                    
                    % Update the maps
                    % Transform sensor info into map coord
                    inRefGrid = homtrans(trfStartingPos, [sensorGridX(in); sensorGridY(in); zeros(size(sensorGridX(in)))]);
                    % Update free spaces
                    if not(isempty(inRefGrid))
                        setOccupancy(occMapImp, [inRefGrid(1,:); inRefGrid(2,:)].', 0);
                    end
                    
                    % Restored pts
                    pts = squeeze(saved_pts(i,:,:));
                    
                    % Restored contacts
                    contacts = logical(saved_contacts(i,:));
                    
                    % Transform sensor info into map coord
                    inRefContacts = homtrans(trfStartingPos, pts(:,contacts));
                    % Update occupied spaces
                    if not(isempty(inRefContacts))
                        setOccupancy(occMapImp, [inRefContacts(1,:); inRefContacts(2,:)].', 1);
                    end
                end
                i = i + 1;
            end
            
            % Update map
            occMap = copy(occMapImp);
            
            % Create an inflated version of the map where the obstacles are inflated
            inflatedOccMap = copy(occMapImp);
            inflate(inflatedOccMap, worldResolution);
            % Create matrices of the maps
            inflatedObstMat = checkOccupancy(inflatedOccMap);
            obstMat = checkOccupancy(occMapImp);
            
            % Update pos
            youbotCurrPosEst = [gps(1); gps(2); gpsEuler3];
            
            youbotMapPos = [youbotCurrPosEst(1) youbotCurrPosEst(2) 0];
            youbotCurrPosEst(3) = gpsEuler3;
            youbotMapEuler3 = youbotEuler(3) - youbotStartingEuler(3);
            
            % Reset the save indicices
            sv_ind = 1;
            
            % Re-Initialise array for saving points
            saved_pts = zeros(itt_max,3,nb_points);
            saved_contacts = zeros(itt_max,nb_points);
            saved_inpoly = zeros(itt_max, length(sensorGridX));
            saved_youbotMapPos = zeros(itt_max, 3);
            saved_youbotMapEuler = zeros(itt_max, 3);
            
            % Increase update number
            nbGpsUpdate = nbGpsUpdate + 1;
            previous_time_gps = total_time;
            
            % Start moving again
            must_stop_gps = false;
            
            currentlyInPath = false;
            lookingInReach = false;
            
            % Plot
            if exploration_milestone
                plotMapM1(fig, obstMat, inflatedOccMap, inflatedObstMat, accUnex, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
            else
                plotMapM2(fig, obstMat, inflatedOccMap, inflatedObstMat, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
            end
        else
            % Send order to stop
            must_stop_gps = true;
        end
    end
    
    
    % Create the transform to the absolute ref
    trfStartingPos = transl(youbotMapPos) * trotx(youbotMapEuler1) * troty(youbotMapEuler2) * trotz(youbotMapEuler3);
    
    
    if allowed_sensor_update
        %% Read data from sensor
        % Read data from the depth sensor, more often called the Hokuyo.
        % This function returns the set of points the Hokuyo saw in pts.
        % Each column of pts has [x;y;z;distancetosensor]; contacts indicates, for each point, if it
        % corresponds to an obstacle.
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer);
        
        % Simplify the points from the sensor to make the computation of inpolygon easier
        pts2D = [[h.hokuyo1Pos(1); h.hokuyo1Pos(2)] ,[pts(1,:); pts(2,:)], [h.hokuyo2Pos(1); h.hokuyo2Pos(2)]];
        simplePoly = simplifyEdgeOfVision(pts2D);
        
        % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo (it returns the area that
        % is visible, but the visualisation draws a series of points that are within this visible area).
        in = inpolygon(sensorGridX, sensorGridY, simplePoly(1, :), simplePoly(2, :));
        
        %% Save the data from sensor and position for later correction
        saved_info_ok(sv_ind) = true;
        saved_pts(sv_ind,:,:) = pts;
        saved_contacts (sv_ind,:) = contacts;
        saved_inpoly(sv_ind,:) = in;
        saved_youbotMapPos(sv_ind,:) = youbotMapPos;
        saved_youbotMapEuler(sv_ind,:) = [youbotMapEuler1 youbotMapEuler2 youbotMapEuler3];
        
        % Increment save indice
        sv_ind = sv_ind +1 ;
        
        %% Update the maps
        % Transform sensor info into map coord
        inRefGrid = homtrans(trfStartingPos, [sensorGridX(in); sensorGridY(in); zeros(size(sensorGridX(in)))]);
        % Update free spaces
        if not(isempty(inRefGrid))
            setOccupancy(occMap, [inRefGrid(1,:); inRefGrid(2,:)].', 0);
        end
        
        % Transform sensor info into map coord
        inRefContacts = homtrans(trfStartingPos, pts(:,contacts));
        % Update occupied spaces
        if not(isempty(inRefContacts))
            setOccupancy(occMap, [inRefContacts(1,:); inRefContacts(2,:)].', 1);
        end
        
        % Create an inflated version of the map where the obstacles are inflated
        inflatedOccMap = copy(occMap);
        inflate(inflatedOccMap, worldResolution);
        % Creates matrices of the maps
        inflatedObstMat = checkOccupancy(inflatedOccMap);
        obstMat = checkOccupancy(occMap);
        
        % Compute the position in grid
        posInGrid = world2grid(occMap, [youbotMapPos(1) youbotMapPos(2)]);
        
        %% Compute vertex of obstacles and passage points
        passPtsInGrid = get_vertex(inflatedObstMat);
        if isempty(passPtsInGrid)
            passPtsInGrid = [posInGrid(1); posInGrid(2)];
        end
        passPts = grid2world(inflatedOccMap, passPtsInGrid.');
        
    else
        % Don't save the data and marks it as not ok
        saved_info_ok(sv_ind) = false;
        
        % Increment save indice
        sv_ind = sv_ind +1 ;
    end
    
    %% Switch from milestone
    if exploration_milestone
        %%% Exploration milestone %%%
        %% Compute closest unexplored and accesible point
        accUnex = get_unexploredAccessible(inflatedObstMat);
        
        %% Check that there is still objectives
        % If not, we have explored the entire map
        if isempty(accUnex)
            % Simulation done : exit the function
            fprintf("Simulation done : The map has been explored\n");
            if displayMap
                plotMapM1(fig, obstMat, inflatedOccMap, inflatedObstMat, accUnex, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
            end
            pause(0.000001);
            exploration_milestone = false;
            must_stop = true;
            forwBackVel = 0;
            rightVel = 0;
            rotateRightVel = 0;
            
            continue;
        end
        
        % Compute point 5m in front of the robot or less if a wall is in the way
        frontPos = [0 -(abs(pts(2,341)))] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)] + [youbotMapPos(1) youbotMapPos(2)];
        frontPosInGrid = world2grid(occMap, frontPos);
        % Compute the closest unexplored point to that front point
        closestInGrid = get_closestUnexploredAccessible(frontPosInGrid, accUnex, 1);
        if isnan(closestInGrid)
            fprintf("Closest is Nan\n");
            continue;
        end
        closest = grid2world(inflatedOccMap, closestInGrid.');
        
        %% Path planning
        % Check that path is not empty
        if isempty(path)
            currentlyInPath = false;
        end
        
        % If in path, follow it and upadte when you reach an edge
        if currentlyInPath
            size_path = size(path);
            goal = path(size_path(1),1:2);
            goalInGrid = world2grid(inflatedOccMap, goal);
            % Robot can move again
            must_stop = false;
            
            % If the end goal is a wall or was explored stop the planning
            if inflatedObstMat(goalInGrid(1),goalInGrid(2)) == 1
                currentlyInPath = false;
                
                % Recovery mode : If we are in the path for too long without progress, restart planer with the same goal
                % The recovery mode will be aborted if a closest node is in reach
            elseif currentlyInPathIter > 200
                currentlyInPathIter = 0;
                recoveredGoal = goal;
                inRecovery = true;
                currentlyInPath = false;
                
                % If in reach of the last node (unexplored)
            elseif iPath == size_path(1)
                currentlyInPath = false;
                % If explored, face the goal
                if isExplored(goalInGrid, inflatedObstMat)
                    faceGoal = true;
                    recoveredGoal = goal;
                    % If not explored go to in reach
                else
                    posNextInReach = goal;
                    lookingInReach = true;
                    lookingInReachIter = 0;
                end
                % Else follow the path
            else
                pos = [youbotMapPos(1) youbotMapPos(2)];
                coordNextNode = path(iPath,:);
                % If still far, follow the same point
                if norm(coordNextNode - pos) > 0.05
                    [forwBackVel, rightVel, rotateRightVel] = go_to_location(pos, youbotMapEuler3, [0 -1], coordNextNode);
                    currentlyInPathIter = currentlyInPathIter + 1;
                    % Check that objective is still in sight
                    if ~isInReach([youbotMapPos(1) youbotMapPos(2)], coordNextNode, occMap, inflatedOccMap)
                        lookingInReach = false;
                        currentlyInPath = false;
                    end
                else % If close go to the next point in path
                    iPath = iPath +1;
                    currentlyInPathIter = 0;
                    % Stop
                    forwBackVel = 0;
                    rightVel = 0;
                    rotateRightVel = 0;
                end
            end
            
            % If looking for goal in reach don't update instantly the next point to go
        elseif lookingInReach
            % Check that objective is still in sight
            if ~isInReach([youbotMapPos(1) youbotMapPos(2)], closest, occMap, inflatedOccMap)
                lookingInReach = false;
            elseif lookingInReachIter < 40
                [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], posNextInReach);
                lookingInReachIter = lookingInReachIter +1;
            else
                % Check that we have discovered the point
                posNextInReachInGrid = world2grid(inflatedOccMap, posNextInReach);
                if isExplored(posNextInReachInGrid, inflatedObstMat)
                    lookingInReach = false;
                else % if not, continue exploring
                    lookingInReachIter = 0;
                end
            end
            % Robot can move again
            must_stop = false;
            % If goal is in reach go into lookingInReach mode
        elseif isInReach([youbotMapPos(1) youbotMapPos(2)], closest, occMap, inflatedOccMap)
            posNextInReach = closest;
            [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], closest);
            lookingInReach = true;
            lookingInReachIter = 0;
            % We consider the recevory to be aborted if the closest point is in reach
            inRecovery = false;
            % We consider we can forget to face the old goal if the closest point is in reach
            faceGoal = false;
            % Robot can move again
            must_stop = false;
            % If goal is not in reach, plan the path
        elseif faceGoal
            % Robot can move again
            must_stop = false;
            [~, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], recoveredGoal);
            forwBackVel = 0;
            if abs(rotateRightVel) < pi/16
                faceGoal = false;
            end
        else
            % Send order to stop
            must_stop = true;
            % Check it stopped
            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0 || inflatedObstMat(posInGrid(1),posInGrid(2)) == 1
                % Creates set of nodes with index for graph
                size_passPts = size(passPts);
                % If in recovery mode, keep the old goal
                if inRecovery
                    nodePts = [[youbotMapPos(1) youbotMapPos(2) 0]; [recoveredGoal 0]; [passPts, zeros(size_passPts(1),1)]];
                    inRecovery = false;
                else
                    nodePts = [[youbotMapPos(1) youbotMapPos(2) 0]; [closest 0]; [passPts, zeros(size_passPts(1),1)]];
                end
                
                % Avoid starting path planning while the robot is on an inflated obstacle
                if inflatedObstMat(posInGrid(1),posInGrid(2)) == 1
                    % If we are on a red tile (inflated obst) move out of it
                    posFreeInGrid = getPosClosestFreeCell(posInGrid, inflatedObstMat);
                    posFree = grid2world(inflatedOccMap, posFreeInGrid);
                    [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], posFree);
                else
                    % Create graph
                    % INFO : In the version of Peter Corke's Robotic toolbox
                    % we were provided, PGraph's "delete_node" function does
                    % NOT work. So there is no way to keep the graph in memory
                    % and update it each time. I have tried doing it with
                    % matlab graph and it is even slower than what is done
                    % below so I kept it this way
                    g = PGraph();
                    
                    % Add passage points
                    size_nodePts = size(nodePts);
                    for i = 1 : size_nodePts(1)
                        g.add_node(nodePts(i,1:2));
                    end
                    
                    % Create the edges between passage points
                    for k = 1:size_nodePts(1)
                        for j = k:size_nodePts(1)
                            if isInReach(nodePts(k,1:2),nodePts(j,1:2),occMap, inflatedOccMap)
                                g.add_edge(k,j);
                                g.add_edge(j,k);
                            end
                        end
                    end
                    
                    % Use Astar to find the shortest path to the goal
                    gPath = g.Astar(1,2);
                    
                    % Translate path
                    size_gPath = size(gPath);
                    path = zeros(size_gPath(2), 2);
                    for k = 1 : size_gPath(2)
                        path(k,:) = g.coord(gPath(k)).';
                    end
                    
                    % Setup path
                    iPath = 2;
                    currentlyInPath = true;
                    currentlyInPathIter = 0;
                    
                    % Check that the path is not empty
                    % If it is, then compute a graph where the edges between
                    % the nodes can go through inflated obstacles
                    if isempty(path)
                        fprintf("Error : Not able to find a path in the graph to reach the closest point\n");
                        fprintf("Check code comments for more info on possible causes\n");
                        % This might happen when the robot can see through a
                        % tight space while not being able to reach the other
                        % side (it might also happen when the robot could
                        % physically go through that space but identifies the
                        % cells as inflated obstacles)
                        setOccupancy(occMap, closest, 1); % Set closest as a wall
                        plotMapM1(fig, obstMat, inflatedOccMap, inflatedObstMat, accUnex, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
                        pause(0.0001)
                    end
                end
                % Robot can move again
                must_stop = false;
            end
        end
    else
        %%% Manipulation milestone %%%
        if init_manip && h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
            %% Initialisation
            fprintf("Starting manipulation \n");
            
            % Reset states from exploration
            faceGoal = false;
            lookingInReach = false;
            currentlyInPath = false;
            must_stop = false;
            
            [row1,col1] = find(obstMat == 0 );
            plot(row1,col1,'.g','markersize', 10);
            hold on
            
            [row,col] = find(inflatedObstMat == 1 );
            plot(row,col,'.r','markersize', 10);
            
            [row2,col2] = find(obstMat == 1 );
            plot(row2,col2,'.k','markersize', 10);
            
            %% Detect the tables
            % Starting values
            radii = [];
            sensitivity = 0.88;
            incrementS = 0.01;
            minS = 0.0;
            maxS = 1.0;
            
            imgInflate = 6;
            occMatResize = imresize(inflatedObstMat, imgInflate);
            
            % Try to find 3 tables
            while length(radii) ~= 3
                % Stop if increment too small without being able to
                % find 3 tables
                if incrementS < 0.001
                    fprintf("Unable to find 3 tables : increment now too small (%f)", incrementS);
                    break;
                end
                
                % Look for circles
                [centers, radii] = imfindcircles(occMatResize, [3*imgInflate, 4*imgInflate], 'Method', 'TwoStage', 'Sensitivity', sensitivity);
                
                fprintf("Number of tables = %d (sensitivity = %f)\n", length(radii), sensitivity);
                
                % Too few table must increase sensitivity
                if length(radii) < 3
                    % Never go below actual value
                    minS = sensitivity;
                    
                    % Decrease increment if above a value already giving too much circles
                    if sensitivity + incrementS >= maxS
                        incrementS = incrementS/2;
                    end
                    
                    sensitivity = sensitivity + incrementS;
                    
                    % Too many tables must decrease sensitivity
                else
                    % Never go above actual value
                    maxS = sensitivity;
                    
                    % Decrease increment if below a value already giving not enough circles
                    if sensitivity - incrementS <= minS
                        incrementS = incrementS/2;
                    end
                    
                    sensitivity = sensitivity - incrementS;
                end
            end
            
            % Tables centers coordinates (in grid but not rounded)
            tables_center = centers/imgInflate;
            tables_center_to_id = [tables_center(:,2) tables_center(:,1)];
            tables_radii = radii/imgInflate;
            
            % Plot circles
            for i=1:length(radii)
                circle([tables_center(i,2), tables_center(i,1)],tables_radii(i));
            end
            
            % Check that there is 3 tables
            if length(radii) ~= 3
                fprintf("Incorrect number of tables (nb = %d)\n", length(radii));
                break;
            end
            
            %% Setup states for identification
            inTravel = false;
            facePoint = false;
            goalPlanning = true;
            table_2_empty = false;
            task0_completed = false;
            task1_completed = false;
            task2_completed = false;
            task3_completed = false;
            task4_completed = false;
            task5_completed = false;
            
            % Task main grasp
            task1_main_grasp_completed = false;
            task2_main_grasp_completed = false;
            task3_main_grasp_completed = false;
            task4_main_grasp_completed = false;
            task5_main_grasp_completed = false;
            task6_main_grasp_completed = false;
            task7_main_grasp_completed = false;
            task8_main_grasp_completed = false;
            
            % Task grapsing
            task1_grasp_completed = false;
            task2_grasp_completed = false;
            task3_grasp_completed = false;
            task4_grasp_completed = false;
            task5_grasp_completed = false;
            task6_grasp_completed = false;
            task7_grasp_completed = false;
            task8_grasp_completed = false;
            task9_grasp_completed = false;
            task10_grasp_completed = false;
            task11_grasp_completed = false;
            task12_grasp_completed = false;
            task13_grasp_completed = false;
            task14_grasp_completed = false;
            task15_grasp_completed = false;
            task16_grasp_completed = false;
            task17_grasp_completed = false;
            task18_grasp_completed = false;
            task19_grasp_completed = false;
            task20_grasp_completed = false;
            task21_grasp_completed = false;
            task22_grasp_completed = false;
            task23_grasp_completed = false;
            task24_grasp_completed = false;
            table_empty = false;
            nbObj = 0;
            
            % Setup variables
            tables_info = zeros(3,3);
            
            % End the initialisation of the manipulation
            init_manip = false;
        else
            % Stop
            forwBackVel = 0;
            rightVel = 0;
            rotateRightVel = 0;
        end
        
        %% States for travelling across the map
        if inTravel
            %% Path planning
            % Check that path is not empty
            if isempty(path)
                currentlyInPath = false;
            end
            
            % If in path, follow it and upadte when you reach an edge
            if currentlyInPath
                size_path = size(path);
                goal = path(size_path(1),1:2);
                goalInGrid = world2grid(inflatedOccMap, goal);
                
                % If the end goal is a wall or was explored stop the planning
                if inflatedObstMat(goalInGrid(1),goalInGrid(2)) == 1
                    currentlyInPath = false;
                    
                    % Recovery mode : If we are in the path for too long without progress, restart planer with the same goal
                    % The recovery mode will be aborted if a closest node is in reach
                elseif currentlyInPathIter > 200
                    currentlyInPathIter = 0;
                    recoveredGoal = goal;
                    inRecovery = true;
                    currentlyInPath = false;
                    
                    % If in reach of the last node (unexplored)
                elseif iPath == size_path(1)
                    currentlyInPath = false;
                    posNextInReach = goal;
                    lookingInReach = true;
                    % Else follow the path
                else
                    pos = [youbotMapPos(1) youbotMapPos(2)];
                    coordNextNode = path(iPath,:);
                    % If still far, follow the same point
                    if norm(coordNextNode - pos) > 0.05
                        [forwBackVel, rightVel, rotateRightVel] = go_to_location(pos, youbotMapEuler3, [0 -1], coordNextNode);
                        currentlyInPathIter = currentlyInPathIter + 1;
                        % Check that objective is still in sight
                        if ~isInReach([youbotMapPos(1) youbotMapPos(2)], coordNextNode, occMap, inflatedOccMap)
                            lookingInReach = false;
                            currentlyInPath = false;
                        end
                    else % If close go to the next point in path
                        iPath = iPath +1;
                        currentlyInPathIter = 0;
                        % Stop
                        forwBackVel = 0;
                        rightVel = 0;
                        rotateRightVel = 0;
                    end
                end
                % Robot can move again
                must_stop = false;
                
                % If looking for goal in reach don't update instantly the next point to go
            elseif lookingInReach
                % Go to the location
                [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], objective);
                % Check that we have arrived at destiantion
                if norm([youbotMapPos(1) youbotMapPos(2)] - objective) < 0.05
                    % Stop travelling
                    inTravel = false;
                    lookingInReach = false;
                    goalPlanning = true;
                    % Stop moving
                    forwBackVel = 0;
                    rightVel = 0;
                    rotateRightVel = 0;
                end
                % Check that objective is still in sight
                if ~isInReach([youbotMapPos(1) youbotMapPos(2)], objective, occMap, inflatedOccMap)
                    lookingInReach = false;
                    currentlyInPath = false;
                    forwBackVel = 0;
                    rightVel = 0;
                    rotateRightVel = 0;
                end
                % Robot can move again
                must_stop = false;
                
                % If goal is in reach go into lookingInReach mode
            elseif isInReach([youbotMapPos(1) youbotMapPos(2)], objective, occMap, inflatedOccMap)
                posNextInReach = objective;
                [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], objective);
                lookingInReach = true;
                % We consider the recevory to be aborted if the closest point is in reach
                inRecovery = false;
                % Robot can move again
                must_stop = false;
            else
                % Send order to stop
                must_stop = true;
                % Check it stopped
                if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0 || inflatedObstMat(posInGrid(1),posInGrid(2)) == 1
                    % Creates set of nodes with index for graph
                    size_passPts = size(passPts);
                    % If in recovery mode, keep the old goal
                    if inRecovery
                        nodePts = [[youbotMapPos(1) youbotMapPos(2) 0]; [recoveredGoal 0]; [passPts, zeros(size_passPts(1),1)]];
                        inRecovery = false;
                    else
                        nodePts = [[youbotMapPos(1) youbotMapPos(2) 0]; [objective 0]; [passPts, zeros(size_passPts(1),1)]];
                    end
                    
                    % Avoid starting path planning while the robot is on an inflated obstacle
                    if inflatedObstMat(posInGrid(1),posInGrid(2)) == 1
                        % If we are on a red tile (inflated obst) move out of it
                        posFreeInGrid = getPosClosestFreeCell(posInGrid, inflatedObstMat);
                        posFree = grid2world(inflatedOccMap, posFreeInGrid);
                        [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], posFree);
                    else
                        % Create graph
                        % INFO : In the version of Peter Corke's Robotic toolbox
                        % we were provided, PGraph's "delete_node" function does
                        % NOT work. So there is no way to keep the graph in memory
                        % and update it each time. I have tried doing it with
                        % matlab graph and it is even slower than what is done
                        % below so I kept it this way
                        g = PGraph();
                        
                        % Add passage points
                        size_nodePts = size(nodePts);
                        for i = 1 : size_nodePts(1)
                            g.add_node(nodePts(i,1:2));
                        end
                        
                        % Create the edges between passage points
                        for k = 1:size_nodePts(1)
                            for j = k:size_nodePts(1)
                                if isInReach(nodePts(k,1:2),nodePts(j,1:2),occMap, inflatedOccMap)
                                    g.add_edge(k,j);
                                    g.add_edge(j,k);
                                end
                            end
                        end
                        
                        % Use Astar to find the shortest path to the goal
                        gPath = g.Astar(1,2);
                        
                        % Translate path
                        size_gPath = size(gPath);
                        path = zeros(size_gPath(2), 2);
                        for k = 1 : size_gPath(2)
                            path(k,:) = g.coord(gPath(k)).';
                        end
                        
                        % Setup path
                        iPath = 2;
                        currentlyInPath = true;
                        currentlyInPathIter = 0;
                        
                        % Check that the path is not empty
                        % If it is, then compute a graph where the edges between
                        % the nodes can go through inflated obstacles
                        if isempty(path)
                            fprintf("Error : Not able to find a path in the graph to reach the closest point\n");
                            fprintf("Check code comments for more info on possible causes\n");
                            % This might happen when the robot can see through a
                            % tight space while not being able to reach the other
                            % side (it might also happen when the robot could
                            % physically go through that space but identifies the
                            % cells as inflated obstacles)
                            
                            % Recompute objective
                            if task2_main_grasp_completed && ~task6_main_grasp_completed
                                % If we are going to table 2
                                center_table_2 = tables_info(2,1:2);
                                % Find the closest point 60cm away from the center
                                % of the table 2
                                r_to_table = r_t + 0.6; % m
                                vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - center_table_2;
                                dist_to_table = norm(vec_to_table);
                                ptInWorld = center_table_2 + vec_to_table/dist_to_table * r_to_table;
                                cIngrid = world2grid(inflatedOccMap, center_table_2);
                                ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                                
                                % Check that ptInWorld is not a wall
                                j = 1;
                                while inflatedObstMat(ptInGrid(1),ptInGrid(2)) == 1
                                    fprintf("Pt to reach is a wall, recomputing...\n")
                                    % If it is a wall, rotate until valid point is
                                    % found
                                    alpha = pi/16 * j;
                                    rot_mat_2d = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
                                    new_v = vec_to_table * rot_mat_2d;
                                    ptInWorld = center_table_2 + new_v/dist_to_table * r_to_table;
                                    ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                                    
                                    j = j+1;
                                end
                                
                                hold on
                                plot(ptInGrid(1),ptInGrid(2),'Color','c','Marker', '*', 'MarkerSize', 10);
                                plot(cIngrid(1),cIngrid(2),'Color','c','Marker', 'p', 'MarkerSize', 10);
                                
                            elseif task2_main_grasp_completed && task6_main_grasp_completed
                                % If we are going to table 1
                                center_table_1 = tables_info(1,1:2);
                                % Find the closest point 60cm away from the center
                                % of the table 1
                                r_to_table = r_t + 0.6; % m
                                vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - center_table_1;
                                dist_to_table = norm(vec_to_table);
                                ptInWorld = center_table_1 + vec_to_table/dist_to_table * r_to_table;
                                cIngrid = world2grid(inflatedOccMap, center_table_1);
                                ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                                
                                % Check that ptInWorld is not a wall
                                j = 1;
                                while inflatedObstMat(ptInGrid(1),ptInGrid(2)) == 1
                                    fprintf("Pt to reach is a wall, recomputing...\n")
                                    % If it is a wall, rotate until valid point is
                                    % found
                                    alpha = pi/16 * j;
                                    rot_mat_2d = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
                                    new_v = vec_to_table * rot_mat_2d;
                                    ptInWorld = center_table_1 + new_v/dist_to_table * r_to_table;
                                    ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                                    
                                    j = j+1;
                                end
                                
                                hold on
                                plot(ptInGrid(1),ptInGrid(2),'Color','c','Marker', '*', 'MarkerSize', 10);
                                plot(cIngrid(1),cIngrid(2),'Color','c','Marker', 'p', 'MarkerSize', 10);
                            else
                                fprintf("Recomputing went wrong...\n")
                            end
                            plotMapM2(fig, obstMat, inflatedOccMap, inflatedObstMat, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
                            pause(0.0001)
                        end
                    end
                    % Robot can move again
                    must_stop = false;
                end
            end
        elseif facePoint
            %% Face in a certain direction and stop
            [~, ~, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], ptToFace);
            forwBackVel = 0;
            rightVel = 0;
            if abs(rotateRightVel) < pi/64
                facePoint = false;
                rotateRightVel = 0;
            end
        elseif goalPlanning
            %% Plan the goal to reach
            
            %% Identifying tables
            if ~isempty(tables_center_to_id)
                
                if ~task0_completed
                    %% Compute the next goal
                    % Find the closest table (from remaining tables to
                    % identify)
                    vect_tables = posInGrid - tables_center_to_id;
                    dist_tables = sqrt(vect_tables(:,1).^2 + vect_tables(:,2).^2);
                    [min_dist, index_min] = min(dist_tables);
                    target_table_center = tables_center_to_id(index_min,:);
                    
                    % Find the closest point 1 m away from the center of the table
                    r_to_table = 1.3; % m
                    pt = target_table_center + vect_tables(index_min,:)/min_dist * r_to_table/worldResolution;
                    ptInGrid = [round(pt(1)) round(pt(2))];
                    
                    % Plot
                    hold on
                    plot(target_table_center(1),target_table_center(2), 'Color','y','Marker', 'p', 'MarkerSize', 10);
                    plot(ptInGrid(1),ptInGrid(2), 'Color','y','Marker', '*', 'MarkerSize', 10);
                    pause(1)
                    task0_completed = true;
                elseif ~task1_completed
                    % Send the robot to the pos
                    objective = grid2world(inflatedOccMap, ptInGrid);
                    inTravel = true;
                    task1_completed = true;
                    
                elseif ~task2_completed
                    % Face the center of the table
                    ptToFaceInGrid = [round(target_table_center(1)) round(target_table_center(2))];
                    ptToFace = grid2world(inflatedOccMap, ptToFaceInGrid);
                    facePoint = true;
                    task2_completed = true;
                    allowed_sensor_update = false;
                    
                elseif ~task3_completed
                    % Recompute the center thanks to a 3d scan
                    pts_triple_scan = scan_3d(vrep, id, h, 0, pi/8, 3, true, true);
                    [r_t, center_t] = find_table_center(pts_triple_scan, 0);
                    center_t_world = homtrans(trfStartingPos, [center_t(1) -center_t(2) 0].');
                    center_t_inGrid = world2grid(inflatedOccMap, [center_t_world(1) center_t_world(2)]);
                    
                    hold on
                    plot(center_t_inGrid(1),center_t_inGrid(2),'Color','c','Marker', 'p', 'MarkerSize', 10);
                    
                    % Find the closest point 60cm away from the center of the table
                    r_to_table = r_t + 0.6; % m
                    vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - [center_t_world(1) center_t_world(2)];
                    dist_to_table = sqrt(vec_to_table(1).^2 + vec_to_table(2).^2);
                    ptInWorld = [center_t_world(1) center_t_world(2)] + vec_to_table/dist_to_table * r_to_table;
                    
                    hold on
                    ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                    plot(ptInGrid(1),ptInGrid(2),'Color','c','Marker', '*', 'MarkerSize', 10);
                    
                    task3_completed = true;
                    
                elseif ~task4_completed
                    % Move the robot closer
                    [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], ptInWorld);
                    % Check for arrival
                    if all(abs([youbotMapPos(1) youbotMapPos(2)] - ptInWorld) < 0.05)
                        if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                            task4_completed = true;
                            % Face the center of the table
                            ptToFace = [center_t_world(1) center_t_world(2)];
                            facePoint = true;
                        end
                        forwBackVel = 0;
                        rightVel = 0;
                        rotateRightVel = 0;
                    end
                    
                elseif ~task5_completed
                    % Make triple scans to identify the table
                    pts_triple_scan = scan_3d(vrep, id, h, 0, pi/8, 2, true, true);
                    
                    objectpts = pts_triple_scan([1,3], pts_triple_scan(2,:) > -0.03);
                    objectpts = uniquetol(objectpts', 0.001, 'ByRows', true)';
                    
                    % Check for empty table
                    if isempty(objectpts)
                        id_table = 1;
                    else
                        % Compute the sum of the variance
                        var_obj = sum(var(objectpts.'));
                        
                        % Check for easy table
                        if var_obj > 0.01
                            id_table = 2;
                            % Check for hard table
                        else
                            id_table = 3;
                        end
                    end
                    
                    % Register info about that table
                    fprintf("id_table = %d\n", id_table);
                    centerInGrid = [round(target_table_center(1)) round(target_table_center(2))];
                    centerInWorld = grid2world(inflatedOccMap, centerInGrid);
                    tables_info(id_table,:) = [centerInWorld(1) centerInWorld(2) r_t];
                    
                    % Update tables_center_to_id
                    tables_center_to_id(index_min,:) = [];
                    
                    % Reset the states of the identification
                    inTravel = false;
                    facePoint = false;
                    goalPlanning = true;
                    task0_completed = false;
                    task1_completed = false;
                    task2_completed = false;
                    task3_completed = false;
                    task4_completed = false;
                    task5_completed = false;
                    allowed_sensor_update = true;
                end
                
            elseif ~table_2_empty
                %%% Move objects between table 2 and 1 %%%
                %% Go to table 2
                if ~task1_main_grasp_completed
                    % Compute goal to table 2
                    center_table_2 = tables_info(2,1:2);
                    % Find the closest point 60cm away from the center
                    % of the table 2
                    r_to_table = r_t + 0.6; % m
                    vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - center_table_2;
                    dist_to_table = norm(vec_to_table);
                    ptInWorld = center_table_2 + vec_to_table/dist_to_table * r_to_table;
                    cIngrid = world2grid(inflatedOccMap, center_table_2);
                    ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                    
                    % Check that ptInWorld is not a wall
                    j = 1;
                    while inflatedObstMat(ptInGrid(1),ptInGrid(2)) == 1
                        fprintf("Pt to reach is a wall, recomputing...\n")
                        % If it is a wall, rotate until valid point is
                        % found
                        alpha = pi/16 * j;
                        rot_mat_2d = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
                        new_v = vec_to_table * rot_mat_2d;
                        ptInWorld = center_table_2 + new_v/dist_to_table * r_to_table;
                        ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                        
                        j = j+1;
                    end
                    
                    hold on
                    plot(ptInGrid(1),ptInGrid(2),'Color','c','Marker', '*', 'MarkerSize', 10);
                    plot(cIngrid(1),cIngrid(2),'Color','c','Marker', 'p', 'MarkerSize', 10);
                    
                    task1_main_grasp_completed = true;
                    
                elseif ~task2_main_grasp_completed
                    % Send the robot to the table 2
                    objective = ptInWorld;
                    inTravel = true;
                    task2_main_grasp_completed = true;
                    
                elseif ~task3_main_grasp_completed
                    % Face the center of table 2
                    [~, ~, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], center_table_2);
                    forwBackVel = 0;
                    rightVel = 0;
                    if abs(rotateRightVel) < pi/32
                        rotateRightVel = 0;
                        task3_main_grasp_completed = true;
                        allowed_sensor_update = false;
                    end
                elseif ~task4_main_grasp_completed
                    %% Grab object %%
                    %% Anchor
                    if ~task1_grasp_completed
                        % Compute the center and point close to it
                        pts_triple_scan = scan_3d(vrep, id, h, 0, pi/8, 2, true, true);
                        [r_t, center_t] = find_table_center(pts_triple_scan, 0);
                        center_t_world = homtrans(trfStartingPos, [center_t(1) -center_t(2) 0].');
                        
                        % Find the closest point r + 60cm away from the center of the table
                        r_to_table = r_t + 0.5; % m
                        vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - [center_t_world(1) center_t_world(2)];
                        dist_to_table = sqrt(vec_to_table(1).^2 + vec_to_table(2).^2);
                        ptInWorld = [center_t_world(1) center_t_world(2)] + vec_to_table/dist_to_table * r_to_table;
                        
                        task1_grasp_completed = true;
                        
                    elseif ~task2_grasp_completed
                        % Move the robot closer
                        [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], ptInWorld);
                        % Check for arrival
                        if all(abs([youbotMapPos(1) youbotMapPos(2)] - ptInWorld) < 0.05)
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task2_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                        
                    elseif ~task3_grasp_completed
                        %% Face in a certain direction and stop
                        [~, ~, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], [center_t_world(1) center_t_world(2)]);
                        forwBackVel = 0;
                        rightVel = 0;
                        if abs(rotateRightVel) < pi/32
                            rotateRightVel = 0;
                            angle_to_rotate = youbotMapEuler3 - pi/2;
                            task3_grasp_completed = true;
                        end
                        
                    elseif ~task4_grasp_completed
                        % Rotate 90°
                        forwBackVel = 0;
                        rightVel = 0;
                        rotateRightVel = angdiff(angle_to_rotate, youbotMapEuler3);
                        % Check for arrival
                        if abs(angdiff(angle_to_rotate, youbotMapEuler3)) < 0.01
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task4_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                    elseif ~task5_grasp_completed
                        % Slide closer
                        % Find the distance between robot and r + 0.3 cm of the table
                        r_to_table = r_t + 0.3; % m
                        vec_to_table = [center_t_world(1) center_t_world(2)] - [youbotMapPos(1) youbotMapPos(2)];
                        dist_to_table = norm(vec_to_table) - r_to_table;
                        vec_to_pt = vec_to_table / norm(vec_to_table) * (dist_to_table);
                        sideVector = [-1 0] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
                        rightVel = -dist_to_table * sign(dot(vec_to_pt, sideVector));
                        forwBackVel = 0;
                        rotateRightVel = 0;
                        % Check for arrival
                        if abs(dist_to_table) < 0.01
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task5_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                    elseif ~task6_grasp_completed
                        % Compute error
                        cam_angle = pi/2;
                        pts_cloud = scan_3d(vrep, id, h, cam_angle, pi/8, 1, true, true);
                        [dist_center, tilt, r_t] = errAnchor(pts_cloud, cam_angle);
                        r_to_table = r_t + 0.30;
                        
                        % Correct error
                        forwBackVel = 0;
                        rightVel = (dist_center - r_to_table);
                        rotateRightVel = -tilt;
                        
                        % Check error corrected
                        if abs(dist_center - r_to_table) < 0.005 && abs(tilt) < 0.005
                            task6_grasp_completed = true;
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                    elseif ~task7_grasp_completed
                        % Search for objects
                        camera_angle = pi - atan(r_to_table/0.25);
                        pts_cloud = scan_3d(vrep, id, h, camera_angle, pi/8, 1.5, true, true);
                        [table_empty, angle, dist_to_obj] = next_object(pts_cloud, camera_angle, r_to_table, plotGrasp);
                        angle_to_reach = youbotMapEuler3 + angle;
                        task7_grasp_completed = true;
                        
                    elseif table_empty
                        %% Stop if no more object on table 2
                        fprintf("Table 2 is empty\n");
                        fprintf("Manipulation milestone completed\n");
                        break;
                        
                    elseif ~task8_grasp_completed
                        % Rotate around table to the object
                        [forwBackVel, rightVel, rotateRightVel] = drive_circle(r_to_table, 0.3, angdiff(angle_to_reach, youbotMapEuler3), 0.01);
                        if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0 && abs(angdiff(angle_to_reach,youbotMapEuler3)) < 0.01
                            task8_grasp_completed = true;
                        end
                        
                    elseif ~task9_grasp_completed
                        % Compute error
                        cam_angle = pi/2;
                        pts_cloud = scan_3d(vrep, id, h, cam_angle, pi/8, 1, true, true);
                        [dist_center, tilt, r_t] = errAnchor(pts_cloud, cam_angle);
                        r_to_table = r_t + 0.30;
                        
                        % Correct error
                        forwBackVel = 0;
                        rightVel = (dist_center - r_to_table);
                        rotateRightVel = -tilt;
                        
                        % Check error corrected
                        if abs(dist_center - r_to_table) < 0.005 && abs(tilt) < 0.005
                            task9_grasp_completed = true;
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                    elseif ~task10_grasp_completed
                        % Search alignement with object
                        % New camera angle to the obj
                        camera_angle = pi - atan(dist_to_obj/0.25);
                        pts_single = scan_3d(vrep, id, h, camera_angle, pi/12, 1, true, false);
                        pts_large = scan_3d(vrep, id, h, camera_angle, pi/8, 1, true, true);
                        
                        [r_to_table, obj_angle, grab_dist, grab_angle, readyForGrab] = get_objects(pts_single, pts_large, camera_angle, r_to_table, plotGrasp);
                        fprintf('Grab dist = %f\n Angle = %f\n', grab_dist, grab_angle);
                        % If already ready to grab, skip some tasks
                        if readyForGrab
                            task11_grasp_completed = true;
                            task12_grasp_completed = true;
                            task13_grasp_completed = true;
                            task14_grasp_completed = true;
                            fsm_grasp = "starting";
                        else
                            angle_to_reach = youbotMapEuler3 + obj_angle;
                        end
                        task10_grasp_completed = true;
                    elseif ~task11_grasp_completed
                        % Rotate around table to the object
                        [forwBackVel, rightVel, rotateRightVel] = drive_circle(r_to_table, 0.3, angdiff(angle_to_reach, youbotMapEuler3), 0.01);
                        if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0 && abs(angdiff(angle_to_reach,youbotMapEuler3)) < 0.01
                            frontVector = [0 -1] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
                            pos_to_go = [youbotMapPos(1) youbotMapPos(2)] + frontVector * 0.168;
                            task11_grasp_completed = true;
                        end
                    elseif ~task12_grasp_completed
                        % Forward to align robot arm
                        new_vect = pos_to_go - [youbotMapPos(1) youbotMapPos(2)];
                        forwBackVel = -norm(new_vect) * sign(dot(frontVector, new_vect));
                        rightVel = 0;
                        rotateRightVel = 0;
                        
                        % Check for arrival
                        if abs(norm(new_vect)) < 0.005
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                fsm_grasp = "starting";
                                task12_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                        
                    elseif ~task13_grasp_completed
                        % last adjustement
                        cam_to_object_squared = 0.4162^2 + grab_dist^2 - 2*0.4162*grab_dist*cos(grab_angle);
                        cam_angle = pi - acos((0.4162^2 + cam_to_object_squared - grab_dist^2)/(2*0.4162*sqrt(cam_to_object_squared)));
                        
                        pts_single = scan_3d(vrep, id, h, cam_angle, pi/12, 1, true, false);
                        
                        [front_adjust, grab_dist, grab_angle] = adjust_object(pts_single, cam_angle, sqrt(cam_to_object_squared), plotGrasp);
                        
                        frontVector = [0 -1] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
                        pos_to_go = [youbotMapPos(1) youbotMapPos(2)] + frontVector * front_adjust;
                        
                        task13_grasp_completed = true;
                        
                    elseif ~task14_grasp_completed
                        % Forward to align robot arm
                        new_vect = pos_to_go - [youbotMapPos(1) youbotMapPos(2)];
                        forwBackVel = -norm(new_vect) * sign(dot(frontVector, new_vect));
                        rightVel = 0;
                        rotateRightVel = 0;
                        
                        % Check for arrival
                        if abs(norm(new_vect)) < 0.005
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                fsm_grasp = "starting";
                                task14_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                        
                    elseif ~task15_grasp_completed
                        % Grab the object
                        fsm_grasp = grasping(vrep, id, h, fsm_grasp, grab_angle, grab_dist+0.01);
                        
                        if strcmp(fsm_grasp, "finished")
                            task15_grasp_completed = true;
                            task4_main_grasp_completed = true;
                            allowed_sensor_update = true;
                        end
                    end
                    
                elseif ~task5_main_grasp_completed
                    %% Go to table 1
                    % Compute goal to table 1
                    center_table_1 = tables_info(1,1:2);
                    % Find the closest point 60cm away from the center
                    % of the table 2
                    r_to_table = r_t + 0.6; % m
                    vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - center_table_1;
                    dist_to_table = norm(vec_to_table);
                    ptInWorld = center_table_1 + vec_to_table/dist_to_table * r_to_table;
                    cIngrid = world2grid(inflatedOccMap, center_table_1);
                    ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                    
                    % Check that ptInWorld is not a wall
                    j = 1;
                    while inflatedObstMat(ptInGrid(1),ptInGrid(2)) == 1
                        fprintf("Pt to reach is a wall, recomputing...\n")
                        % If it is a wall, rotate until valid point is
                        % found
                        alpha = pi/16 * j;
                        rot_mat_2d = [cos(alpha), sin(alpha); -sin(alpha), cos(alpha)];
                        new_v = vec_to_table * rot_mat_2d;
                        ptInWorld = center_table_1 + new_v/dist_to_table * r_to_table;
                        ptInGrid = world2grid(inflatedOccMap, ptInWorld);
                        
                        j = j+1;
                    end
                    
                    hold on
                    plot(ptInGrid(1),ptInGrid(2),'Color','c','Marker', '*', 'MarkerSize', 10);
                    plot(cIngrid(1),cIngrid(2),'Color','c','Marker', 'p', 'MarkerSize', 10);
                    
                    task5_main_grasp_completed = true;
                    
                elseif ~task6_main_grasp_completed
                    % Send the robot to the table 1
                    objective = ptInWorld;
                    inTravel = true;
                    task6_main_grasp_completed = true;
                    
                elseif ~task7_main_grasp_completed
                    % Face the center of table 1
                    [~, ~, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], center_table_1);
                    forwBackVel = 0;
                    rightVel = 0;
                    if abs(rotateRightVel) < pi/32
                        rotateRightVel = 0;
                        task7_main_grasp_completed = true;
                        allowed_sensor_update = false;
                    end
                elseif ~task8_main_grasp_completed
                    %% Drop object %%
                    %% Anchor
                    if ~task16_grasp_completed
                        % Compute the center and point close to it
                        pts_triple_scan = scan_3d(vrep, id, h, 0, pi/8, 2, true, true);
                        [r_t, center_t] = find_table_center(pts_triple_scan, 0);
                        center_t_world = homtrans(trfStartingPos, [center_t(1) -center_t(2) 0].');
                        
                        % Find the closest point r + 60cm away from the center of the table
                        r_to_table = r_t + 0.5; % m
                        vec_to_table = [youbotMapPos(1) youbotMapPos(2)] - [center_t_world(1) center_t_world(2)];
                        dist_to_table = sqrt(vec_to_table(1).^2 + vec_to_table(2).^2);
                        ptInWorld = [center_t_world(1) center_t_world(2)] + vec_to_table/dist_to_table * r_to_table;
                        
                        task16_grasp_completed = true;
                        
                    elseif ~task17_grasp_completed
                        % Move the robot closer
                        [forwBackVel, rightVel, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], ptInWorld);
                        % Check for arrival
                        if all(abs([youbotMapPos(1) youbotMapPos(2)] - ptInWorld) < 0.05)
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task17_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                        
                    elseif ~task18_grasp_completed
                        % Face table center
                        [~, ~, rotateRightVel] = go_to_location([youbotMapPos(1) youbotMapPos(2)], youbotMapEuler3, [0 -1], [center_t_world(1) center_t_world(2)]);
                        forwBackVel = 0;
                        rightVel = 0;
                        if abs(rotateRightVel) < pi/32
                            rotateRightVel = 0;
                            angle_to_rotate = youbotMapEuler3 - pi/2;
                            task18_grasp_completed = true;
                        end
                        
                    elseif ~task19_grasp_completed
                        % Rotate 90°
                        forwBackVel = 0;
                        rightVel = 0;
                        rotateRightVel = angdiff(angle_to_rotate, youbotMapEuler3);
                        % Check for arrival
                        if abs(angdiff(angle_to_rotate, youbotMapEuler3)) < 0.01
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task19_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                    elseif ~task20_grasp_completed
                        % Slide closer
                        % Find the distance between robot and r + 0.3 cm of the table
                        r_to_table = r_t + 0.3; % m
                        vec_to_table = [center_t_world(1) center_t_world(2)] - [youbotMapPos(1) youbotMapPos(2)];
                        dist_to_table = norm(vec_to_table) - r_to_table;
                        vec_to_pt = vec_to_table / norm(vec_to_table) * (dist_to_table);
                        sideVector = [-1 0] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
                        rightVel = -dist_to_table * sign(dot(vec_to_pt, sideVector));
                        forwBackVel = 0;
                        rotateRightVel = 0;
                        % Check for arrival
                        if abs(dist_to_table) < 0.01
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task20_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                        end
                    elseif ~task21_grasp_completed
                        % Compute error
                        cam_angle = pi/2;
                        pts_cloud = scan_3d(vrep, id, h, cam_angle, pi/8, 1, true, true);
                        [dist_center, tilt, r_t] = errAnchor(pts_cloud, cam_angle);
                        r_to_table = r_t + 0.30;
                        
                        % Correct error
                        forwBackVel = 0;
                        rightVel = (dist_center - r_to_table);
                        rotateRightVel = -tilt;
                        
                        % Check error corrected
                        if abs(dist_center - r_to_table) < 0.005 && abs(tilt) < 0.005
                            task21_grasp_completed = true;
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                            frontVector = [0 -1] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
                            pos_to_go = [youbotMapPos(1) youbotMapPos(2)] + frontVector * 0.168;
                            if nbObj == 0
                                angle_ref = youbotMapEuler3;
                            end
                            angle_to_reach = angle_ref + nbObj * pi/6;
                        end
                        
                    elseif ~task22_grasp_completed
                        % Rotate around table to the object
                        [forwBackVel, rightVel, rotateRightVel] = drive_circle(r_to_table, 0.3, angdiff(angle_to_reach, youbotMapEuler3), 0.01);
                        if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0 && abs(angdiff(angle_to_reach,youbotMapEuler3)) < 0.01
                            frontVector = [0 -1] * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
                            pos_to_go = [youbotMapPos(1) youbotMapPos(2)] + frontVector * 0.168;
                            task22_grasp_completed = true;
                        end
                    elseif ~task23_grasp_completed
                        % Forward to align robot arm
                        new_vect = pos_to_go - [youbotMapPos(1) youbotMapPos(2)];
                        forwBackVel = -norm(new_vect) * sign(dot(frontVector, new_vect));
                        rightVel = 0;
                        rotateRightVel = 0;
                        
                        % Check for arrival
                        if abs(norm(new_vect)) < 0.005
                            if h.previousForwBackVel == 0 && h.previousLeftRightVel == 0 && h.previousRotVel == 0
                                task23_grasp_completed = true;
                            end
                            forwBackVel = 0;
                            rightVel = 0;
                            rotateRightVel = 0;
                            
                            fsm_replace = 'starting';
                        end
                        
                    elseif ~task24_grasp_completed
                        [fsm_replace] = replace_object(vrep, id, h, fsm_replace);
                        
                        if strcmp(fsm_replace, "finished")
                            allowed_sensor_update = true;
                            nbObj = nbObj + 1;
                            % Reset task
                            % task main grasp
                            task1_main_grasp_completed = false;
                            task2_main_grasp_completed = false;
                            task3_main_grasp_completed = false;
                            task4_main_grasp_completed = false;
                            task5_main_grasp_completed = false;
                            task6_main_grasp_completed = false;
                            task7_main_grasp_completed = false;
                            task8_main_grasp_completed = false;
                            
                            % task grapsing
                            task1_grasp_completed = false;
                            task2_grasp_completed = false;
                            task3_grasp_completed = false;
                            task4_grasp_completed = false;
                            task5_grasp_completed = false;
                            task6_grasp_completed = false;
                            task7_grasp_completed = false;
                            task8_grasp_completed = false;
                            task9_grasp_completed = false;
                            task10_grasp_completed = false;
                            task11_grasp_completed = false;
                            task12_grasp_completed = false;
                            task13_grasp_completed = false;
                            task14_grasp_completed = false;
                            task15_grasp_completed = false;
                            task16_grasp_completed = false;
                            task17_grasp_completed = false;
                            task18_grasp_completed = false;
                            task19_grasp_completed = false;
                            task20_grasp_completed = false;
                            task21_grasp_completed = false;
                            task22_grasp_completed = false;
                            task23_grasp_completed = false;
                            task24_grasp_completed = false;
                        end
                    end
                end
            end
        else
            % Stop
            forwBackVel = 0;
            rightVel = 0;
            rotateRightVel = 0;
        end
    end
    %% Plot map if required.
    if mod(itnb,100)==0
        if exploration_milestone
            plotMapM1(fig, obstMat, inflatedOccMap, inflatedObstMat, accUnex, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
        else
            plotMapM2(fig, obstMat, inflatedOccMap, inflatedObstMat, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach);
        end
    end
    
    %% Check for must stop and stop if necessary
    if must_stop
        forwBackVel = 0;
        rightVel = 0;
        rotateRightVel = 0;
    end
    
    %% Check for must stop and stop if necessary
    if must_stop_gps
        forwBackVel = 0;
        rightVel = 0;
        rotateRightVel = 0;
    end
    
    %% End of loop
    % Update wheel velocities using the global values (whatever the state is).
    h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
    
    % Increase interation nomber
    itnb = itnb + 1;
    
    % Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms).
    elapsed = toc;
    timeleft = timestep - elapsed;
    if timeleft > 0
        total_time = total_time + elapsed + min(timeleft, .01);
        pause(min(timeleft, .01));
    else
        fprintf("Excess time %dms\n", floor(elapsed*1000));
        total_time = total_time + elapsed;
    end
end
end

% Plot the map for the milestone 1
function plotMapM1(fig, obstMat, inflatedOccMap, inflatedObstMat, accUnex, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach)
set(0, 'CurrentFigure', fig);

[row1,col1] = find(obstMat == 0 );
plot(row1,col1,'.g','markersize', 10);
hold on

[row,col] = find(inflatedObstMat == 1 );
plot(row,col,'.r','markersize', 10);

[row2,col2] = find(obstMat == 1 );
plot(row2,col2,'.k','markersize', 10);

plot(accUnex(1,:),accUnex(2,:),'.c','markersize', 10);

plot(passPtsInGrid(1,:),passPtsInGrid(2,:),'.b','markersize', 10);

% Check that path is not empty
if isempty(path)
    currentlyInPath = false;
end

if currentlyInPath
    % Error may happen here : Error using world2grid Expected input number 2, xy, to be nonempty.
    pathInGrid = world2grid(inflatedOccMap, path);
    plot(pathInGrid(iPath:end,1), pathInGrid(iPath:end,2),'.y','markersize', 10);
    plot([posInGrid(1) pathInGrid(iPath,1)], [posInGrid(2) pathInGrid(iPath,2)], 'LineWidth', 2);
    plot(pathInGrid(iPath:end,1), pathInGrid(iPath:end,2), 'LineWidth', 2);
elseif lookingInReach
    posNextInReachInGrid = world2grid(inflatedOccMap, posNextInReach);
    plot(posNextInReachInGrid(1),posNextInReachInGrid(2),'.y','markersize', 10);
    plot([posInGrid(1) posNextInReachInGrid(1)], [posInGrid(2) posNextInReachInGrid(2)], 'LineWidth', 2);
end

plot(posInGrid(1),posInGrid(2),'.m','markersize', 10);

hold off
pause(0.00001);
end

% Plot the map for the milestone 2
function plotMapM2(fig, obstMat, inflatedOccMap, inflatedObstMat, passPtsInGrid, path, iPath, posInGrid, currentlyInPath, lookingInReach, posNextInReach)
set(0, 'CurrentFigure', fig);

[row1,col1] = find(obstMat == 0 );
plot(row1,col1,'.g','markersize', 10);
hold on

[row,col] = find(inflatedObstMat == 1 );
plot(row,col,'.r','markersize', 10);

[row2,col2] = find(obstMat == 1 );
plot(row2,col2,'.k','markersize', 10);

plot(passPtsInGrid(1,:),passPtsInGrid(2,:),'.b','markersize', 10);

% Check that path is not empty
if isempty(path)
    currentlyInPath = false;
end

if currentlyInPath
    % Error may happen here : Error using world2grid Expected input number 2, xy, to be nonempty.
    pathInGrid = world2grid(inflatedOccMap, path);
    plot(pathInGrid(iPath:end,1), pathInGrid(iPath:end,2),'.y','markersize', 10);
    plot([posInGrid(1) pathInGrid(iPath,1)], [posInGrid(2) pathInGrid(iPath,2)], 'LineWidth', 2);
    plot(pathInGrid(iPath:end,1), pathInGrid(iPath:end,2), 'LineWidth', 2);
elseif lookingInReach
    posNextInReachInGrid = world2grid(inflatedOccMap, posNextInReach);
    plot(posNextInReachInGrid(1),posNextInReachInGrid(2),'.y','markersize', 10);
    plot([posInGrid(1) posNextInReachInGrid(1)], [posInGrid(2) posNextInReachInGrid(2)], 'LineWidth', 2);
end

plot(posInGrid(1),posInGrid(2),'.m','markersize', 10);

hold off
pause(0.00001);
end

% Get the position (grid coord) of the extended vertex of the inflated obstacles
function vertex = get_vertex(inflatedObstMat)
size_map = size(inflatedObstMat);
vertex = zeros(2,size_map(1)*size_map(2));
k = 1;

for i = 2 : size_map(1)-1
    for j = 2 : size_map(2)-1
        % First check that the cell (i,j) is an obstacle
        if inflatedObstMat(i,j) == 1
            % Case 1 : Vertex in (i+1,j+1)
            if (inflatedObstMat(i+1,j+1)==0) && (inflatedObstMat(i,j-1)==1) && (inflatedObstMat(i-1,j)==1)  && (inflatedObstMat(i,j+1)~=1) && (inflatedObstMat(i+1,j)~=1)
                if ((inflatedObstMat(i+1,j-1)~=1) || (inflatedObstMat(i-1,j+1)~=1)) && ((inflatedObstMat(i,j+1)==0) || (inflatedObstMat(i+1,j)==0) || (inflatedObstMat(i+1,j-1)==0) || (inflatedObstMat(i-1,j+1)==0))
                    vertex(1,k) = i+1;
                    vertex(2,k) = j+1;
                    k=k+1;
                end
                % Case 2 : Vertex in (i+1,j-1)
            elseif (inflatedObstMat(i+1,j-1)==0) && (inflatedObstMat(i,j-1)~=1) && (inflatedObstMat(i-1,j)==1)  && (inflatedObstMat(i,j+1)==1) && (inflatedObstMat(i+1,j)~=1)
                if ((inflatedObstMat(i-1,j-1)~=1) || (inflatedObstMat(i+1,j+1)~=1)) && ((inflatedObstMat(i,j-1)==0) || (inflatedObstMat(i+1,j)==0) || (inflatedObstMat(i-1,j-1)==0) || (inflatedObstMat(i+1,j+1)==0))
                    vertex(1,k) = i+1;
                    vertex(2,k) = j-1;
                    k=k+1;
                end
                % Case 3 : Vertex in (i-1,j+1)
            elseif (inflatedObstMat(i-1,j+1)==0) && (inflatedObstMat(i,j-1)==1) && (inflatedObstMat(i-1,j)~=1)  && (inflatedObstMat(i,j+1)~=1) && (inflatedObstMat(i+1,j)==1)
                if ((inflatedObstMat(i-1,j-1)~=1) || (inflatedObstMat(i+1,j+1)~=1)) && ((inflatedObstMat(i-1,j)==0) || (inflatedObstMat(i,j+1)==0) || (inflatedObstMat(i-1,j-1)==0) || (inflatedObstMat(i+1,j+1)==0))
                    vertex(1,k) = i-1;
                    vertex(2,k) = j+1;
                    k=k+1;
                end
                % Case 4 : Vertex in (i-1,j-1)
            elseif (inflatedObstMat(i-1,j-1)==0) && (inflatedObstMat(i,j-1)~=1) && (inflatedObstMat(i-1,j)~=1)  && (inflatedObstMat(i,j+1)==1) && (inflatedObstMat(i+1,j)==1)
                if ((inflatedObstMat(i+1,j-1)~=1) || (inflatedObstMat(i-1,j+1)~=1)) && ((inflatedObstMat(i,j-1)==0) || (inflatedObstMat(i-1,j)==0) || (inflatedObstMat(i+1,j-1)==0) || (inflatedObstMat(i-1,j+1)==0))
                    vertex(1,k) = i-1;
                    vertex(2,k) = j-1;
                    k=k+1;
                end
            end
        end
    end
end

vertex = vertex(:,1:k-1);

end


% Get the position (grid coord) of the accessible cell that have an
% unexplored cell as neighbor (8 connectivity)
function accUnex = get_unexploredAccessible(inflatedObstMat)
size_map = size(inflatedObstMat);
accUnex = zeros(2,size_map(1)*size_map(2));
k = 1;

for i = 2 : size_map(1)-1
    for j = 2 : size_map(2)-1
        % First check that the cell (i,j) is an accessible cell
        if inflatedObstMat(i,j) == 0
            % Check that a 8 connectivity neighbor is an unexplored cell
            if (inflatedObstMat(i+1,j)==-1) || (inflatedObstMat(i-1,j)==-1) || (inflatedObstMat(i,j+1)==-1) || (inflatedObstMat(i,j-1)==-1) || (inflatedObstMat(i+1,j+1)==-1) || (inflatedObstMat(i-1,j+1)==-1) || (inflatedObstMat(i+1,j-1)==-1) || (inflatedObstMat(i-1,j-1)==-1)
                % Check that it is not inside a 4 connectivity obstacle by
                % checking that a free space is in 4 connectivity
                if (inflatedObstMat(i+1,j)==0) || (inflatedObstMat(i-1,j)==0) || (inflatedObstMat(i,j+1)==0) || (inflatedObstMat(i,j-1)==0)
                    accUnex(1,k) = i;
                    accUnex(2,k) = j;
                    k=k+1;
                end
            end
        end
    end
end

accUnex = accUnex(:,1:k-1);

end

% Get the position (grid coord) of the closest unexplored cell that has an
% accessible cell as neighbor (4 connectivity)
% If none exist return [NaN ; NaN]
function closest = get_closestUnexploredAccessible(pos, accUnex, distToClose)
size_accUnex = size(accUnex);
closest = [NaN ; NaN];
minDist = Inf;

for j = 1 : size_accUnex(2)
    pt = accUnex(:,j);
    dist = pdist([pt(1),pt(2);pos(1) pos(2)]);
    if (dist < minDist) && (dist > distToClose)
        minDist = dist;
        closest = pt;
    end
end

end

% Get the position (in grid) of the closest free cell
function posFree = getPosClosestFreeCell(posInGrid, inflatedObstMat)
posFree = [NaN NaN];
for i = -2:2
    for j = -2:2
        if(inflatedObstMat(posInGrid(1)+i, posInGrid(2)+j)==0)
            posFree = [posInGrid(1)+i, posInGrid(2)+j];
            return;
        end
    end
end
end

% Check that a cell has been explored
function explo = isExplored(posInGrid, inflatedObstMat)
explo = true;
for i = -1:1
    for j = -1:1
        if(inflatedObstMat(posInGrid(1)+i, posInGrid(2)+j)==-1)
            explo = false;
            return;
        end
    end
end
end

% Check that there two point are in reach :
% In other words, that no obstacle or unknown cell are between p1 and p2
function bool = isInReach(p1, p2, obstMap, inflatedOccMap)
% Get cells between them
[~,midPts] = raycast(obstMap,p1,p2);

% Check not empty and get the values of each cell
if not(isempty(midPts))
    midPtsInWorld = grid2world(inflatedOccMap, midPts);
    midPtsVal = checkOccupancy(inflatedOccMap, [midPtsInWorld(:,1) midPtsInWorld(:,2)]);
else
    midPtsVal = 0;
end

% Check that they are not obstacle or unknown cells
if isempty(midPtsVal(midPtsVal~=0))
    bool = true;
else
    bool = false;
end
end

% Control the speed of the weels to go to a target location (supposing that
% there is no wall between youbot and the target location)
function [forwBackVel, rightVel, rotateRightVel] = go_to_location(youbotMapPos2D, youbotMapEuler3, refVector, goal)
%% Angle of rotation to reach the goal
% Rotate the reference vector by the orientation
frontVector = refVector * [cos(-youbotMapEuler3) -sin(-youbotMapEuler3); sin(-youbotMapEuler3) cos(-youbotMapEuler3)];
% Create vector from position to the goal
goalVector = goal - youbotMapPos2D;
% Angle between front vector and goal
CosTheta = max(min(dot(frontVector,goalVector)/(norm(frontVector)*norm(goalVector)),1),-1);
angleOfRotation = acos(CosTheta);
% Sign of that angle
crossProd = cross([frontVector 0],[goalVector 0]);
if crossProd(3) < 0
    angleOfRotation = -angleOfRotation;
end

%% Compute angular speed
rotateRightVel = angleOfRotation;
% Stop when the robot is close to the angle of rotation
if abs(angleOfRotation) < .002 || (abs(norm(goalVector)) < 0.025)
    rotateRightVel = 0;
end

%% Compute forward speed
if (abs(angleOfRotation) < pi/16) %&& (abs(norm(goalVector)) > 0.5)
    forwBackVel = -1*(cos(angleOfRotation)*abs(norm(goalVector)));
else
    forwBackVel = 0;
end
rightVel = 0;
end

% Simplify the pts returned by the hokuyo to make inpolygon faster
function poly = simplifyEdgeOfVision(pts2D)
nbrIter = 0;
poly = pts2D;
while true
    nbrChange = 0;
    nextPoly = zeros(2,size(poly,2));
    poly = horzcat(poly, poly(:,2));
    size_poly = size(poly);
    k = 1;
    i = 1;
    while i < size_poly(2) - 1
        p1 = poly(:,i);
        p2 = poly(:,i+1);
        
        % Skip the point if too close
        if norm(p2-p1) < 0.05
            i = i+2;
            % Compute the area between third point
        else
            p3 = poly(:,i+2);
            v1 = p2-p1;
            v2 = p3-p2;
            v1 = [v1(1) v1(2) 0]/norm(v1);
            v2 = [v2(1) v2(2) 0]/norm(v2);
            area = cross(v1,v2);
            area = abs(area(3));
            
            % If area is under 0.15 skip the point
            if area < 0.15
                i = i+2;
                nbrChange = nbrChange + 1;
            else
                i = i + 1;
            end
        end
        nextPoly(:,k) = p1;
        k = k+1;
    end
    
    nextPoly(:,k) = nextPoly(:,1);
    nextPoly = nextPoly(:,1:k);
    poly = nextPoly;
    
    nbrIter = nbrIter + 1;
    if nbrChange < 2 || nbrIter > 8
        break;
    end
end
end

% Take a 3D picture (points cloud)
function [pts] = scan_3d(vrep, id, h, middle_angle, open_angle, max_dist, no_wall, triple)

res = vrep.simxSetFloatSignal(id, 'rgbd_sensor_scan_angle', open_angle, vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error.



%% MIDDLE SCAN
% set the cam angle
vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, -pi/2 + middle_angle], vrep.simx_opmode_oneshot);
vrchk(vrep, res);

res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

% Then retrieve the last point cloud the depth sensor took.
% If you were to try to capture multiple images in a row, try other values than
% vrep.simx_opmode_oneshot_wait.
pts = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);

pts = pts(:, pts(4, :) < max_dist);

if no_wall
    pts = remove_wall(pts);
end

if triple
    %% LEFT SCAN
    theta = open_angle;
    rot_mat = [cos(theta), 0, sin(theta), 0;
        0, 1, 0, 0;
        -sin(theta), 0, cos(theta), 0
        0, 0, 0, 1];
    vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, -pi/2 + middle_angle + theta], vrep.simx_opmode_oneshot);
    vrchk(vrep, res);
    
    
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % Then retrieve the last point cloud the depth sensor took.
    % If you were to try to capture multiple images in a row, try other values than
    % vrep.simx_opmode_oneshot_wait.
    pts_left = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
    
    pts_left = pts_left(:, pts_left(4, :) < max_dist);
    
    if no_wall
        pts_left = remove_wall(pts_left);
    end
    
    pts_left = rot_mat * pts_left;
    
    %% RIGHT SCAN
    theta = -open_angle;
    rot_mat = [cos(theta), 0, sin(theta), 0;
        0, 1, 0, 0;
        -sin(theta), 0, cos(theta), 0
        0, 0, 0, 1];
    vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, -pi/2 + middle_angle + theta], vrep.simx_opmode_oneshot);
    vrchk(vrep, res);
    
    res = vrep.simxSetIntegerSignal(id, 'handle_xyz_sensor', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % Then retrieve the last point cloud the depth sensor took.
    % If you were to try to capture multiple images in a row, try other values than
    % vrep.simx_opmode_oneshot_wait.
    pts_right = youbot_xyz_sensor(vrep, h, vrep.simx_opmode_oneshot_wait);
    
    pts_right = pts_right(:, pts_right(4, :) < max_dist);
    
    if no_wall
        pts_right = remove_wall(pts_right);
    end
    
    pts_right = rot_mat * pts_right;
    
    %% CONCAT
    
    pts = cat(2, pts, pts_left);
    pts = cat(2, pts, pts_right);
end
end

% Remove wall points from 3d point cloud coming from xyz sensor
function [pts] = remove_wall(pts)

% Get points above a treshold and keep unique value horizontal
% dimensions (1 and 3 -> x,z)
ptsWall = pts([1,3], pts(2, :) >= 0.085);
ptsWall = unique(ptsWall', 'rows')';

% for each x,z tuple, remove points close enough (within tolerance)
tol = 0.01;
cond_tot = false(size(pts(1,:)));
for i=1:length(ptsWall(1,:))
    cond = bitand(ptsWall(:,i) > (pts([1,3], :) - tol), ptsWall(:,i) < (pts([1,3], :) + tol));
    cond = bitand(cond(1,:), cond(2,:));
    cond_tot = bitor(cond, cond_tot);
end

pts = pts(:,not(cond_tot));
end

% Find circle from 3 points
function [R,xcyc] = fit_circle_through_3_points(ABC)
% FIT_CIRCLE_THROUGH_3_POINTS
% Mathematical background is provided in http://www.regentsprep.org/regents/math/geometry/gcg6/RCir.htm
%
% Input:
%
%   ABC is a [3 x 2n] array. Each two columns represent a set of three points which lie on
%       a circle. Example: [-1 2;2 5;1 1] represents the set of points (-1,2), (2,5) and (1,1) in Cartesian
%       (x,y) coordinates.
%
% Outputs:
%
%   R     is a [1 x n] array of circle radii corresponding to each set of three points.
%   xcyc  is an [2 x n] array of of the centers of the circles, where each column is [xc_i;yc_i] where i
%         corresponds to the {A,B,C} set of points in the block [3 x 2i-1:2i] of ABC
%
% Author: Danylo Malyuta.
% Version: v1.0 (June 2016)
% ----------------------------------------------------------------------------------------------------------
% Each set of points {A,B,C} lies on a circle. Question: what is the circles radius and center?
% A: point with coordinates (x1,y1)
% B: point with coordinates (x2,y2)
% C: point with coordinates (x3,y3)
% ============= Find the slopes of the chord A<-->B (mr) and of the chord B<-->C (mt)
%   mt = (y3-y2)/(x3-x2)
%   mr = (y2-y1)/(x2-x1)
% /// Begin by generalizing xi and yi to arrays of individual xi and yi for each {A,B,C} set of points provided in ABC array
x1 = ABC(1,1:2:end);
x2 = ABC(2,1:2:end);
x3 = ABC(3,1:2:end);
y1 = ABC(1,2:2:end);
y2 = ABC(2,2:2:end);
y3 = ABC(3,2:2:end);
% /// Now carry out operations as usual, using array operations
mr = (y2-y1)./(x2-x1);
mt = (y3-y2)./(x3-x2);
% A couple of failure modes exist:
%   (1) First chord is vertical       ==> mr==Inf
%   (2) Second chord is vertical      ==> mt==Inf
%   (3) Points are collinear          ==> mt==mr (NB: NaN==NaN here)
%   (4) Two or more points coincident ==> mr==NaN || mt==NaN
% Resolve these failure modes case-by-case.
idf1 = isinf(mr); % Where failure mode (1) occurs
idf2 = isinf(mt); % Where failure mode (2) occurs
idf34 = isequaln(mr,mt) | isnan(mr) | isnan(mt); % Where failure modes (3) and (4) occur
% ============= Compute xc, the circle center x-coordinate
xcyc = (mr.*mt.*(y3-y1)+mr.*(x2+x3)-mt.*(x1+x2))./(2*(mr-mt));
xcyc(idf1) = (mt(idf1).*(y3(idf1)-y1(idf1))+(x2(idf1)+x3(idf1)))/2; % Failure mode (1) ==> use limit case of mr==Inf
xcyc(idf2) = ((x1(idf2)+x2(idf2))-mr(idf2).*(y3(idf2)-y1(idf2)))/2; % Failure mode (2) ==> use limit case of mt==Inf
xcyc(idf34) = NaN; % Failure mode (3) or (4) ==> cannot determine center point, return NaN
% ============= Compute yc, the circle center y-coordinate
xcyc(2,:) = -1./mr.*(xcyc-(x1+x2)/2)+(y1+y2)/2;
idmr0 = mr==0;
xcyc(2,idmr0) = -1./mt(idmr0).*(xcyc(idmr0)-(x2(idmr0)+x3(idmr0))/2)+(y2(idmr0)+y3(idmr0))/2;
xcyc(2,idf34) = NaN; % Failure mode (3) or (4) ==> cannot determine center point, return NaN
% ============= Compute the circle radius
R = sqrt((xcyc(1,:)-x1).^2+(xcyc(2,:)-y1).^2);
R(idf34) = Inf; % Failure mode (3) or (4) ==> assume circle radius infinite for this case
end

% Find the table center from 3d point cloud
function [r, center] = find_table_center(pts_cloud, sensor_angle)

% Get table points (ordered in x axis)
tablepts = pts_cloud([1,3], bitand(pts_cloud(2,:) < -0.053, pts_cloud(2,:) > -0.1));
tablepts = uniquetol(tablepts', 0.001, 'ByRows', true);
tablepts = sortrows(tablepts)';

% isolate closest table to sensor
sensor_dist = sqrt((tablepts(1,:)).^2 + (tablepts(2,:)).^2);

[~, idx] = min(sensor_dist);
closest = tablepts(:, idx);
closest_dist = sqrt((tablepts(1,:)-closest(1)).^2 + (tablepts(2,:)-closest(2)).^2);
tablepts = tablepts(:, closest_dist < 0.8);

ABC = tablepts(:, [1, floor(length(tablepts(1,:))/2), length(tablepts(1,:))])';
[r, center] = fit_circle_through_3_points(ABC);

% rotate center table by sensor angle
rot_mat = [cos(sensor_angle), sin(sensor_angle);
    -sin(sensor_angle), cos(sensor_angle)];
center = rot_mat * center;

% translate center table by youbot center (in current reference)
youbot_center = [0 ; -0.25];

center = center - youbot_center;

end

% Get X Y estimation points of a given circle
function [xunit, yunit] = create_circle(center ,r)
% based on circle() function of Matlab
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
end

% Drive in a circle at the left of the youbot
function [forwBackVel, rightVel, rotateRightVel] = drive_circle(r_circle, forward_speed, remaining_angle, tol)
% Check that we have reach our objective
if abs(remaining_angle) < tol
    forwBackVel = 0;
    rightVel = 0;
    rotateRightVel = 0;
else
    % Compute velocity
    forwBackVel = -forward_speed * abs(sin(remaining_angle)) * sign(remaining_angle);
    rightVel = 0;
    rotateRightVel = -forwBackVel*0.7781/((r_circle));
end
end

% Find the table and give the tilt of youbot with the tangent of the table,
% and the actual distance of youbot to the center of the table
function [dist_center, tilt, r] = errAnchor(pts_cloud, sensor_angle)

%% get table points
tablepts = pts_cloud([1,3], bitand(pts_cloud(2,:) < -0.053, pts_cloud(2,:) > -0.1));
tablepts = uniquetol(tablepts', 0.001, 'ByRows', true);
tablepts = sortrows(tablepts)';

ABC = tablepts(:, [1, floor(length(tablepts(1,:))/2), length(tablepts(1,:))])';
[r, center_t] = fit_circle_through_3_points(ABC);

%% get Youbot Components Pos
youbot_sensor = [0 ; 0];

rot_mat_2d = [cos(sensor_angle), -sin(sensor_angle);
    sin(sensor_angle), cos(sensor_angle)];

youbot_center = rot_mat_2d * [0 ; -0.25];

% vect_yCenter_tCenter
u = center_t - youbot_center;
% vect_yCenter_ySensor
v = youbot_sensor - youbot_center;


%% Compute tilt and distance
dist_center = norm(u);
dist_comp = 0.25;

% tilt positive in the direction of the table
tilt = pi/2 - acos(dot(u,v)/(dist_center * dist_comp));

end

% Find the rotation around the table needed to be more or less in front of
% the closest object
function [table_empty, angle, dist_to_obj] = next_object(pts, sensor_angle, r_traj, plotData)

table_empty = false;
angle = NaN;
dist_to_obj = NaN;

%% youbot_center
rot_mat_2d = [cos(-sensor_angle), sin(-sensor_angle);
    -sin(-sensor_angle), cos(-sensor_angle)];
youbot_center = rot_mat_2d * [0; -0.25];

%% get table
tablepts = pts([1,3], bitand(pts(2,:) < -0.053, pts(2,:) > -0.1));

if isempty(tablepts)
    return;
end

% projection (pts ordered on x)
tablepts = uniquetol(tablepts', 0.001, 'ByRows', true);
tablepts = sortrows(tablepts)';

% get center
ABC = tablepts(:, [1, floor(length(tablepts(1,:))/2), length(tablepts(1,:))])';
[r_t, center_t] = fit_circle_through_3_points(ABC);

% theoritical trajectory
[traj_x, traj_y] = create_circle(center_t, r_traj);

% plot table and traj
if plotData
    figure;
    axis equal;
    hold on;
    circle(center_t .* [-1 ; 1], r_t, 'k');
    plot(-traj_x, traj_y, 'm');
end

%% get objects
objectpts = pts([1,3], pts(2,:) > -0.03);
objectpts = uniquetol(objectpts', 0.001, 'ByRows', true)';

if isempty(objectpts)
    table_empty = true;
    return;
end

%% isolate closest shape (to youbot center)

% closest pt
bot_dist = sqrt((objectpts(1,:)-youbot_center(1)).^2 + (objectpts(2,:)-youbot_center(2)).^2);
[~, idx] = min(bot_dist);
closest = objectpts(:, idx);

% pts close enough to closest pt
closest_dist = sqrt((objectpts(1,:)-closest(1)).^2 + (objectpts(2,:)-closest(2)).^2);
keep_pts = objectpts(:, closest_dist < 0.08);

% grav center of selected pts
grav_center = mean(keep_pts, 2);

% plot object and sensor/center
if plotData
    plot(-objectpts(1,:), objectpts(2,:), 'Marker', 'o', 'LineStyle', 'none');
    plot(-keep_pts(1,:), keep_pts(2,:), 'Marker', 'o', 'Color', [0.4660 0.6740 0.1880], 'LineStyle', 'none');
    plot(0,0, 'Marker', 'd', 'Color', [0.6350 0.0780 0.1840]);
    plot(-grav_center(1), grav_center(2), 'Marker', 'o', 'Color', 'r');
    plot(-youbot_center(1), youbot_center(2), 'Marker', 'p', 'Color', [0.4940 0.1840 0.5560]);
end


%% find angle
% unit vector table -> grav_center
v1 = grav_center - center_t;
v1 = v1/norm(v1);

% intersection between radius to grav center and theoritical traj
[hit_pt_x, hit_pt_y] = polyxpoly([center_t(1), center_t(1) + 5*v1(1)], ...
    [center_t(2), center_t(2) + 5*v1(2)], ...
    traj_x, traj_y);

% plot perpendiculars and hit points
if plotData
    plot(-hit_pt_x, hit_pt_y, 'Marker', 'o', 'Color', 'm', 'LineStyle', 'none')
    plot(-[center_t(1), center_t(1) + 1*v1(1)], ...
        [center_t(2), center_t(2) + 1*v1(2)], 'r');
end

%% get angle
% get youbot change angle [0, pi]
% vector center table -> objective
v2 = [hit_pt_x ; hit_pt_y] - center_t;
v2 = v2/norm(v2);

% vector center table -> actual pos of youbot
v3 = (youbot_center - center_t);
v3 = v3/norm(v3);

angle = acos(dot(v2, v3));

% get direction forward/backward
% -> place objective in youbot ref
hit_bot_ref = [hit_pt_x ; hit_pt_y] - youbot_center;
hit_bot_ref = hit_bot_ref' * rot_mat_2d;

% objective is backward
if hit_bot_ref(2) < 0
    angle = -angle;
end

% Distance to object
dist_to_obj = norm([hit_pt_x ; hit_pt_y] - grav_center);

if plotData
    hold off;
end
end

% Find the rotation of youbot needed to have the center as close as
% possible to the objectand aligned with a side if shape is a square.
% In case of circle, give immediate angle and distance to grab if shape is
% close enough
function [r_traj, obj_angle, minDist, grab_angle, readyForGrab] = get_objects(pts, pts_large, sensor_angle, r_traj, plotData)

obj_angle = NaN;
minDist = NaN;
grab_angle = NaN;
readyForGrab = false;

%% youbot components
rot_mat_2d = [cos(-sensor_angle), sin(-sensor_angle);
    -sin(-sensor_angle), cos(-sensor_angle)];
% distance between components determined in the simulator
youbot_center = rot_mat_2d * [0; -0.25];
youbot_arm = rot_mat_2d * [0; -0.4162];
% youbot is 40cm width and 60cm long
youbot_edges = rot_mat_2d * [0.2, 0.05 ; -0.2, 0.05 ; -0.2, -0.55 ; 0.2, -0.55 ; 0.2, 0.05]';

%% object
% assumption on the shapes
shape_width = 0.05;

% projection of objects layer
objectpts = pts([1,3], pts(2,:) > -0.03);

if isempty(objectpts)
    return;
end

objectpts = uniquetol(objectpts', 0.001, 'ByRows', true)';


%% plot object and youbot
if plotData
    figure;
    plot(-objectpts(1,:), objectpts(2,:), 'Marker', 'o', 'LineStyle', 'none');
    axis equal;
    hold on;
    plot(0,0, 'Marker', 'd', 'Color', [0.6350 0.0780 0.1840]);
    plot(-youbot_center(1), youbot_center(2), 'Marker', 'p', 'Color', [0.4940 0.1840 0.5560]);
    plot(-youbot_arm(1), youbot_arm(2), 'Marker', '^', 'Color', [0.4940 0.1840 0.5560]);
    line(-youbot_edges(1,:), youbot_edges(2,:), 'Color', 'k');
end
%% get table
tablepts = pts_large([1,3], bitand(pts_large(2,:) < -0.053, pts_large(2,:) > -0.1));

if isempty(tablepts)
    return;
end

% projection (pts ordered on x)
tablepts = uniquetol(tablepts', 0.001, 'ByRows', true);
tablepts = sortrows(tablepts)';

% get center
ABC = tablepts(:, [1, floor(length(tablepts(1,:))/2), length(tablepts(1,:))])';
[r_t, xcyc_t] = fit_circle_through_3_points(ABC);

% theoritical trajectory
r_traj = norm(xcyc_t - youbot_center);
[traj_x, traj_y] = create_circle(xcyc_t, r_traj);

% plot trajectory
if plotData
    circle(xcyc_t .* [-1 ; 1], r_t, 'k');
    plot(-traj_x, traj_y, 'm');
end

% objectives
best_hit = [[],[]];
center_obj = [0,0];

%% isolate closest shape (to youbot center)

% closest pt
bot_dist = sqrt((objectpts(1,:)-youbot_center(1)).^2 + (objectpts(2,:)-youbot_center(2)).^2);
[~, idx] = min(bot_dist);
closest = objectpts(:, idx);

% pts close enough to closest pt
closest_dist = sqrt((objectpts(1,:)-closest(1)).^2 + (objectpts(2,:)-closest(2)).^2);
keep_pts = objectpts(:, closest_dist < 0.08);
n_obj_pt = length(keep_pts(1,:));
keep_pts = sortrows(keep_pts')';

% plot points from closest shape
if plotData
    plot(-keep_pts(1,:), keep_pts(2,:), 'Marker', 'o', 'Color', [0.4660 0.6740 0.1880], 'LineStyle', 'none');
end

%% identify shape
% need more points
if length(keep_pts(1,:)) < 5
    fprintf('Too few points\n');
    return;
    % find shape
else
    % check linear correlation of all pts
    correlation = corrcoef(keep_pts');
    correlation = abs(correlation(1,2));
    
    % only one line got (square case)
    if correlation > 0.99
        dist = norm(keep_pts(:,1)- keep_pts(:,n_obj_pt));
        fprintf('Shape is a square : Got A single line (length %f)\n', dist);
        
        % center of the line
        center_l1 = [keep_pts(1,1) + keep_pts(1,n_obj_pt); ...
            keep_pts(2,1) + keep_pts(2,n_obj_pt)]/2;
        % line unit vect
        vect1 = [keep_pts(1,1) - keep_pts(1,n_obj_pt), ...
            keep_pts(2,1) - keep_pts(2,n_obj_pt)];
        vect1 = vect1/norm(vect1);
        % perpendicular vector (point to opposite to the sensor)
        vect2 = [vect1(2) ; - vect1(1)];
        if vect2(2) < 0
            vect2 = -vect2;
        end
        
        % estimate center
        center_obj = (center_l1 + (vect2 * shape_width/2))';
        
        % plot guessed center
        if plotData
            plot(-center_obj(1), center_obj(2), 'Marker', '*', 'Color', 'r', 'LineStyle', 'none')
        end
        
        %intersections of the perpendiculars too square with the trajectory
        [hit_pt_x, hit_pt_y] = polyxpoly([center_obj(1) - 2*vect1(1), center_obj(1) + 2*vect1(1), NaN, ...
            center_obj(1) - 2*vect2(1), center_obj(1) + 2*vect2(1)], ...
            [center_obj(2) - 2*vect1(2), center_obj(2) + 2*vect1(2), NaN, ...
            center_obj(2) - 2*vect2(2), center_obj(2) + 2*vect2(2)], ...
            traj_x, traj_y);
        
        % plot perpendiculars and hit points
        if plotData
            plot(-hit_pt_x, hit_pt_y, 'Marker', '*', 'Color', 'm', 'LineStyle', 'none')
            plot(-[center_obj(1) - 1*vect1(1), center_obj(1) + 1*vect1(1)], ...
                [center_obj(2) - 1*vect1(2), center_obj(2) + 1*vect1(2)], 'r', ...
                -[center_obj(1) - 1*vect2(1), center_obj(1) + 1*vect2(1)], ...
                [center_obj(2) - 1*vect2(2), center_obj(2) + 1*vect2(2)], 'r');
        end
        
        % best hit
        dist = sqrt((hit_pt_x-center_obj(1)).^2 + (hit_pt_y-center_obj(2)).^2);
        [minDist, idx_best] = min(dist);
        best_hit = [hit_pt_x(idx_best), hit_pt_y(idx_best)];
        
        
        
        % square (two lines) or circle
    else
        square_matched = false;
        
        %try to match a square
        %cut the series of ordered pts (on x) to get a match of a line
        %on both sides
        for i = 2:n_obj_pt-2
            % linear correlation of left/right "lines"
            corr1 = corrcoef(keep_pts(:,1:i)');
            corr1 = abs(corr1(1,2));
            corr2 = corrcoef(keep_pts(:,i+1:n_obj_pt)');
            corr2 = abs(corr2(1,2));
            
            % Match found
            if (corr1 + corr2) > 2*0.99
                fprintf('Shape is a square : Lines matched\n');
                square_matched = true;
                % save last point of the first line
                line1_last = i;
                
                % replot first line in another color
                if plotData
                    plot(-keep_pts(1,1:line1_last), keep_pts(2,1:line1_last), 'Marker', 'o', 'Color', 'g', 'LineStyle', 'none');
                end
                %% get hit points
                % unit vector, approximate corner to further pt in line 1
                vect1 = [keep_pts(1,1) - keep_pts(1,line1_last), ...
                    keep_pts(2,1) - keep_pts(2,line1_last)];
                vect1 = vect1/norm(vect1);
                % unit vector, approximate corner to further pt in line 2
                vect2 = [keep_pts(1,n_obj_pt) - keep_pts(1,line1_last+1), ...
                    keep_pts(2,n_obj_pt) - keep_pts(2,line1_last+1)];
                vect2 = vect2/norm(vect2);
                
                angle_corner = acos(dot(vect1,vect2));
                
                % angle differ for more than 10 degrees
                % (more a diagnostic feature)
                if abs(angle_corner - pi/2) > 10 * pi/180
                    fprintf('Angle between lines is %f° : May be too wide\n', angle_corner*180/pi);
                end
                
                % get more precise corner (intersection of the lines)
                [corner_x, corner_y] = polyxpoly([keep_pts(1,1) - 2*vect1(1), keep_pts(1,1) + 2*vect1(1)], ...
                    [keep_pts(2,1) - 2*vect1(2), keep_pts(2,1) + 2*vect1(2)], ...
                    [keep_pts(1,n_obj_pt) - 2*vect2(1), keep_pts(1,n_obj_pt) + 2*vect2(1)], ...
                    [keep_pts(2,n_obj_pt) - 2*vect2(2), keep_pts(2,n_obj_pt) + 2*vect2(2)]);
                
                % estimate center
                center_obj = [corner_x, corner_y] + (vect1+vect2) * shape_width/2;
                
                % plot the corner and the center of the square
                if plotData
                    plot(-corner_x, corner_y, 'Marker', 'o', 'Color', 'r', 'LineStyle', 'none')
                    plot(-center_obj(1), center_obj(2), 'Marker', '*', 'Color', 'r', 'LineStyle', 'none')
                end
                
                % intersection(s) between perpendicular lines of the square
                % and the youbot circular trajectory
                [hit_pt_x, hit_pt_y] = polyxpoly([center_obj(1) - 2*vect1(1), center_obj(1) + 2*vect1(1), NaN, ...
                    center_obj(1) - 2*vect2(1), center_obj(1) + 2*vect2(1)], ...
                    [center_obj(2) - 2*vect1(2), center_obj(2) + 2*vect1(2), NaN, ...
                    center_obj(2) - 2*vect2(2), center_obj(2) + 2*vect2(2)], ...
                    traj_x, traj_y);
                
                % plot perpendiculars and hit points
                if plotData
                    plot(-hit_pt_x, hit_pt_y, 'Marker', '*', 'Color', 'm', 'LineStyle', 'none')
                    plot(-[center_obj(1) - 1*vect1(1), center_obj(1) + 1*vect1(1)], ...
                        [center_obj(2) - 1*vect1(2), center_obj(2) + 1*vect1(2)], 'r', ...
                        -[center_obj(1) - 1*vect2(1), center_obj(1) + 1*vect2(1)], ...
                        [center_obj(2) - 1*vect2(2), center_obj(2) + 1*vect2(2)], 'r');
                end
                
                % best hit (closest to shape)
                dist = sqrt((hit_pt_x-center_obj(1)).^2 + (hit_pt_y-center_obj(2)).^2);
                [minDist, idx_best] = min(dist);
                best_hit = [hit_pt_x(idx_best), hit_pt_y(idx_best)];
                
                
                break;
            end
        end
        
        % then object should be a circle
        if ~square_matched
            fprintf('Shape is a circle\n');
            
            % get center of the object
            ABC = keep_pts(:, [1, floor(length(keep_pts(1,:))/2), length(keep_pts(1,:))])';
            [r, center_obj] = fit_circle_through_3_points(ABC);
            center_obj = center_obj';
            
            % plot circle
            if plotData
                circle(center_obj .* [-1  1], r, 'r');
            end
            
            % setup direct vector from arm to object
            vect_arm_to_obj = center_obj - youbot_arm';
            dist_arm_to_obj = norm(vect_arm_to_obj);
            vect_arm_to_obj = vect_arm_to_obj/dist_arm_to_obj;
            
            % check for ready to grab
            if dist_arm_to_obj < 0.48
                minDist = dist_arm_to_obj;
                vect_arm_to_sensor = -youbot_arm'/norm(youbot_arm);
                grab_angle = acos(dot(vect_arm_to_obj, vect_arm_to_sensor));
                readyForGrab = true;
                if plotData
                    plot(-[center_obj(1), youbot_arm(1)], [center_obj(2), youbot_arm(2)], 'Color', 'r');
                    hold off;
                end
                return;
            end
            % not ready for grab :
            
            % unit vect center table to center object
            vect1 = [center_obj(1) - xcyc_t(1); center_obj(2) - xcyc_t(2)];
            norm_v1 = norm(vect1);
            vect1 = vect1/norm_v1;
            
            % best hit
            best_hit = (xcyc_t + r_traj * vect1)';
            minDist = r_traj - norm_v1;
            
            %plot radius crossing object center
            if plotData
                plot(-[xcyc_t(1), best_hit(1)], [xcyc_t(2), best_hit(2)], 'r');
            end
        end
    end
    
end

% plot goal
if plotData
    plot(-best_hit(1), best_hit(2), 'Marker', 'o', 'Color', 'm', 'LineStyle', 'none', 'MarkerSize', 12);
end

%% get grab angle (ASSUME TRIGONOMETRIC REVOLUTION AROUND TABLE)
% (REM x axis inverted)

% vector objective -> object
v1 = center_obj - best_hit;
v1 = v1/norm(v1);

% vector center table -> objective
v2 = best_hit - xcyc_t';
v2 = v2/norm(v2);

% vector tangent to circle (pointing in same direction as youbot)
v2_p = [v2(2), -v2(1)];

% determine angle for the arm
grab_angle = acos(dot(v1,v2_p));

% plot tangent facing as youbot
if plotData
    plot(-[best_hit(1), best_hit(1) + 0.5*v2_p(1)], [best_hit(2), best_hit(2) + 0.5*v2_p(2)], 'm');
end

%% get youbot change angle +-[0, pi]

% vector center table -> center youbot
v3 = (youbot_center - xcyc_t)';
v3 = v3/norm(v3);

obj_angle = acos(dot(v2, v3));

% get direction forward/backward
% -> place objective in youbot ref
hit_bot_ref = best_hit - youbot_center';
hit_bot_ref = hit_bot_ref * rot_mat_2d;

% objective is backward (objective behind youbot ref in its referential)
if hit_bot_ref(2) < 0
    obj_angle = -obj_angle;
end

if plotData
    hold off;
end

end

% Find the angle and distance for the grab, in case of square shape,
% give a motion (without rotation), to align with the side of the square
function [front_adjust, grab_dist, grab_angle] = adjust_object(pts, sensor_angle, est_dist_obj_to_sensor, plotData)

%default value
front_adjust = 0;

%% youbot components
rot_mat_2d = [cos(-sensor_angle), sin(-sensor_angle);
    -sin(-sensor_angle), cos(-sensor_angle)];
% distance between components determined in the simulator
youbot_center = rot_mat_2d * [0; -0.25];
youbot_arm = rot_mat_2d * [0; -0.4162];
% youbot is 40cm width and 60cm long
youbot_edges = rot_mat_2d * [0.2, 0.05 ; -0.2, 0.05 ; -0.2, -0.55 ; 0.2, -0.55 ; 0.2, 0.05]';

%unit vector from arm to cam
v_arm_cam = - youbot_arm/norm(youbot_arm);

%% object
% assumption on the shapes
shape_width = 0.05;

% projection of objects layer
objectpts = pts([1,3], pts(2,:) > -0.03);

if isempty(objectpts)
    return;
end

objectpts = uniquetol(objectpts', 0.001, 'ByRows', true)';

%% plot object and youbot
if plotData
    figure;
    plot(-objectpts(1,:), objectpts(2,:), 'Marker', 'o', 'LineStyle', 'none');
    axis equal;
    hold on;
    plot(0,0, 'Marker', 'd', 'Color', [0.6350 0.0780 0.1840]);
    plot(-youbot_center(1), youbot_center(2), 'Marker', 'p', 'Color', [0.4940 0.1840 0.5560]);
    plot(-youbot_arm(1), youbot_arm(2), 'Marker', '^', 'Color', [0.4940 0.1840 0.5560]);
    line(-youbot_edges(1,:), youbot_edges(2,:), 'Color', 'k');
    plot(-[youbot_center(1) - v_arm_cam(1), youbot_center(1) + v_arm_cam(1)], ...
        [youbot_center(2) - v_arm_cam(2), youbot_center(2) + v_arm_cam(2)], 'Color', 'm');
end

%% isolate closest shape to estimate
estimated_pos = [0, est_dist_obj_to_sensor];

% get point closest to estimation
dist_to_est = sqrt((objectpts(1,:)-estimated_pos(1)).^2 + (objectpts(2,:)-estimated_pos(2)).^2);
[~, idx] = min(dist_to_est);
closest = objectpts(:, idx);

% keep pts close enough to closest (ordered)
closest_dist = sqrt((objectpts(1,:)-closest(1)).^2 + (objectpts(2,:)-closest(2)).^2);
keep_pts = objectpts(:, closest_dist < 0.08);
n_obj_pt = length(keep_pts(1,:));
keep_pts = sortrows(keep_pts')';

%% identify shape
% need more points
if length(keep_pts(1,:)) < 5
    fprintf('Too few points\n');
    return;
    % find shape
else
    % check linear correlation of all pts
    correlation = corrcoef(keep_pts');
    correlation = abs(correlation(1,2));
    
    % only one line got (square case)
    if correlation > 0.99
        dist = norm(keep_pts(:,1)- keep_pts(:,n_obj_pt));
        fprintf('Shape is a square : Got A single line (length %f)\n', dist);
        
        % center of the line
        center_l1 = [keep_pts(1,1) + keep_pts(1,n_obj_pt); ...
            keep_pts(2,1) + keep_pts(2,n_obj_pt)]/2;
        % line unit vect
        vect1 = [keep_pts(1,1) - keep_pts(1,n_obj_pt), ...
            keep_pts(2,1) - keep_pts(2,n_obj_pt)];
        vect1 = vect1/norm(vect1);
        % perpendicular vector (point to opposite to the sensor)
        vect2 = [vect1(2) ; - vect1(1)];
        if vect2(2) < 0
            vect2 = -vect2;
        end
        
        % estimate center
        center_obj = (center_l1 + (vect2 * shape_width/2))';
        
        % plot guessed center
        if plotData
            plot(-center_obj(1), center_obj(2), 'Marker', '*', 'Color', 'r', 'LineStyle', 'none')
        end
        
        % intersection(s) between perpendicular lines of the square and
        % youbot front direction (should only give one)
        [hit_pt_x, hit_pt_y] = polyxpoly([center_obj(1) - 2*vect1(1), center_obj(1) + 2*vect1(1), NaN, ...
            center_obj(1) - 2*vect2(1), center_obj(1) + 2*vect2(1)], ...
            [center_obj(2) - 2*vect1(2), center_obj(2) + 2*vect1(2), NaN, ...
            center_obj(2) - 2*vect2(2), center_obj(2) + 2*vect2(2)], ...
            [youbot_arm(1)-2*v_arm_cam(1), youbot_arm(1)+2*v_arm_cam(1)], ...
            [youbot_arm(2)-2*v_arm_cam(2), youbot_arm(2)+2*v_arm_cam(2)]);
        
        % plot perpendiculars and hit points
        if plotData
            plot(-hit_pt_x, hit_pt_y, 'Marker', '*', 'Color', 'm', 'LineStyle', 'none')
            plot(-[center_obj(1) - 1*vect1(1), center_obj(1) + 1*vect1(1)], ...
                [center_obj(2) - 1*vect1(2), center_obj(2) + 1*vect1(2)], 'r', ...
                -[center_obj(1) - 1*vect2(1), center_obj(1) + 1*vect2(1)], ...
                [center_obj(2) - 1*vect2(2), center_obj(2) + 1*vect2(2)], 'r');
        end
        
        % best hit (only useful if more than one intersection)
        dist = sqrt((hit_pt_x-youbot_arm(1)).^2 + (hit_pt_y-youbot_arm(2)).^2);
        [front_adjust, idx_best] = min(dist);
        best_hit = [hit_pt_x(idx_best), hit_pt_y(idx_best)];
        
        % check if adjustement is forward or backward
        if dot(v_arm_cam, best_hit' - youbot_arm) < 0
            front_adjust = - front_adjust;
        end
        
        
        % square (two lines) or circle
    else
        square_matched = false;
        
        %try to match a square
        %cut the series of ordered pts (on x) to get a match of a line
        %on both sides
        for i = 2:n_obj_pt-2
            % linear correlation of left/right "lines"
            corr1 = corrcoef(keep_pts(:,1:i)');
            corr1 = abs(corr1(1,2));
            corr2 = corrcoef(keep_pts(:,i+1:n_obj_pt)');
            corr2 = abs(corr2(1,2));
            
            % Match found
            if (corr1 + corr2) > 2*0.99
                fprintf('Shape is a square : Lines matched\n');
                square_matched = true;
                % save last point of the first line
                line1_last = i;
                
                % replot first line in another color
                if plotData
                    plot(-keep_pts(1,1:line1_last), keep_pts(2,1:line1_last), 'Marker', 'o', 'Color', 'g', 'LineStyle', 'none');
                end
                
                %% get hit points
                % unit vector, approximate corner to further pt in line 1
                vect1 = [keep_pts(1,1) - keep_pts(1,line1_last), ...
                    keep_pts(2,1) - keep_pts(2,line1_last)];
                vect1 = vect1/norm(vect1);
                % unit vector, approximate corner to further pt in line 2
                vect2 = [keep_pts(1,n_obj_pt) - keep_pts(1,line1_last+1), ...
                    keep_pts(2,n_obj_pt) - keep_pts(2,line1_last+1)];
                vect2 = vect2/norm(vect2);
                
                angle_corner = acos(dot(vect1,vect2));
                
                % angle differ for more than 10 degrees
                % (more a diagnostic feature)
                if abs(angle_corner - pi/2) > 10 * pi/180
                    fprintf('Angle between lines is %f° : May be too wide\n', angle_corner*180/pi);
                end
                
                % get more precise corner (intersection of the lines)
                [corner_x, corner_y] = polyxpoly([keep_pts(1,1) - 2*vect1(1), keep_pts(1,1) + 2*vect1(1)], ...
                    [keep_pts(2,1) - 2*vect1(2), keep_pts(2,1) + 2*vect1(2)], ...
                    [keep_pts(1,n_obj_pt) - 2*vect2(1), keep_pts(1,n_obj_pt) + 2*vect2(1)], ...
                    [keep_pts(2,n_obj_pt) - 2*vect2(2), keep_pts(2,n_obj_pt) + 2*vect2(2)]);
                
                % estimate center
                center_obj = [corner_x, corner_y] + (vect1+vect2) * shape_width/2;
                
                % plot the corner and the center of the square
                if plotData
                    plot(-corner_x, corner_y, 'Marker', 'o', 'Color', 'r', 'LineStyle', 'none')
                    plot(-center_obj(1), center_obj(2), 'Marker', '*', 'Color', 'r', 'LineStyle', 'none')
                end
                
                % intersection(s) between perpendicular lines of the square and
                % youbot front direction (should only give one)
                [hit_pt_x, hit_pt_y] = polyxpoly([center_obj(1) - 2*vect1(1), center_obj(1) + 2*vect1(1), NaN, ...
                    center_obj(1) - 2*vect2(1), center_obj(1) + 2*vect2(1)], ...
                    [center_obj(2) - 2*vect1(2), center_obj(2) + 2*vect1(2), NaN, ...
                    center_obj(2) - 2*vect2(2), center_obj(2) + 2*vect2(2)], ...
                    [youbot_arm(1)-2*v_arm_cam(1), youbot_arm(1)+2*v_arm_cam(1)], ...
                    [youbot_arm(2)-2*v_arm_cam(2), youbot_arm(2)+2*v_arm_cam(2)]);
                
                % plot perpendiculars and hit points
                if plotData
                    plot(-hit_pt_x, hit_pt_y, 'Marker', '*', 'Color', 'm', 'LineStyle', 'none')
                    plot(-[center_obj(1) - 1*vect1(1), center_obj(1) + 1*vect1(1)], ...
                        [center_obj(2) - 1*vect1(2), center_obj(2) + 1*vect1(2)], 'r', ...
                        -[center_obj(1) - 1*vect2(1), center_obj(1) + 1*vect2(1)], ...
                        [center_obj(2) - 1*vect2(2), center_obj(2) + 1*vect2(2)], 'r');
                end
                
                % best hit (only useful if more than one intersection)
                dist = sqrt((hit_pt_x-youbot_arm(1)).^2 + (hit_pt_y-youbot_arm(2)).^2);
                [front_adjust, idx_best] = min(dist);
                best_hit = [hit_pt_x(idx_best), hit_pt_y(idx_best)];
                
                % check if adjustement is forward or backward
                if dot(v_arm_cam, best_hit' - youbot_arm) < 0
                    front_adjust = - front_adjust;
                end
                
                
                break;
            end
        end
        
        % then object should be a circle
        if ~square_matched
            fprintf('Shape is a circle\n');
            
            % get center of the object
            ABC = keep_pts(:, [1, floor(length(keep_pts(1,:))/2), length(keep_pts(1,:))])';
            [r, center_obj] = fit_circle_through_3_points(ABC);
            center_obj = center_obj';
            
            % plot circle
            if plotData
                circle(center_obj .* [-1  1], r, 'r');
            end
            
            
            % only need to adjust the angle for a circle
            best_hit = youbot_arm';
            
            %plot line to arm
            if plotData
                plot(-[center_obj(1), best_hit(1)], [center_obj(2), best_hit(2)], 'r');
            end
        end
    end
    
end

%% get new angle and dist

% vector : objectives -> object
v1 = center_obj - best_hit;
grab_dist = norm(v1);
v1 = v1/grab_dist;

grab_angle = acos(dot(v1,v_arm_cam));


end

% Loop on this function by feeding the return fsm as new entry each time to
% grab an object with the specified angle and distance
function [fsm] = grasping(vrep, id, h, fsm, obj_angle, grip_distance)

startingJoints = [0 * pi / 180, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

% define treshold where the arm should be reverse to reach the object
grip_treshold = .43;

if grip_distance < grip_treshold
    pickupJoints = [obj_angle, 45.5 * pi / 180, 122.5 * pi / 180, -78 * pi / 180, 0];
    intermediateJoints2 = [-180 *pi /180, 25 * pi / 180, 25 * pi / 180, -90 * pi / 180, 0];
else
    pickupJoints = [(obj_angle-pi), -45.5 * pi / 180, -122.5 * pi / 180,  78 * pi / 180, 0];
    intermediateJoints2 = [180 *pi /180, 0 * pi / 180, -20 * pi / 180, -90 * pi / 180, 0];
end

intermediateJoints1 = [obj_angle/2, 30 * pi / 180, 30 * pi / 180, -40 * pi / 180, 0];

blockingJoints = [180 *pi /180, -45 * pi / 180, -40 * pi / 180, -95 * pi / 180, 0];

%% initiate grasping
% make sure the arm start at resting pos
if strcmp(fsm, 'starting')
    
    reached = set_joints(vrep, id, h, startingJoints);
    
    % Change state when pos reached
    if reached
        if grip_distance < grip_treshold
            fsm = 'intermediate1';
        else
            fsm = 'pickup';
        end
    end
    
    %% short grasping require intermediate pos here
elseif strcmp(fsm, 'intermediate1')
    
    reached = set_joints(vrep, id, h, intermediateJoints1);
    
    % Change state when pos reached
    if reached
        fsm = 'pickup';
    end
    %% set to pickup pos
elseif strcmp(fsm, 'pickup')
    reached = set_joints(vrep, id, h, pickupJoints);
    
    % Change state when pos reached, switch to IK
    if reached
        res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
        vrchk(vrep, res, true);
        fsm = 'extend';
    end
    %% extend arm
elseif strcmp(fsm, 'extend')
    % get tip pos
    [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % increase extension slightly
    tpos(1) = tpos(1) + (0.01 * sin(obj_angle));
    tpos(2) = tpos(2) + (-0.01 * cos(obj_angle));
    
    res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
    vrchk(vrep, res, true);
    
    if norm([tpos(1), tpos(2)]) > grip_distance
        fsm = 'grab';
    end
    
    %% close grip
elseif strcmp(fsm, 'grab')
    
    % close grip
    res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    % Make MATLAB wait for the gripper to be closed. This value was determined by experiments.
    pause(2);
    
    % Disable IK; this is used at the next state to move the joints manually.
    res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    
    fsm = 'intermediate2';
    
    %% intermediate pos before blocking (depend on short/long grasp)
elseif strcmp(fsm, 'intermediate2')
    
    reached = set_joints(vrep, id, h, intermediateJoints2);
    
    % Change state when pos reached
    if reached
        fsm = 'blocking';
    end
    
    %% set the arm to blocking pos
elseif strcmp(fsm, 'blocking')
    reached = set_joints(vrep, id, h, blockingJoints);
    
    % Change state when pos reached
    if reached
        fsm = 'finished';
    end
    
    %% arm motion ended, nothing to do
elseif strcmp(fsm, 'finished')
    % nothing to do
else
    fprintf('unknown state\n');
end

end

% Loop on this function with 5 joints angle value to reach,
% cond returned is true when angles are reached
function [cond] = set_joints(vrep, id, h, joints_val)
cond = false;

% set to asked pos
for i = 1:5
    res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), joints_val(i), vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end

% check current angle
currAngle = zeros(1,5);
for i = 1:5
    [res, currAngle(i)] = vrep.simxGetJointPosition(id, h.armJoints(i), vrep.simx_opmode_streaming);
    vrchk(vrep, res, true);
end

% Change state when pos reached
if norm(joints_val - currAngle) < 0.002
    cond = true;
end

end

% Loop on this function by feeding the return fsm as new entry each time to
% place an object on a table (assuming youbot is "well placed"
% (see main function)
function [fsm] = replace_object(vrep, id, h, fsm)

startingJoints = [0 * pi / 180, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];

intermediateJoints1 = [90 *pi /180, -35 * pi / 180, -30 * pi / 180, 0 * pi / 180, 0];
intermediateJoints2 = [90 *pi /180, 40 * pi / 180, 50 * pi / 180, -40 * pi / 180, 0];
intermediateJoints3 = [90 *pi /180, 50 * pi / 180, 80 * pi / 180, -40 * pi / 180, 0];
intermediateJoints4 = [45 *pi /180, 40 * pi / 180, 50 * pi / 180, -40 * pi / 180, 0];

blockingJoints = [180 *pi /180, -45 * pi / 180, -40 * pi / 180, -95 * pi / 180, 0];

%% initiate grasping
if strcmp(fsm, 'starting')
    
    reached = set_joints(vrep, id, h, blockingJoints);
    
    % Change state when pos reached
    if reached
        fsm = 'intermediate1';
    end
    
    %% intermediate pos
elseif strcmp(fsm, 'intermediate1')
    
    reached = set_joints(vrep, id, h, intermediateJoints1);
    
    % Change state when pos reached
    if reached
        fsm = 'intermediate2';
    end
    %% intermediate pos
elseif strcmp(fsm, 'intermediate2')
    reached = set_joints(vrep, id, h, intermediateJoints2);
    
    % Change state when pos reached
    if reached
        fsm = 'intermediate3';
    end
elseif strcmp(fsm, 'intermediate3')
    reached = set_joints(vrep, id, h, intermediateJoints3);
    
    % Change state when pos reached, switch to IK
    if reached
        fsm = 'open';
    end
    %% down arm
elseif strcmp(fsm, 'open')
    % Make sure that youbot is stabilized after the arm motion
    pause(2);
    
    % Open the gripper when the arm is placed
    res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
    pause(2);
    fsm = 'intermediate4';
    
    %% close grip
elseif strcmp(fsm, 'intermediate4')
    
    reached = set_joints(vrep, id, h, intermediateJoints2);
    
    if reached
        fsm = 'intermediate5';
    end
    
    %% intermediate pos
elseif strcmp(fsm, 'intermediate5')
    
    reached = set_joints(vrep, id, h, intermediateJoints4);
    
    if reached
        fsm = 'resting';
    end
    
    %% intermediate pos
elseif strcmp(fsm, 'resting')
    
    reached = set_joints(vrep, id, h, startingJoints);
    
    if reached
        fsm = 'finished';
    end
    %% arm motion ended
elseif strcmp(fsm, 'finished')
    % nothing to do
else
    fprintf('unknown state\n');
end

end
