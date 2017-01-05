function waypoint = waypoint_trajectory_generation(tractorLead,waypoint,controller, trajSmooth,timeStepS,timeStepNo)

% Unpack Paramters from "waypoint" structure
timeWayPoint = waypoint.timeWayPoint; 
nWayPoint = waypoint.nWayPoint;
nWayPointState = waypoint.nWayPointState;
nWayPointLocationFiltered = waypoint.nWayPointLocationFiltered;
wayPointTimer = waypoint.wayPointTimer;
wayPointTimeStepS = waypoint.wayPointTimeStepS;
wayPointNo = waypoint.wayPointNo;

tractorStructOldWayPoint = waypoint.tractorStructOldWayPoint;
tractorStructNewWayPoint = waypoint.tractorStructNewWayPoint;

splineCoeffX = waypoint.splineCoeffX;
splineCoeffY = waypoint.splineCoeffY;

splineCoeffXFiltered = waypoint.splineCoeffXFiltered;
splineCoeffYFiltered = waypoint.splineCoeffYFiltered;


% Gathers WayPoints at specified Intervals, generate splines between points for trajectory
if abs(wayPointTimer - wayPointTimeStepS) < (1E-8)
    wayPointNo = wayPointNo + 1;
    nWayPointState(:,wayPointNo) = way_point_log(tractorLead);
    %nWayPointLocation(:,wayPointNo) = way_point_log_cartesian(tractorLead, controller);
    %nWayPointLocationFiltered(:,wayPointNo) = filter_waypoints(nWayPointLocation,wayPointNo,windowN);
    %nWayPointLocationFiltered(:,wayPointNo) = way_point_log_curvilinear(tractorLead, controller);
    %nWayPointLocationFiltered(:,wayPointNo) = way_point_log_curvilinear_nominal_trajectory(tractorLead, controller, trajSmooth, timeStepNo);
    wayPointTimer = 0;
    %tractorStructOldWayPoint = tractorStructNewWayPoint;
    %tractorStructNewWayPoint = tractorLead;
    %if (wayPointNo >= 2)
        %[splineCoeffX(:,wayPointNo-1), splineCoeffY(:,wayPointNo-1)] = trajectory_generation(nWayPointLocation(:,wayPointNo-1:wayPointNo),tractorStructNewWayPoint,tractorStructOldWayPoint,timeWayPoint,wayPointNo);
        %[splineCoeffXFiltered(:,wayPointNo-1), splineCoeffYFiltered(:,wayPointNo-1)] = trajectory_generation(nWayPointLocationFiltered(:,wayPointNo-1:wayPointNo),tractorStructNewWayPoint,tractorStructOldWayPoint,timeWayPoint,wayPointNo);
    %end
end

% Increment waypoint timer
wayPointTimer = wayPointTimer + timeStepS;

% Pack Up waypoint paramters
waypoint.nWayPointLocation = nWayPoint ;
waypoint.nWayPointLocationFiltered = nWayPointLocationFiltered;
waypoint.wayPointTimer = wayPointTimer;
waypoint.wayPointNo = wayPointNo;

waypoint.tractorStructOldWayPoint = tractorStructOldWayPoint;
waypoint.tractorStructNewWayPoint = tractorStructNewWayPoint;

waypoint.splineCoeffX = splineCoeffX;
waypoint.splineCoeffY = splineCoeffY;

waypoint.splineCoeffXFiltered = splineCoeffXFiltered;
waypoint.splineCoeffYFiltered = splineCoeffYFiltered;
end

function nWayPointLocationFiltered = filter_waypoints(nWayPointLocation,wayPointNo,windowN)

% Waypoint Indices for filtering
windowIndexBegin = wayPointNo - windowN;
if windowIndexBegin < 1
    windowIndexBegin = 1;
end
windowIndexEnd = wayPointNo;
window = windowIndexBegin:windowIndexEnd;
windowSize = numel(window);

nLocationX = nWayPointLocation(1,window);
nLocationY = nWayPointLocation(2,window);

B = ones(1,windowSize)/windowSize;
locationXFiltered = sum(B.*nLocationX);
locationYFiltered = sum(B.*nLocationY);

nWayPointLocationFiltered = [locationXFiltered locationYFiltered 0].';


end