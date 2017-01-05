function [waypoint, nWayPointFollowEval] = initialize_waypoint_paramters(wayPointTimeStepS, timeArray, simulationTime)

%------------- First data structure 'waypoint' ----------------------------
waypoint.timeWayPoint = [0:wayPointTimeStepS:simulationTime].'; % time array for waypoint recording
waypoint.nWayPoint = size(waypoint.timeWayPoint,1); % number of waypoints
waypoint.nWayPointStateLeader = zeros(5,waypoint.nWayPoint); % [X Y heading speed thetaDot].'
waypoint.nWayPointStateFollower = zeros(5,waypoint.nWayPoint);

waypoint.wayPointTimer = wayPointTimeStepS; % This is initialized to capture the starting condition
waypoint.wayPointTimeStepS = wayPointTimeStepS;
waypoint.wayPointNo = 0;

waypoint.nWayPointFollowEvalMatTime = zeros(numel(timeArray),2);
waypoint.nWayPointAllTime = zeros(numel(timeArray),2);

%------------- Second Data Structure 'nWayPointFollowEval'-----------------

nWayPointFollowEval.nWayPointFollowEvalMat = 0;
nWayPointFollowEval.wayPointFollowEval = zeros(1,2);
nWayPointFollowEval(1:waypoint.nWayPoint) = nWayPointFollowEval;


end


% waypoint.tractorStructOldWayPoint = 0;
% waypoint.tractorStructNewWayPoint = 0;
% 
% waypoint.splineCoeffX = zeros(4,waypoint.nWayPoint-1);
% waypoint.splineCoeffY = zeros(4,waypoint.nWayPoint-1);
% 
% waypoint.splineCoeffXFiltered = zeros(4,waypoint.nWayPoint-1);
% waypoint.splineCoeffYFiltered = zeros(4,waypoint.nWayPoint-1);
