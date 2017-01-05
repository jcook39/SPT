function [inputVecOut, controller, waypoint, nWayPointFollowEval] = speed_heading_control(tractorLead, tractorFollow, inputVecIn, controller, waypoint, nWayPointFollowEval, timeStepS, timeStepNo)

% Unpack needed tractor states
XFollow = tractorFollow.state(1);
YFollow = tractorFollow.state(2);
thetaFollow = tractorFollow.state(3);
vxFollow = tractorFollow.state(6);
vyFollow = tractorFollow.state(7);
speedFollow = sqrt(vxFollow^2 + vyFollow^2);
gpsHeadingFollow = atan2(vyFollow,vxFollow) + thetaFollow;

% Unpack controller paramters
Kp_heading = controller.Kp_heading;
Ki_heading = controller.Ki_heading;
integratedError = controller.integratedError;
headingIntegratedError = integratedError(1,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[waypoint, nWayPointFollowEval] = wayPoint_Control(tractorLead, tractorFollow, waypoint, nWayPointFollowEval, controller, timeStepS, timeStepNo);

% X-Y Coordinates for Follower Heading Reference
wayPointFlagX = nWayPointFollowEval(waypoint.wayPointNo).wayPointFollowEval(1,1);
wayPointFlagY = nWayPointFollowEval(waypoint.wayPointNo).wayPointFollowEval(1,2);

% Gather X-Y Coordinate in time domain for follower 
waypoint.nWayPointFollowEvalMatTime(timeStepNo,:) = [wayPointFlagX wayPointFlagY];

% Gather all waypoints placed for time domain
waypoint.nWayPointAllTime(timeStepNo,:) = waypoint.nWayPointStateFollower(1:2,waypoint.wayPointNo).';

dist2PtGlobalXwayPoint = wayPointFlagX - XFollow;
dist2PtGlobalYwayPoint = wayPointFlagY - YFollow;

%----------------------- Steering Control ---------------------------------
headingRef = atan2(dist2PtGlobalYwayPoint, dist2PtGlobalXwayPoint);
headingError = headingRef - gpsHeadingFollow;
headingIntegratedError = headingIntegratedError + headingError*timeStepS;

% Compute Control Inputs
steerPumpCmd = Kp_heading*headingError + Ki_heading*headingIntegratedError;
if steerPumpCmd > 1
    steerPumpCmd = 1;
elseif steerPumpCmd < -1
    steerPumpCmd = -1;
end

%--------------------- Speed Control --------------------------------------
Kp_speed = controller.Kp_speed;
Ki_speed = controller.Ki_speed;
kx = controller.kx;
speedIntegratedError = integratedError(2,1);
XLead = tractorLead.state(1);
YLead = tractorLead.state(2);
vxLead = tractorLead.state(6);
vyLead = tractorLead.state(7);
speedLead = sqrt(vxLead^2 + vyLead^2);
throttleFeedForward = inputVecIn(1,1);

dist2LeadGlobalX = XLead - XFollow;
dist2LeadGlobalY = YLead - YFollow;
dist2LeadBF1 = rotation_matrix_z(gpsHeadingFollow)*[dist2LeadGlobalX dist2LeadGlobalY 0].';
dist2Leadx = dist2LeadBF1(1,1);
errorx = -1*controller.Refp1 - dist2Leadx;
%fprintf(' errorx = %f \n', errorx)

speedRef = -kx*errorx + speedLead;
speedError = speedRef - speedFollow;
speedIntegratedError = speedIntegratedError + speedError*timeStepS;

throttleCommand = throttleFeedForward + Kp_speed*speedError + Ki_speed*speedIntegratedError;
%throttleCommand = throttleFeedForward + Kp_speed*speedError;
if throttleCommand > 1
    fprintf('Throttle Controller is Saturating %f \n',throttleCommand)
    throttleCommand = 1;
elseif throttleCommand < 0
    fprintf('Throttle Controller is Low Saturating %f \n', throttleCommand)
    throttleCommand = 0;
end

% Package Output as a row vector
%inputVecOut = [inputVecIn(1), inputVecIn(2), steerPumpCmd, inputVecIn(4), inputVecIn(5)];
inputVecOut = [throttleCommand, inputVecIn(2), steerPumpCmd, inputVecIn(4), inputVecIn(5)];

% Package controller error and integrated Error signals
controller.error = [headingError speedError 0].';
controller.integratedError = [headingIntegratedError speedIntegratedError 0].';
controller.errorMatrix(:,timeStepNo-1) = controller.error;
controller.integratedErrorMatrix(:,timeStepNo-1) = controller.integratedError;
controller.steerPumpCmd(timeStepNo-1,1) = steerPumpCmd;
controller.headingRef(timeStepNo-1,:) = headingRef ;
controller.speedRef(timeStepNo-1,:) = speedRef;

controller.errorx(timeStepNo-1) = errorx;
controller.throttleCommand(timeStepNo-1,1) = throttleCommand;

end