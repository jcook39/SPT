function [inputVecOut, controller, waypoint, nWayPointFollowEval] = speed_heading_control(tractorLead, tractorFollow, inputVecIn, controller, waypoint, nWayPointFollowEval, timeStepS, timeStepNo)

% Unpack needed tractor states
XFollow = tractorFollow.state(1);
YFollow = tractorFollow.state(2);
thetaFollow = tractorFollow.state(3);
vxFollow = tractorFollow.state(6);
vyFollow = tractorFollow.state(7);
speedFollow = sqrt(vxFollow^2 + vyFollow^2);
gpsHeadingFollow = atan2(vyFollow,vxFollow) + thetaFollow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[waypoint, nWayPointFollowEval] = wayPoint_Control(tractorLead, tractorFollow, waypoint, nWayPointFollowEval, controller, timeStepS, timeStepNo);

% X-Y Coordinates for Follower Heading Reference
wayPointFlagX = nWayPointFollowEval(waypoint.wayPointNo).wayPointFollowEval(1,1);
wayPointFlagY = nWayPointFollowEval(waypoint.wayPointNo).wayPointFollowEval(1,2);
wayPointFlag = [wayPointFlagX wayPointFlagY];

% Gather X-Y Coordinate in time domain for follower 
waypoint.nWayPointFollowEvalMatTime(timeStepNo,:) = wayPointFlag;

% Gather all waypoints placed for time domain
waypoint.nWayPointAllTime(timeStepNo,:) = waypoint.nWayPointStateFollower(1:2,waypoint.wayPointNo).';

% Control loop for heading
[inputVecOut, controller] = heading_control(tractorFollow,inputVecIn,controller, wayPointFlag.', timeStepS,timeStepNo);


%--------------------- Speed Control --------------------------------------
Kpv = controller.Kpv;
Kiv = controller.Kiv;
kx = controller.kx;
speedIntegratedError = controller.speedIntegratedError(timeStepNo,1);
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
errorx = -1*controller.refDlong - dist2Leadx;
%fprintf(' errorx = %f \n', errorx)

speedRef = -kx*errorx + speedLead;
speedError = speedRef - speedFollow;
speedIntegratedError = speedIntegratedError + speedError*timeStepS;

[throttleCommandPI] = PI_Velocity_Control_Diff_EQ(controller, speedError, timeStepNo);
throttleCommand = throttleFeedForward + throttleCommandPI;

% throttleCommand = throttleFeedForward + Kpv*speedError + Kiv*speedIntegratedError;
% %throttleCommand = throttleFeedForward + Kp_speed*speedError;
% if throttleCommand > 1
%     fprintf('Throttle Controller is Saturating %f \n',throttleCommand)
%     throttleCommand = 1;
% elseif throttleCommand < 0
%     fprintf('Throttle Controller is Low Saturating %f \n', throttleCommand)
%     throttleCommand = 0;
% end

% Package Output as a row vector
inputVecOut(1,1) = throttleCommand;

% Package controller error and integrated Error signals
controller.speedError(timeStepNo,1) = speedError;
controller.speedIntegratedError(timeStepNo,1) = speedIntegratedError;
controller.speedRef(timeStepNo,1) = speedRef;
controller.errorx(timeStepNo,1) = errorx;
controller.throttleCommand(timeStepNo,1) = throttleCommand;
controller.throttleCommandPI(timeStepNo,1) = throttleCommandPI;

end


function [throttleCommandPI] = PI_Velocity_Control_Diff_EQ(controller, speedError, timeStepNo)

% --------------------- Unpack Needed Parameters --------------------------
% Controller Parameters
vSysConD = controller.hSysConD;
vSysConDTFnum = vSysConD.num{1};
vSysConDTFden = vSysConD.den{1};

b0 = vSysConDTFnum(1,1);
b1 = vSysConDTFnum(1,2);

a0 = vSysConDTFden(1,1);
a1 = vSysConDTFden(1,2);

% Look up past error values and inputs
ek = speedError;
if (timeStepNo-1) <= 0
    ekm1 = 0;
    ukm1 = 0;
else
    ekm1 = controller.speedError(timeStepNo-1,1);
    ukm1 = controller.throttleCommandPI(timeStepNo-1,1);
end
      

% ------------------- Compute Control Input -------------------------------
throttleCommandPI = (1/a0)*(-ukm1*a1 + b0*ek + b1*ekm1);

end