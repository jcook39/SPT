function [inputVecOut, controller] = heading_control(tractor,inputVecIn,controller, wayPointFlag, timeStepS,timeStepNo)

% Unpack needed tractor states
X = tractor.state(1);
Y = tractor.state(2);
theta = tractor.state(3);
vx = tractor.state(6);
vy = tractor.state(7);
speed = sqrt(vx^2 + vy^2);
gpsHeading = atan2(vy,vx) + theta;

% Unpack controller paramters
Kph = controller.Kph;
Kih = controller.Kih;
headingIntegratedError = controller.headingIntegratedError(timeStepNo);

% Error and integrated error signals for PI Controller
wayPointFlagX = wayPointFlag(1,1);
wayPointFlagY = wayPointFlag(2,1);
dist2PtGlobalX = wayPointFlagX - X;
dist2PtGlobalY = wayPointFlagY - Y;
dist2PtBodyFixedFrame1 = rotation_matrix_z(gpsHeading)*[dist2PtGlobalX dist2PtGlobalY 0].';

headingRef = atan2(dist2PtGlobalY, dist2PtGlobalX);
headingError = headingRef - gpsHeading;
headingIntegratedError = headingIntegratedError + headingError*timeStepS;

% Compute Control Inputs
[steerPumpCmdPI] = PI_Heading_Control_Diff_EQ(controller, headingError, timeStepNo);

% Package Output as a row vector
inputVecOut = [inputVecIn(1), inputVecIn(2), steerPumpCmdPI, inputVecIn(4), inputVecIn(5)];

% Package controller error and integrated Error signals
controller.headingRef(timeStepNo,1) = headingRef ;
controller.headingError(timeStepNo,1) = headingError;
controller.steerPumpCmdPI(timeStepNo,1) = steerPumpCmdPI;

end



