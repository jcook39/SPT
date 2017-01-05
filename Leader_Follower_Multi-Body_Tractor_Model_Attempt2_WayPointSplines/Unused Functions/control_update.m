function [input, controller] = control_update(tractorOne, tractorTwo, input, controller, timeStepS)
% This function is the controller for a follower vehicle



%%%%%%%%%%%%%%%%%%%%%% Unpack Controller Parameters %%%%%%%%%%%%%%%%%%%%%%%
Kpx = controller.Kpx;
Kix = controller.Kix;
Kpy = controller.Kpy;
Kiy = controller.Kiy;
integratedError = controller.integratedError;

Refx1 = controller.Refx1;
Refy1 = controller.Refy1;
Refz1 = 0;
ref = [Refx1 Refy1 Refz1].';


%%%%%%%%%%%%%%%%%%%% Unpack Leader Information %%%%%%%%%%%%%%%%%%%%%%%%%%%%
XL = tractorOne.state(1);
YL = tractorOne.state(2);
thetaL = tractorOne.state(3);

%%%%%%%%%%%%%%%%%%%% Unpack Follower Information %%%%%%%%%%%%%%%%%%%%%%%%%%
XF = tractorTwo.state(1);
YF = tractorTwo.state(2);
thetaF = tractorTwo.state(3);

%%%%%%%%%%%%%%%%%%%%%% Compute Error Signal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Distance from follower to leader in global/inertial frame
distX = -( XL - XF);
distY = -( YL - YF);
distZ = 0;
distGlobalFrame = [distX distY distZ].';

% Distance from leader to follower in leader's body fixed frame
Rz = rotation_matrix_z( thetaL);
distBFLeaderFrame = Rz*distGlobalFrame;

% Error signal
error = ref - distBFLeaderFrame;

% Rearrange for heading angle correction
lookAheadDistancem = 10;
error(2) = atan2(error(2),lookAheadDistancem);

integratedError = integratedError + error*timeStepS;

%fprintf('distX = %f , distY = %f, distBFLeaderFrame(1) = %f , distBFLeaderFrame(2) = %f \n ', distX, distY, distBFLeaderFrame(1), distBFLeaderFrame(2) )

%%%%%%%%%%%%%%%%%%%%%% Compute Control inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

input(1,1) = input(1,1) + Kpx*error(1) + Kix*integratedError(1);
input(1,3) = input(1,3) + Kpy*error(2) + Kiy*integratedError(2);

% Throttle Input Satuation
if input(1,1) > 1
    input(1,1) = 1;
elseif input(1,1) < 0
    input(1,1) = 0;
end

% Steering angle saturation
if input(1,3) > 170
    input(1,3) = 170;
elseif input(1,3) < -170
    input(1,3) = -170;
end

%fprintf('error = %f , integratedError = %f , throttle = %f, steerAngle = %f  \n', error, integratedError, input(1,1), input(1,3));

controller.error = error;
controller.integratedError = integratedError;

end