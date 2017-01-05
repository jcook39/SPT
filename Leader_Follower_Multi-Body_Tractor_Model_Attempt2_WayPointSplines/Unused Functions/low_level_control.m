function [inputVecOut, controller] = low_level_control(tractor,inputVecIn,trajSmooth,controller,timeStepS,timeStepNo)

% Unpack needed tractor states
yawRate = tractor.state(8);

% Unpack controller paramters
Kp_yawRate = controller.Kp_yawRate;
Ki_yawRate = controller.Ki_yawRate;
integratedError = controller.integratedError;
yawRateIntegratedError = integratedError(1);

% Unpack trajSmooth
yawRateCmdTraj = trajSmooth.yawRateCmdTraj;
yawRateCmdRef = yawRateCmdTraj(timeStepNo-1);

% Error and integrated error signals for PI Controller
yawRateError = yawRateCmdRef - yawRate;
yawRateIntegratedError = yawRateIntegratedError + yawRateError*timeStepS;

% Compute Control Inputs
steerPumpCmd = Kp_yawRate*yawRateError + Ki_yawRate*yawRateIntegratedError;
if steerPumpCmd > 1
    steerPumpCmd = 1;
elseif steerPumpCmd < -1
    steerPumpCmd = -1;
end

% Package Output as a row vector
inputVecOut = [inputVecIn(1), inputVecIn(2), steerPumpCmd, inputVecIn(4), inputVecIn(5)];

% Package controller error and integrated Error signals
controller.error = [yawRateError 0 0].';
controller.integratedError = [yawRateIntegratedError 0 0].';
controller.errorMatrix(:,timeStepNo-1) = controller.error;
controller.integratedErrorMatrix(:,timeStepNo-1) = controller.integratedError;
controller.steerPumpCmd(timeStepNo-1,1) = steerPumpCmd;

%fprintf('steerPumpCmd = %f \n',steerPumpCmd)

end