function [ trajSmooth ] = smooth_yaw_rate( inputMat, trajSmooth, nConstantMT865, timeStepS, timeStepNo )
% This function smooths out the driver input of the steering angle
%   so that a smooth nominal trajectory is generated

% Unpack trajSmooth
tI = trajSmooth.tI;
tF = trajSmooth.tF;
yawRateFuncCoeff = trajSmooth.yawRateFuncCoeff;
yawRateCmdTraj = trajSmooth.yawRateCmdTraj;

% Unpack Input Matrix
throttle = inputMat(:,1);
gear = inputMat(:,2);
steerAngleDeg = inputMat(:,3);
timeArray = inputMat(:,5);

% Define Parameters
trackCG2trackCG = nConstantMT865.trackCG2trackCG;
maxDiffTrackSpeed = nConstantMT865.maxDiffTrackSpeed;
maxNominalYawRate = maxDiffTrackSpeed/trackCG2trackCG;

% Determine if there has been a command change
yawRateCmd_k = (steerAngleDeg(timeStepNo-1)/170)*maxNominalYawRate;
if timeStepNo == 2
    yawRateCmdChange = 0;
else
    yawRateCmd_km1 = (steerAngleDeg(timeStepNo-2)/170)*maxNominalYawRate;
    yawRateCmdChange = ( abs(yawRateCmd_k - yawRateCmd_km1) > (1E-12) );
end
noCmdChange = ~yawRateCmdChange;

% Determine if current time is in the function window
timeCurrent = timeStepNo*timeStepS - timeStepS;
timeIsInWindow = ( (tI <= timeCurrent) && (timeCurrent <= tF) );
timeIsNotInWindow = ~timeIsInWindow;

% if timeCurrent > 7.9
%     fake = 3;
% end

% Compute the transition time between past yaw rate command and the new one
if yawRateCmdChange
    yawRateDotMax = 0.005;
    yawRateCmdDiffAbs = abs(yawRateCmd_k - yawRateCmd_km1);
    transitionTime = yawRateCmdDiffAbs/yawRateDotMax;
    tI = timeStepNo*timeStepS - timeStepS;
    tF_Exact = tI + transitionTime;
    tF = round(tF_Exact*(1/timeStepS))/(1/timeStepS);
    yawRateFuncCoeff = polynomial_fit(tI,tF,yawRateCmd_km1,yawRateCmd_k, 0, 0);
    yawRateCmdSmooth = poly_eval(yawRateFuncCoeff,timeCurrent);
% Evaluate current function    
elseif noCmdChange && timeIsInWindow
    yawRateCmdSmooth = poly_eval(yawRateFuncCoeff,timeCurrent);
elseif noCmdChange && timeIsNotInWindow
    yawRateCmdSmooth = yawRateCmd_k;
end

% Pack trajSmooth
trajSmooth.tI = tI;
trajSmooth.tF = tF; 
trajSmooth.yawRateFuncCoeff = yawRateFuncCoeff;
trajSmooth.yawRateCmdTraj(timeStepNo-1,1) = yawRateCmdSmooth;

end

function funcCoeff = polynomial_fit(tI,tF,z_tI,z_tF, zDot_tI, zDot_tF)
 
A = [1  tI  tI^2  tI^3;
     1  tF  tF^2  tF^3;
     0  1   2*tI  3*tI^2; 
     0  1   2*tF  3*tF^2];
 
b = [z_tI z_tF zDot_tI zDot_tF].';

% A = [1  tI  tI^2  tI^3;
%      1  tF  tF^2  tF^3;
%      0  1   2*tI  3*tI^2; 
%      0  1   2*tF  3*tF^2;
%      0  0   2     6*tI;
%      0  0   2     6*tF];
%  
% b = [z_tI z_tF zDot_tI zDot_tF  0  0].';

funcCoeff = A\b;

end

function cmdTraj = poly_eval(coeff,t)

c0 = coeff(1,1);
c1 = coeff(2,1);
c2 = coeff(3,1);
c3 = coeff(4,1);

cmdTraj = c3*t.^3 + c2*t.^2 + c1*t + c0;
end