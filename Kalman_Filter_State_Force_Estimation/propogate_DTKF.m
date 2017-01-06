function [structDTKF] = propogate_DTKF(structDTKF ,MT865, nConstantMT865, input, timeStepNo, kalmanGainString)

% ------------------ Kalman Filter Annotation/Definitions -----------------
%
% Estimated State Vector:
% xHat = [vHat omegaHat F_T,NetHat F_T,NetHatDot tau_ResHat tau_ResHatDot 
%           DBHat DBHatDot].';
%
% True State Vector: 
% x = [v omega F_T,Net F_T,NetDot tau_Res tau_ResDot DB DBDot].';
%
% Measurement Vector:
% y = [vDot v omega R_S].'

% --------------------- Unpack DTKF Structure -----------------------------
F = structDTKF.F;
G = structDTKF.G;
Q = structDTKF.Q;
R = structDTKF.R;
H = structDTKF.H;

processNoiseMean = structDTKF.processNoiseMean;
measurementNoiseMean = structDTKF.measurementNoiseMean;

x = structDTKF.x;
xError = structDTKF.xError;
xHatMinus = structDTKF.xHatMinus;
xHatPlus = structDTKF.xHatPlus;
PPlus = structDTKF.PPlus;
PMinus = structDTKF.PMinus;
K = structDTKF.K;
Koffline = structDTKF.Koffline;

nState = structDTKF.nState;
nMeasure = structDTKF.nMeasure;

engineThrottleSignalNoiseVariance = structDTKF.engineThrottleSignalNoiseVariance;

% ------------------- Unpack Tractor Parameters ---------------------------
nGR = nConstantMT865.nGearRatio;
FD = nConstantMT865.finalDriveRatio;
mT = nConstantMT865.massTractorKG;
rollingRadiusM = nConstantMT865.rollingRadiusM;
trackDamp = nConstantMT865.trackDamp;

% -------------------- Unpack Tractor Structure ---------------------------
vx = MT865.state(4);
vy = MT865.state(5);
wl = MT865.state(7);
wr = MT865.state(8);
engineThrottle = MT865.state(9);
engineSpeedRadPS = MT865.state(10);
engineControlThrottle = MT865.state(12);

% -------------------- Declare Measurement Vector -------------------------
vxd = MT865.xdot(4);
v = sqrt(vx^2 + vy^2);
omega = (wl + wr)/2;
vDot = vxd;
DB = MT865.drawBarPullN;

% ------------------- Calculate F_TNet ------------------------------------
Fl = MT865.forces(1,1);
Fr = MT865.forces(2,1);
RL = MT865.forces(3,1);
RR = MT865.forces(4,1);
F_TNet = (Fl + Fr) - (RL + RR);
tauRes = (Fl + Fr)*rollingRadiusM;

% ------------------ Inject Measurement Noise -----------------------------
measurementNoise = mvnrnd(measurementNoiseMean.',R);
x(:,timeStepNo) = [v omega F_TNet 0 tauRes 0 DB 0].';
y = H*x(:,timeStepNo) + measurementNoise.';

% ------------------ Estimate Input Torque --------------------------------
engineThrottleSignalWithNoise = engineThrottle + sqrt(engineThrottleSignalNoiseVariance)*randn;
engTorqNM = engine_interp( engineThrottleSignalWithNoise, engineControlThrottle, engineSpeedRadPS, nConstantMT865 );
gearNo = input(2,1);
gearRatio = nGR(gearNo);
tauApplied = engTorqNM*gearRatio*FD;
u = tauApplied;

% ------------------ Propagration Equations -------------------------------
if strcmp(kalmanGainString, 'Konline')
    
    PMinus(:,:,timeStepNo) = F*PPlus(:,:,timeStepNo-1)*F.' + Q;
    K(:,:,timeStepNo) = PMinus(:,:,timeStepNo)*transpose(H)*inv(H*PMinus(:,:,timeStepNo)*transpose(H) + R);
    xHatMinus(:,timeStepNo) = F*xHatPlus(:,timeStepNo-1) + G*u;
    xHatPlus(:,timeStepNo) = xHatMinus(:,timeStepNo) + K(:,:,timeStepNo)*(y - H*xHatMinus(:,timeStepNo));
    PPlus(:,:,timeStepNo) = ( eye(nState,nState) - K(:,:,timeStepNo)*H)*PMinus(:,:,timeStepNo);
    
elseif strcmp(kalmanGainString, 'Koffline')
    
    %PMinus(:,:,timeStepNo) = F*PPlus(:,:,timeStepNo-1)*F.' + Q;
    %K(:,:,timeStepNo) = PMinus(:,:,timeStepNo)*transpose(H)*inv(H*PMinus(:,:,timeStepNo)*transpose(H) + R);
    xHatMinus(:,timeStepNo) = F*xHatPlus(:,timeStepNo-1) + G*u;
    xHatPlus(:,timeStepNo) = xHatMinus(:,timeStepNo) + Koffline*(y - H*xHatMinus(:,timeStepNo));
    %PPlus(:,:,timeStepNo) = ( eye(nState,nState) - K(:,:,timeStepNo)*H)*PMinus(:,:,timeStepNo);   
    
end

xError(:,timeStepNo) = x(:,timeStepNo) - xHatPlus(:,timeStepNo);

% ----------------- Velocity Low Pass Filter ------------------------------
% Unpack smoother parameters
smootherAd = structDTKF.smootherAd;
smootherBd = structDTKF.smootherBd;
smoothedvHatm1 = structDTKF.smoothedvHat(1,timeStepNo-1);

% Output velocity estimate from DTKF
vHat = xHatPlus(1,timeStepNo);
smoothedvHat = smootherAd*smoothedvHatm1 + smootherBd*vHat;

% ----------------- Calculate Estimates of Augmented States ---------------
omegaHat = xHatPlus(1,timeStepNo);
slipHat = calculate_slip(vHat, omegaHat, nConstantMT865);
slipHatSmooth = calculate_slip(smoothedvHat,omegaHat,nConstantMT865);


% ------------------ Package DTKF structure -------------------------------
% DTKF
structDTKF.x = x;
structDTKF.xError = xError;
structDTKF.xHatMinus = xHatMinus;
structDTKF.xHatPlus = xHatPlus;
structDTKF.PPlus = PPlus;
structDTKF.PMinus = PMinus;
structDTKF.K = K;
structDTKF.y(:,timeStepNo) = y;

% Velocity Filter/Smoother
structDTKF.smoothedvHat(1,timeStepNo) = smoothedvHat;

% Augmented State / slip estimates
structDTKF.slipHat(1,timeStepNo) = slipHat;
structDTKF.slipHatSmooth(1,timeStepno) = slipHatSmooth;

end