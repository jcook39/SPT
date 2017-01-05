function [ structDTKF ] = intialize_DTKF(processNoiseVariance, nConstantMT865, nTimeStep, timeStepS)
% This function intializes a linear discrete time kalman filter
%
% --------------------- Estimated State Vector ----------------------------
% xHat = [vHat omegaHat F_T,NetHat F_T,NetHatDot tau_ResHat tau_ResHatDot 
%           R_SHat R_SHatDot].';
%
% --------------------- True State Vector ----------------------------
% x = [v omega F_T,Net F_T,NetDot tau_Res tau_ResDot R_S R_SDot].';
%
% -------------------- Measurement Vector ---------------------------------
% y = [vDot v omega R_S].'
%
% ------- Covariance Matrices for Process and Measurement Noise -----------
filterTimeStepS = timeStepS;
measurementNoiseVariance = [0.1 0.003 2.8521E-4 2.6795E7].'; % v in Optimal State Estimation by Simon
measurementNoiseMean = zeros(size(measurementNoiseVariance,1),1);
processNoiseMean = zeros(size(processNoiseVariance,1),1);
Q = diag(processNoiseVariance);
R = diag(measurementNoiseVariance);

% ---------------------- Unpack Tractor Parameters ------------------------
JT = nConstantMT865.trackInertiaKGM2;
mT = nConstantMT865.massTractorKG;
trackDamp = nConstantMT865.trackDamp;

nState = 8;
nInput = 1;
nMeasure = 4;

% -------------------- Inialize Estimated System Model --------------------
A = [0  0  1/mT  0   0     0  -1/mT  0;
     0  0  0     0  -1/JT  0   0    0;
     0  0  0     1   0     0   0    0;
     0  0  0     0   0     0   0    0;
     0  0  0     0   0     1   0    0;
     0  0  0     0   0     0   0    0;
     0  0  0     0   0     0   0    1;
     0  0  0     0   0     0   0    0];
 
B = zeros(nState,nInput);
B(2,1) = 1/JT;

H = [0  0  1/mT  0  0  0  -1/mT  0;
     1  0  0     0  0  0   0     0;
     0  1  0     0  0  0   0     0;
     0  0  0     0  0  0   1     0];
     
[F, G] = c2d(A, B, filterTimeStepS);


% ------------------ Package DTKF Structure -------------------------------
structDTKF.F = F;
structDTKF.G = G;
structDTKF.filterTimeStepS = filterTimeStepS;
structDTKF.Q = Q;
structDTKF.processNoiseMean = processNoiseMean;
structDTKF.R = R;
structDTKF.measurementNoiseMean = measurementNoiseMean;
structDTKF.xHatMinus = zeros(nState,nTimeStep);
structDTKF.xHatPlus = zeros(nState,nTimeStep);
structDTKF.x = zeros(nState,nTimeStep);
structDTKF.xError = zeros(nState,nTimeStep);
structDTKF.PMinus = zeros(nState,nState,nTimeStep);
structDTKF.PPlus = zeros(nState,nState,nTimeStep);
structDTKF.K = zeros(nState,nMeasure,nTimeStep);
structDTKF.H = H;
structDTKF.y = zeros(nMeasure,nTimeStep);

structDTKF.nState = nState;
structDTKF.nMeasure = nMeasure;

structDTKF.engineThrottleSignalNoiseVariance = 1.57e-6;

load('Koffline.mat')
structDTKF.Koffline = Koffline;



end

