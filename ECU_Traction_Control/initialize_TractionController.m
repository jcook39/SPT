function structTractionController = initialize_TractionController(KpAll, KiAll, KdAll, nConstantMT865, nTimeStep)

% --------------- Unpack Tractor Needed Parameters ------------------------
nGR = nConstantMT865.nGearRatio;

% ----------------- Calculate Gear ratio specific gains -------------------
KpGR = KpAll./nGR;
KiGR = KiAll./nGR;
KdGR = KdAll./nGR;

%KpGR(:) = KpAll;
%KiGR(:) = KiAll;

% ------------------- Controller Initilization ----------------------------
% PIDF
% KpGR = 0.4;
% KiGR = 0.2;
% KdGR = 0.1;
Tf = 0.000001;

ControllerC = pid(KpGR(1:16).', KiGR(1:16).', KdGR(1:16).', Tf);
controllerSampleTime = 0.05; % 20 Hz;
sysControllerC = tf(ControllerC);
sysControllerD = c2d(sysControllerC,controllerSampleTime,'tustin');

% ----------------- Package controller structure --------------------------
structTractionController.KpGR = KpGR;
structTractionController.KiGR = KiGR;

structTractionController.errorOmegaIntegrated = zeros(nTimeStep,1);
structTractionController.errorOmega = zeros(nTimeStep,1);
structTractionController.throttleFeedForward = zeros(nTimeStep,1);
structTractionController.torqueFeedForwardNM = zeros(nTimeStep,1);
structTractionController.peakSlip = zeros(nTimeStep,1);
structTractionController.iref = zeros(nTimeStep,1);
structTractionController.omegaRef = zeros(nTimeStep,1); 
structTractionController.throttleControllerPIDFF = zeros(nTimeStep,1);
structTractionController.throttleControllerPID = zeros(nTimeStep,1);

structTractionController.tractionControlIsOn = zeros(nTimeStep,1);

structTractionController.gearShiftControlUpdateRateHz = 1;
structTractionController.gearShiftControlIsOn = zeros(nTimeStep,1);
structTractionController.gearShiftControlCountInt = zeros(nTimeStep,1);
structTractionController.gearShiftControlEngineRPMTargetRPM = 1700;
structTractionController.gearNo = zeros(nTimeStep,1);

structTractionController.gearShiftFlag = zeros(nTimeStep,1);
structTractionController.sysControllerD = sysControllerD;

end