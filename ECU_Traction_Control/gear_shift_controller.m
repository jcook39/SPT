function [ gearNoNew ] = gear_shift_controller( structTractionController, gearNoOld, nConstantMT865, timeStepNo, timeStepS )
% 

% ------------------ Unpack Needed Controller Parameters ------------------
gearControlString = structTractionController.gearControlString;
engSpeedRadPS = structTractionController.engSpeedRadPS;

% 'maxPower' gear controller parameters
gearShiftControlEngineRPMTargetRPM = structTractionController.gearShiftControlEngineRPMTargetRPM;
gearShiftControlEngineRPMTargetRadPS = gearShiftControlEngineRPMTargetRPM*((2*pi)/60); 

% 'engineRPMRange' gear controller parameters
gearShiftControlHighEngineRPMBound = structTractionController.gearShiftControlHighEngineRPMBound;
gearShiftControlHighEngineRadPSBound = gearShiftControlHighEngineRPMBound*((2*pi)/60);
gearShiftControlLowEngineRPMBound = structTractionController.gearShiftControlLowEngineRPMBound;
gearShiftControlLowEngineRadPSBound = gearShiftControlLowEngineRPMBound*((2*pi)/60);

% ------------------- Unpack Needed Tractor Paramters ---------------------
nGearNo = nConstantMT865.nGearNo;
nGR = nConstantMT865.nGearRatio(1:16);
FD = nConstantMT865.finalDriveRatio;
engineTorqueDataNM = nConstantMT865.engineTorqueDataNM;
engineSpeedDataRadPS = nConstantMT865.engineSpeedDataRadPS;

% ------------------- Gather Last Second History of Engine Speeds ---------
timeLookBack = 0.5;
nTimeStepLookBack = timeLookBack/timeStepS;
timeStepNoLookBackEnd = timeStepNo;
timeStepNoLookBackBeg = timeStepNoLookBackEnd - nTimeStepLookBack;
if timeStepNoLookBackBeg < 1
    timeStepNoLookBackBeg = 1;
end
timeStepNoLookBack = timeStepNoLookBackBeg:timeStepNoLookBackEnd;
engSpeedRadPSLookBack = engSpeedRadPS(timeStepNoLookBack,1);

% ----------------------- Gear Shift Control ------------------------------

engSpeedRadPSBelowBound = sum((engSpeedRadPSLookBack < gearShiftControlLowEngineRadPSBound) >= 1);
engSpeedRadPSAboveBound = sum((engSpeedRadPSLookBack > gearShiftControlHighEngineRadPSBound) >= 1);
engSpeedRadPSInBound = ~engSpeedRadPSBelowBound && ~engSpeedRadPSAboveBound;
if engSpeedRadPSBelowBound
    gearNoNew = gearNoOld - 1;
elseif engSpeedRadPSAboveBound
    gearNoNew = gearNoOld + 1;
elseif engSpeedRadPSInBound
    gearNoNew = gearNoOld;    
end

if gearNoNew < 1
    gearNoNew = 1;
elseif gearNoNew > 16
    gearNoNew = 16;
end

end



