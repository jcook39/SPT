function [ gearNo ] = gear_shift_controller( structTractionController, omegaHat, nConstantMT865 )
% 

% ------------------ Unpack Needed Controller Parameters ------------------
gearShiftControlEngineRPMTargetRPM = structTractionController.gearShiftControlEngineRPMTargetRPM;
gearShiftControlEngineRPMTargetRadPS = gearShiftControlEngineRPMTargetRPM*((2*pi)/60); 

% ------------------- Unpack Needed Tractor Paramters ---------------------
nGearNo = nConstantMT865.nGearNo;
nGR = nConstantMT865.nGearRatio(1:16);
FD = nConstantMT865.finalDriveRatio;
engineTorqueDataNM = nConstantMT865.engineTorqueDataNM;
engineSpeedDataRadPS = nConstantMT865.engineSpeedDataRadPS;

% --------------- Compute Optimal Gear Ratio ------------------------------
% Note: Optimal here means the gear ratio that minimizes the squared error
% between the engine speed associated with "omegaRef" and
% "gearShiftControlEngineRPMTargetRPM"

lumpedGR = nGR*FD;
engineSpeedRadPSnGR = omegaHat*lumpedGR; % Engine Speed in each gear ratio for the track reference speed
diffHold = (engineSpeedRadPSnGR - gearShiftControlEngineRPMTargetRadPS).^2; % difference between reference and actual for each gear
indexOptimalGR = min(diffHold) == diffHold;
gearNo = nGearNo(indexOptimalGR);

% NEEDS TO BE LOGIC SO THAT NO MORE THAN A CHANGE OF 1 GEAR TAKES PLACE!

end

