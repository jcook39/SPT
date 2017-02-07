function [ structTractionController, inputMat ] = traction_control( structTractionController, structRBE, structDTKF, tractor, inputMat, nConstantMT865, timeStepNo, nTimeParam )

% ------------------ Unpack time parameters -------------------------------
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;
currentSimTime = time(timeStepNo);

% ------------------- Unpack Needed Tractor Paramters ---------------------
rollingRadiusM = nConstantMT865.rollingRadiusM;
normalForceN = nConstantMT865.weightTractorN;
nGR = nConstantMT865.nGearRatio;
FD = nConstantMT865.finalDriveRatio;

% ------------------ Unpack Needed Tractor Inputs -------------------------
inputkm1 = inputMat(timeStepNo-1,:);
inputk = inputMat(timeStepNo,:);
gearNoOld = inputkm1(1,2);
GR = nGR(gearNoOld);

% ------------------- Unpack Needed Tractor States ------------------------
x = tractor.state;
engSpeedRadPS = x(10);
structTractionController.engSpeedRadPS(timeStepNo,1) = engSpeedRadPS;

% ------------------- Unpack parameters from DTKF -------------------------
xHatPlus = structDTKF.xHatPlus(:,timeStepNo);
vHat = xHatPlus(1);
omegaHat = xHatPlus(2);
F_TNetHat = xHatPlus(3);
tau_ResHat = xHatPlus(5);
DBHat = xHatPlus(7); % notation is also R_S for drawbar pull

smoothedvHat = structDTKF.smoothedvHat(1,timeStepNo);

slipHat = structDTKF.slipHat(1,timeStepNo);
slipHatSmooth = structDTKF.slipHatSmooth(1,timeStepNo);

% ---------------------- Unpack RBE Parameters ----------------------------
peakSlip = structRBE.peakSlip(1,timeStepNo);

% --------- Unpack Controller Structure: structTractionController ---------
tractionControlIsOn = structTractionController.tractionControlIsOn(timeStepNo-1);
gearShiftControlCountInt = structTractionController.gearShiftControlCountInt(timeStepNo-1);
gearShiftControlUpdateRateHz = structTractionController.gearShiftControlUpdateRateHz;

errorOmegaIntegrated = structTractionController.errorOmegaIntegrated(timeStepNo-1,:);
throttleFeedForwardm1 = structTractionController.throttleFeedForward(timeStepNo-1,1);

lastThrottleInputTC = structTractionController.lastThrottleInputTC;

TChasBeenTurnedOnAtLeastOnce = structTractionController.TChasBeenTurnedOnAtLeastOnce;

minimumEngineSpeedRPMRef = structTractionController.minimumEngineSpeedRPMRef;
minimumEngineSpeedRadPSRef = minimumEngineSpeedRPMRef*((2*pi)/60);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ====== Logic for Activating/Deactivating Traction Controller ============
if tractionControlIsOn
    if slipHatSmooth < 5
        tractionControlIsOn = 0;
    end
    if tractionControlIsOn 
        feedForwardString = 'usePastValue';         
    end
elseif ~tractionControlIsOn
    if slipHatSmooth > 25
        tractionControlIsOn = 1;
    end
    if tractionControlIsOn   % 
        feedForwardString = 'useDriverValue'; % use driver's throttle command for feedforward throttle
    end
end
% =========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('controlIsOn = %f \n',tractionControlIsOn)
if tractionControlIsOn
    
    % ------------------ Gear Shift Controller ----------------------------
    [gearShiftControlCountInt, gearShiftFlag] = controller_counter_func(gearShiftControlUpdateRateHz, gearShiftControlCountInt, timeStepS);
    if gearShiftFlag
        [ gearNoNew ] = gear_shift_controller( structTractionController, gearNoOld, nConstantMT865, timeStepNo, timeStepS );
        structTractionController.gearNo(timeStepNo,1) = gearNoNew;
        inputk(1,2) = gearNoNew;
    else
        if ~TChasBeenTurnedOnAtLeastOnce
            gearNoNew = gearNoOld;
            inputk(1,2) = gearNoNew;
        else
            gearNoNew = structTractionController.gearNo(timeStepNo-1,1);
            inputk(1,2) = gearNoNew;
        end
        structTractionController.gearNo(timeStepNo,1) = gearNoNew;
    end
    
    % -------------- Compute Track Speed Input ----------------------------
    iref = peakSlip;
    omegaRef = smoothedvHat/( rollingRadiusM*(1-(iref/100)) );
    floorOmegaRef = minimumEngineSpeedRadPSRef/(GR*FD);
    if (omegaRef < floorOmegaRef) % Need to compute minimum for gear selection
        omegaRef = floorOmegaRef;
    end
    
    % -------------------- Compute Feed Forward Term ----------------------
    if strcmp(feedForwardString, 'usePastValue')
        throttleFeedForward = throttleFeedForwardm1;
    elseif strcmp(feedForwardString, 'useDriverValue')
        throttleFeedForward = inputkm1(1);
    end
    
    % --------------------- PI CONTROLLER ---------------------------------
    errorOmega = omegaRef - omegaHat;
    [throttleControllerPID] = PID_F_Control(structTractionController, errorOmega, gearNoOld, timeStepNo);
    errorOmegaIntegrated = errorOmegaIntegrated + errorOmega*timeStepS;
    throttleControllerPIDFF = throttleFeedForward + throttleControllerPID;
    if throttleControllerPIDFF > 1 
        throttleControllerPIDFF = 1;
    elseif throttleControllerPIDFF < 0
        throttleControllerPIDFF = 0;
    end
    
    % --------- EXPORT THROTTLE CONTROLLER COMMAND TO INPUT ---------------
    inputk(1,1) = throttleControllerPIDFF;
    
    % ---- Store last throttle input and gear used by traction controller--
    TChasBeenTurnedOnAtLeastOnce = 1;
    lastThrottleInputTC = throttleControllerPIDFF;
    lastGearInputTC = gearNoNew;
    
elseif ~tractionControlIsOn
    
    % ------------------ Gear Shift Controller ----------------------------
    [gearShiftControlCountInt, gearShiftFlag] = controller_counter_func(gearShiftControlUpdateRateHz, gearShiftControlCountInt, timeStepS);
        if ~TChasBeenTurnedOnAtLeastOnce % if the Traction Controller has not come on yet in the simulation
            gearNoNew = gearNoOld;
        elseif TChasBeenTurnedOnAtLeastOnce && (currentSimTime < 20)
            gearNoNew = inputk(1,2);
        elseif TChasBeenTurnedOnAtLeastOnce && (currentSimTime >= 20)
            if gearShiftFlag
                [ gearNoNew ] = gear_shift_controller( structTractionController, gearNoOld, nConstantMT865, timeStepNo, timeStepS );
                structTractionController.gearNo(timeStepNo,1) = gearNoNew;
                inputk(1,2) = gearNoNew;
            else
                gearNoNew = gearNoOld;
            end
        end
        inputk(1,2) = gearNoNew;
        structTractionController.gearNo(timeStepNo,1) = gearNoNew;
    
    % ----------- Use last output from TC once it turns off ---------------
    throttleInput = lastThrottleInputTC;
    if ~TChasBeenTurnedOnAtLeastOnce
        throttleInput = inputkm1(1,1);
    elseif TChasBeenTurnedOnAtLeastOnce && (currentSimTime < 20)
        throttleInput = inputk(1,1);
    elseif TChasBeenTurnedOnAtLeastOnce && (currentSimTime >=20)
        throttleInput = lastThrottleInputTC;
    end
    inputk(1,1) = throttleInput;
    
    errorOmegaIntegrated = 0;
    errorOmega = 0;
    throttleFeedForward = NaN;
    %torqueFeedForwardNM = NaN;
    iref = NaN;
    omegaRef = NaN;
    throttleControllerPIDFF = 0;
    throttleControllerPID = 0;
    
    
end

% ------------------- Input Vector For Next Time Step ---------------------
inputMat(timeStepNo,:) = inputk;

% ------------------- Pack Up structTractionController --------------------
structTractionController.errorOmegaIntegrated(timeStepNo,1) = errorOmegaIntegrated;
structTractionController.errorOmega(timeStepNo,1) = errorOmega;
structTractionController.throttleFeedForward(timeStepNo,1) = throttleFeedForward;
%structTractionController.torqueFeedForwardNM(timeStepNo,1) = torqueFeedForwardNM;
structTractionController.peakSlip(timeStepNo,1) = peakSlip;
structTractionController.iref(timeStepNo,1) = iref;
structTractionController.omegaRef(timeStepNo,1) = omegaRef; 
structTractionController.throttleControllerPIDFF(timeStepNo,1) = throttleControllerPIDFF;
structTractionController.throttleControllerPID(timeStepNo,1) = throttleControllerPID;
structTractionController.tractionControlIsOn(timeStepNo,1) = tractionControlIsOn;
%structTractionController.gearShiftControlIsOn(timeStepNo,1) = gearShiftControlIsOn;
structTractionController.gearShiftControlCountInt(timeStepNo,1) = gearShiftControlCountInt;
structTractionController.gearShiftFlag(timeStepNo,1) = gearShiftFlag;
structTractionController.lastThrottleInputTC = lastThrottleInputTC;

structTractionController.TChasBeenTurnedOnAtLeastOnce = TChasBeenTurnedOnAtLeastOnce;

fprintf('gearNo = %f, engine RPM = %f \n', inputk(1,2), engSpeedRadPS*(60/(2*pi)) )
end


function [throttleControllerPID] = PID_F_Control(structTractionController, errorOmega, gear, timeStepNo)

% --------------------- Unpack Needed Parameters --------------------------
% Controller Parameters
sysControllerD = structTractionController.sysControllerD;
sysControllerDTFnum = sysControllerD.num{:,:,gear,1};
sysControllerDTFden = sysControllerD.den{:,:,gear,1};
%fprintf('a = %f %f %f \n',sysControllerDTFden)
%fprintf('b = %f %f %f \n',sysControllerDTFnum)

b0 = sysControllerDTFnum(1,1);
b1 = sysControllerDTFnum(1,2);
b2 = sysControllerDTFnum(1,3);

a0 = sysControllerDTFden(1,1);
a1 = sysControllerDTFden(1,2);
a2 = sysControllerDTFden(1,3);

% Look up past error values and inputs
ek = errorOmega;
if (timeStepNo-1) <= 0
    ekm1 = 0;
    ukm1 = 0;
else
    ekm1 = structTractionController.errorOmega(timeStepNo-1,1);
    ukm1 = structTractionController.throttleControllerPID(timeStepNo-1,1);
end
    
if (timeStepNo-2) <= 0
    ekm2 = 0;
    ukm2 = 0;
else
    ekm2 = structTractionController.errorOmega(timeStepNo-2,1);
    ukm2 = structTractionController.throttleControllerPID(timeStepNo-2,1);
end   

% ------------------- Compute Control Input -------------------------------
throttleControllerPID = (1/a0)*(-ukm1*a1 - ukm2*a2...
    + b0*ek + b1*ekm1 + b2*ekm2);


end

















% function [throttleFeedForward, peakSlipNoLoad] = compute_feed_forward_throttle(structRBE, nConstantMT865, engSpeedRadPS, GR, timeStepNo)
% 
% % --------------------- Unpack Constant Tractor Parameters ----------------
% FD = nConstantMT865.finalDriveRatio;
% 
% % ---------------------- Unpack RBE Parameters ----------------------------
% terrainParameterEstimate = structRBE.parameterEstimate(:,timeStepNo);
% slipVectorBayes = structRBE.slipVectorBayes;
% 
% % ----------------- Calculate Peak Slip and Steady-State tauRes -----------
% [~, peakSlipNoLoad, ~, peakSlipLoad, tauResPk] = peak_traction(nConstantMT865, terrainParameterEstimate, slipVectorBayes, 'MaxTraction');
% isSlipDiscrepancy = ( abs(peakSlipNoLoad - peakSlipLoad) > eps);
%     if isSlipDiscrepancy
%         error('Error: peak slip discrepancy, check function peak_traction')
%     end
% 
% % ------ Calculate Feed-Forward Throttle Command based on tauResPk ----
% engineTorqueMaxNM  = engine_interp( 1, 0, engSpeedRadPS, nConstantMT865 );
% driverTorqueMaxNM = engineTorqueMaxNM*GR*FD;
% throttleFeedForward = tauResPk/driverTorqueMaxNM;
%     if engSpeedRadPS > 2100*((2*pi)/60);
%         throttleFeedForward = 0;
%     elseif engSpeedRadPS < 1200*((2*pi)/60);
%         throttleFeedForward = 0.3;
%     end
%     
%     if throttleFeedForward > 1
%         throttleFeedForward = 1;
%     elseif throttleFeedForward < 0
%         throttleFeedForward = 0;
%     end
%         
% end
