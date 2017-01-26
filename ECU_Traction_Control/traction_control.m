function [ structTractionController, input ] = traction_control( structTractionController, structRBE, structDTKF, tractor, input, nConstantMT865, timeStepNo, timeStepS )


% ------------------- Unpack Needed Tractor Paramters ---------------------
rollingRadiusM = nConstantMT865.rollingRadiusM;
normalForceN = nConstantMT865.weightTractorN;
nGR = nConstantMT865.nGearRatio;
FD = nConstantMT865.finalDriveRatio;

% ------------------ Unpack Needed Tractor Inputs -------------------------
gear = input(2);
GR = nGR(gear);

% ------------------- Unpack Needed Tractor States ------------------------
x = tractor.state;
engSpeedRadPS = x(10);

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
gearShiftControlIsOn = structTractionController.gearShiftControlIsOn(timeStepNo-1);
gearShiftControlCountInt = structTractionController.gearShiftControlCountInt(timeStepNo-1);
gearShiftControlUpdateRateHz = structTractionController.gearShiftControlUpdateRateHz;

errorOmegaIntegrated = structTractionController.errorOmegaIntegrated(timeStepNo-1,:);
throttleFeedForwardm1 = structTractionController.throttleFeedForward(timeStepNo-1,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ====== Logic for Activating/Deactivating Traction Controller ============
if tractionControlIsOn
    if slipHatSmooth < 5
        tractionControlIsOn = 0;
        %gearShiftControlIsOn = 0;
        %gearShiftControlCountInt = 0;
    end
    if tractionControlIsOn 
        feedForwardString = 'usePastValue';         
    end
elseif ~tractionControlIsOn
    if slipHatSmooth > 25
        tractionControlIsOn = 1;
        gearShiftControlIsOn = 1;
        %gearShiftControlCountInt = (gearShiftControlUpdateRateHz/timeStepS) - 1;
    end
    if tractionControlIsOn   % 
        feedForwardString = 'useDriverValue'; % use driver's throttle command for feedforward throttle
    end
end
% =========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('controlIsOn = %f \n',tractionControlIsOn)
if tractionControlIsOn
    
    % -------------- Compute Track Speed Input ----------------------------
    iref = peakSlip;
    omegaRef = smoothedvHat/( rollingRadiusM*(1-(iref/100)) );
    if omegaRef < 0 % Need to compute minimum for gear selection
        omegaRef = 1;
    end
    
    % ------------------ Gear Shift Controller ----------------------------
    [gearShiftControlCountInt, gearShiftFlag] = controller_counter_func(gearShiftControlUpdateRateHz, gearShiftControlCountInt, timeStepS);
    if gearShiftFlag
        [ gearNo ] = gear_shift_controller( structTractionController, omegaRef, nConstantMT865 );
        structTractionController.gearNo(timeStepNo,1) = gearNo;
        input(2) = gearNo;
    else
        structTractionController.gearNo(timeStepNo,1) = structTractionController.gearNo(timeStepNo-1,1);
        gearNo = structTractionController.gearNo(timeStepNo,1);
        if gearNo == 0
            gearNo = gear;
            input(2) = gear;
        else
            input(2) = gearNo;
        end
    end
    
    % -------------------- Compute Feed Forward Term ----------------------
    if strcmp(feedForwardString, 'usePastValue')
        throttleFeedForward = throttleFeedForwardm1;
    elseif strcmp(feedForwardString, 'useDriverValue')
        %[throttleFeedForward, peakSlipNoLoad] = compute_feed_forward_throttle(structRBE, nConstantMT865, engSpeedRadPS, gearNo, timeStepNo);
        throttleFeedForward = input(1);
    end
    
    % --------------------- PI CONTROLLER ---------------------------------
    %Kp = structTractionController.KpGR(gearNo);
    %Ki = structTractionController.KiGR(gearNo);
    errorOmega = omegaRef - omegaHat;
    [throttleControllerPID] = PID_F_Control(structTractionController, errorOmega, gear, timeStepNo);
    errorOmegaIntegrated = errorOmegaIntegrated + errorOmega*timeStepS;
    throttleControllerPIDFF = throttleFeedForward + throttleControllerPID;
    if throttleControllerPIDFF > 1 
        throttleControllerPIDFF = 1;
    elseif throttleControllerPIDFF < 0
        throttleControllerPIDFF = 0;
    end
    
    % --------- EXPORT THROTTLE CONTROLLER COMMAND TO INPUT ---------------
    input(1) = throttleControllerPIDFF;
    
elseif ~tractionControlIsOn
    
    % ------------------ Gear Shift Controller ----------------------------
    [gearShiftControlCountInt, gearShiftFlag] = controller_counter_func(gearShiftControlUpdateRateHz, gearShiftControlCountInt, timeStepS);
    if gearShiftFlag
        [ gearNo ] = gear_shift_controller( structTractionController, omegaHat, nConstantMT865 );
        structTractionController.gearNo(timeStepNo,1) = gearNo;
        input(2) = gearNo;
    else
        structTractionController.gearNo(timeStepNo,1) = structTractionController.gearNo(timeStepNo-1,1);
        gearNo = structTractionController.gearNo(timeStepNo,1);
        if gearNo == 0
            gearNo = gear;
            input(2) = gear;
        else
            input(2) = gearNo;
        end
    end
    
    errorOmegaIntegrated = 0;
    errorOmega = 0;
    throttleFeedForward = NaN;
    %torqueFeedForwardNM = NaN;
    iref = NaN;
    omegaRef = NaN;
    throttleControllerPIDFF = 0;
    throttleControllerPID = 0;
    
    gearShiftFlag = NaN;
    
end

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
structTractionController.gearShiftControlIsOn(timeStepNo,1) = gearShiftControlIsOn;
structTractionController.gearShiftControlCountInt(timeStepNo,1) = gearShiftControlCountInt;
structTractionController.gearShiftFlag(timeStepNo,1) = gearShiftFlag;

fprintf('engine RPM = %f \n', engSpeedRadPS*(60/(2*pi)) )
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
    + b0*ek + b1*ekm1 + b2*ekm2)


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
