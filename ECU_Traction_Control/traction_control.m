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
terrainParameterEstimate = structRBE.parameterEstimate(:,timeStepNo);
slipVectorBayes = structRBE.slipVectorBayes;

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
    
    % --------- Compute Reference Slip and Track Speed Input --------------
    [~, peakSlipNoLoad, ~, peakSlipLoad, ~] = peak_traction(nConstantMT865, terrainParameterEstimate, slipVectorBayes, 'MaxTraction');
    isSlipDiscrepancy = ( abs(peakSlipNoLoad - peakSlipLoad) > eps);
    if isSlipDiscrepancy
        error('Error: peak slip discrepancy, check function peak_traction')
    end
    iref = peakSlipNoLoad;
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
    Kp = structTractionController.KpGR(gearNo);
    Ki = structTractionController.KiGR(gearNo);
    errorOmega = omegaRef - omegaHat;
    errorOmegaIntegrated = errorOmegaIntegrated + errorOmega*timeStepS;
    throttleController = throttleFeedForward + Kp*errorOmega + Ki*errorOmegaIntegrated;
    if throttleController > 1 
        throttleController = 1;
    elseif throttleController < 0
        throttleController = 0;
    end
    
    % --------- EXPORT THROTTLE CONTROLLER COMMAND TO INPUT ---------------
    input(1) = throttleController;
    
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
    errorOmega = NaN;
    throttleFeedForward = NaN;
    %torqueFeedForwardNM = NaN;
    peakSlipNoLoad = NaN;
    iref = NaN;
    omegaRef = NaN;
    throttleController = 0;
    
    gearShiftFlag = NaN;
    
end

% ------------------- Pack Up structTractionController --------------------
structTractionController.errorOmegaIntegrated(timeStepNo,1) = errorOmegaIntegrated;
structTractionController.errorOmega(timeStepNo,1) = errorOmega;
structTractionController.throttleFeedForward(timeStepNo,1) = throttleFeedForward;
%structTractionController.torqueFeedForwardNM(timeStepNo,1) = torqueFeedForwardNM;
structTractionController.peakSlip(timeStepNo,1) = peakSlipNoLoad;
structTractionController.iref(timeStepNo,1) = iref;
structTractionController.omegaRef(timeStepNo,1) = omegaRef; 
structTractionController.throttleController(timeStepNo,1) = throttleController;
structTractionController.tractionControlIsOn(timeStepNo,1) = tractionControlIsOn;
structTractionController.gearShiftControlIsOn(timeStepNo,1) = gearShiftControlIsOn;
structTractionController.gearShiftControlCountInt(timeStepNo,1) = gearShiftControlCountInt;
structTractionController.gearShiftFlag(timeStepNo,1) = gearShiftFlag;

fprintf('engine RPM = %f \n', engSpeedRadPS*(60/(2*pi)) )
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
