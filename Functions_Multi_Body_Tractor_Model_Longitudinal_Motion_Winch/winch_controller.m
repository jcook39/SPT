function [MT865,input,structWinchController] = winch_controller(MT865,input,inputMinus1,nConstantMT865,structDTKF,structTractionController,structWinchController,nTimeParam,timeStepNo)

% ------------------- Unpack nTimeParam Struct ----------------------------
time = nTimeParam.time;
timeStepS = nTimeParam.timeStepS;

% ------------------------ Unpack Needed Constants ------------------------
rs = nConstantMT865.rollingRadiusM;
rw = nConstantMT865.winchRadiusM;

% ----------------- Unpack Needed Input, State and Other ------------------
valvePosition = input(5);
pSet = input(6);

wl = MT865.state(7);
vx = MT865.state(4);
psiWinchRad = MT865.state(15);
psiWinchRadPS = MT865.state(16);

% ---------------------  Unpack DTKF Parameters ---------------------------
xHatPlus = structDTKF.xHatPlus(:,timeStepNo);
vHat = xHatPlus(1);
smoothedvHat = structDTKF.smoothedvHat(1,timeStepNo); 

slipHat = structDTKF.slipHat(1,timeStepNo);
slipHatSmooth = structDTKF.slipHatSmooth(1,timeStepNo);

% ---------------- Unpack Traction Controller -----------------------------
tractionControlIsOn = structTractionController.tractionControlIsOn(timeStepNo,1);
tractionControlIsOff = ~tractionControlIsOn;

% ------------------- Unpack structWinchController ------------------------
% Threshold limits for controller activation/deactivation
vHatLowLim = structWinchController.vHatLowLim;
slipHatHighLim = structWinchController.slipHatHighLim;

% Whether the winch has been activated 
winchControlIsOn = structWinchController.winchControlIsOn(timeStepNo-1,1);
winchControlIsOff = ~winchControlIsOn;

% Winch controller update rate and counters 
winchControllerUpdateRateHz = structWinchController.winchControllerUpdateRateHz;
winchControllerUpdateRateSec = 1/winchControllerUpdateRateHz;
winchControllerCountInt = structWinchController.winchControllerCountInt(timeStepNo-1,1);

% pressure reference value winch controller
winchControllerpSetRefm1 = structWinchController.winchControllerpSetRef(timeStepNo-1,1);
winchControllerpSetIncrementPSI = structWinchController.winchControllerpSetIncrementPSI(timeStepNo-1,:);
winchControllerpSetIncrementPSIMax = structWinchController.winchControllerpSetIncrementPSIMax;
winchControllerpSetIncrementPSIMin = structWinchController.winchControllerpSetIncrementPSIMin;
winchControllerpSetPSIReduceDBFast = structWinchController.winchControllerpSetPSIReduceDBFast;

attenuationFactor = structWinchController.attenuationFactor;

psi2Pa = structWinchController.psi2Pa;

% -------------------- Winch Decision/Control -----------------------------
tractorBelowSpeedThreshold = (vHatLowLim >= smoothedvHat); 

% Maximum Slip Threshold
tractorIsAboveMaxSlipThreshold = (slipHatHighLim < slipHat);
tractorIsBelowMaxSlipThreshold = ~tractorIsAboveMaxSlipThreshold;

% Incremental Slip Threshold Points
nTimeStepLookBack = winchControllerUpdateRateSec/timeStepS;
timeStepNoPastStart = timeStepNo - nTimeStepLookBack;
if timeStepNoPastStart < 1; timeStepNoPastStart = 1; end
slipHatPast = structDTKF.slipHat(1,timeStepNoPastStart:timeStepNo);
slipHatSmoothPast = structDTKF.slipHatSmooth(1,timeStepNoPastStart:timeStepNo);

% 
slipHatHasBeenAbove2ndUpperLimit = (sum(slipHatPast > 35) > 0);
slipHatHasBeenAbove3rdUpperLimit = (sum(slipHatPast > 20) > 0);

% ----------- Logic to turn winch control off and on ----------------------
winchControlJustTurnedOn = 0;
if winchControlIsOff && (time(timeStepNo) > 5)
    if tractorIsAboveMaxSlipThreshold && tractionControlIsOn
       winchControlIsOn = 1;
       winchControllerCountInt = 0;
       pSet = 0*6894.76;
       winchControllerpSetRefm1 = pSet;
       fprintf('WINCH CONTROL TURNED ON!!!\n')
       winchControlJustTurnedOn = 1;
    end
elseif winchControlIsOn
    if tractionControlIsOff && (psiWinchRad*rw == 0)
       winchControlIsOn = 0; 
       fprintf('WINCH CONTROL TURNED OFF!!!\n')
    end
end

% ----- Input Value used based on winch controller being on/off -----------

valvePositionMinus1 = inputMinus1(1,5);
if winchControlIsOn
    valvePosition = valvePositionMinus1;
elseif winchControlIsOff
    valvePosition = input(1,5);
end

% ----------------- Detect Needed Change in Valve Position ----------------
if winchControlIsOn && ~winchControlJustTurnedOn
    if valvePosition == 2
        winchIsLocked = abs(psiWinchRadPS - 0) < eps;
        if winchIsLocked
           valvePosition = 1;
           %winchControllerpSetRefm1 = 0.1*winchControllerpSetRefm1;
           winchControllerpSetRefm1 =  winchControllerpSetIncrementPSI*6894.76;
           pSet = winchControllerpSetRefm1;
           input(1,1) = input(1,1) + 0.3;           
        end
    end
end
    
    

% ------------------ Controller Counter and Action ------------------------
%if valvePosition == 2
    if winchControlIsOn
        [winchControllerCountInt, winchControllerFlag] = controller_counter_func_winch(winchControllerUpdateRateHz, winchControllerCountInt, timeStepS);
        % ______________________ Regular Winch Controller Update ______________
        if winchControllerFlag
            % ____________ Tractor is slipping way too much ___________________
            if tractorIsAboveMaxSlipThreshold
                pSet = 0*6894.76;
                winchControllerpSetIncrementPSI = reduce_pSet_increment(winchControllerpSetIncrementPSI, attenuationFactor, winchControllerpSetIncrementPSIMax, winchControllerpSetIncrementPSIMin);
                fprintf('WC RESET PSET TO ZERO !!! \n')
            % ____________ Tractor isnt slipping super excessively ____________    
            elseif tractorIsBelowMaxSlipThreshold
                if tractionControlIsOn
                    % _____________ Tractor is about to slip excessively ______
                    if slipHatHasBeenAbove2ndUpperLimit
                        if winchControllerpSetRefm1 <= 0
                            pSet = 0;
                        else
                            pSet = winchControllerpSetRefm1 - winchControllerpSetPSIReduceDBFast*6894.76;
                            if pSet > 0                                 
                                winchControllerpSetIncrementPSI = reduce_pSet_increment(winchControllerpSetIncrementPSI, attenuationFactor, winchControllerpSetIncrementPSIMax, winchControllerpSetIncrementPSIMin);
                            end
                            pSet = pSet*(pSet >= 0);
                        end
                        fprintf('SLIP HAS EXCEEDED 35 REDUCE PSET \n')
                    % ___ Tractor Slipping just the right amount (maybe) ______   
                    else
                        pSet = winchControllerpSetRefm1;
                        fprintf('MAINTAIN SLIP SET POINT')
                    end
                elseif tractionControlIsOff
                    if slipHatHasBeenAbove3rdUpperLimit
                        pSet = winchControllerpSetRefm1;
                        fprintf('TC off MAINTAIN PSET \n')
                    else
                        pSet = winchControllerpSetRefm1 + winchControllerpSetIncrementPSI*6894.76;    
                    end
                end
            end

        elseif ~winchControllerFlag
            if tractorIsBelowMaxSlipThreshold
                pSet = winchControllerpSetRefm1;
            elseif (pSet ~= 0*6894.76) && tractorIsAboveMaxSlipThreshold
                pSet = 0*6894.76;
                winchControllerCountInt = 0;
            end
        end
        fprintf('Winch is activated \n')

    elseif ~winchControlIsOn
        winchControllerCountInt = 0;
        winchControllerFlag = 0;
    end
%end

input(1,5) = valvePosition;
discreteValvePosition = valvePosition;

input(1,6) = pSet;

% ----------- Detect if there's change in discrete valve position ---------
[MT865] = detect_valve_change(MT865,input,inputMinus1);

% ------------------------ Winch Commands ---------------------------------
% Pulling in Winch
if valvePosition == 1
    pSetDisPump = pSet;
    pSetBrakeValve = 0;
end

% Braking Winch
if valvePosition == 2
    pSetBrakeValve = pSet;
    pSetDisPump = 0;
end

% Let Sled Out
if valvePosition == 3
    pSetDisPump = 0;
    pSetBrakeValve = 0;
end
    
% ------------------------- Pack Up Output --------------------------------
MT865.pSetDisPump = pSetDisPump;
MT865.pSetBrakeValve = pSetBrakeValve;

% --------------------- Pack up Winch controller structure ----------------
structWinchController.discreteValvePosition(timeStepNo,1) = discreteValvePosition;
structWinchController.winchControlIsOn(timeStepNo,1) = winchControlIsOn;
structWinchController.winchControllerCountInt(timeStepNo,1) = winchControllerCountInt;
structWinchController.winchControllerFlag(timeStepNo,1) = winchControllerFlag;
structWinchController.winchControllerpSetRef(timeStepNo,1) = pSet;
fprintf('pSet = %f psi \n',pSet/psi2Pa)
structWinchController.winchControllerpSetIncrementPSI(timeStepNo,:) = winchControllerpSetIncrementPSI;

end

function [countInt, controllerFlag] = controller_counter_func_winch(updateRateHz, countInt, timeStepS)
% This is general purpose function for determining whether the controller
% should be active at a given time step based on its update rate. This is
% so that the simulation time step "timeStepS" can run at a different rate
% that a given controller "updateRateHz"


% ------------ Increment Counter and Compute Time Since Last Control Action
updateRateSec = 1/updateRateHz;
countInt = countInt + 1;
timeSinceLastControl = timeStepS*countInt;

% ----------------- Logic for Controller Activation -----------------------
if timeSinceLastControl >= updateRateSec
    countInt = 0;
    controllerFlag = 1;
else
    controllerFlag = 0;
end



end


function winchControllerpSetIncrementPSI = reduce_pSet_increment(winchControllerpSetIncrementPSI, attenuationFactor, winchControllerpSetIncrementPSIMax, winchControllerpSetIncrementPSIMin)

winchControllerpSetIncrementPSI = attenuationFactor*winchControllerpSetIncrementPSI;

if winchControllerpSetIncrementPSI > winchControllerpSetIncrementPSIMax
    winchControllerpSetIncrementPSI = winchControllerpSetIncrementMax;
elseif winchControllerpSetIncrementPSI > winchControllerpSetIncrementPSIMax
    winchControllerpSetIncrementPSI = winchControllerpSetIncrementPSIMin;
end
    
end