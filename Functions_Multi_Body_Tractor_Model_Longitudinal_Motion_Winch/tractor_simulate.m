function [tractor, controlArchitecture,inputMat,nTimeParam] = tractor_simulate(tractor, inputMat, nConstantMT865, nConstantTerrain, nTimeParam, controlArchitecture)

% ------------------- Unpack Simulation Time Paramters --------------------
nTimeStep = nTimeParam.nTimeStep;
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;

% ------------------- Unpack controlArchitecture --------------------------
structDTKF = controlArchitecture.structDTKF;
structRBE = controlArchitecture.structRBE;
structTractionController = controlArchitecture.structTractionController;
structWinchController = controlArchitecture.structWinchController;


% -------------------- Simulation Loop ------------------------------------
for timeStepNo = 2:nTimeStep
    
   % Tractor Dynamics
   tractor(timeStepNo) = motion_tractor(tractor(timeStepNo-1),inputMat(timeStepNo-1,:),nConstantMT865,nConstantTerrain,timeStepS);
   %[tractor(timeStepNo)] = detect_valve_change(tractor(timeStepNo),inputMat(timeStepNo,:),inputMat(timeStepNo-1,:)); 
  
   if structDTKF.FlagDTKFisOn
   structDTKF = propogate_DTKF(structDTKF, tractor(timeStepNo), nConstantMT865, inputMat(timeStepNo-1,:).', timeStepNo, 'Koffline');
   end
   
   if structRBE.FlagRBEisOn
   structRBE = bayes_estimation(structRBE, structDTKF, nConstantMT865, time, timeStepNo);
   end
   
   if structWinchController.FlagWCisOn
   [tractor(timeStepNo), inputMat(timeStepNo,:)] = winch_controller(tractor(timeStepNo),inputMat(timeStepNo,:),inputMat(timeStepNo-1,:),nConstantMT865,structDTKF,structTractionController,structWinchController,timeStepNo);
   end
   
   if structTractionController.FlagTCisOn
   [ structTractionController, inputMat ] = traction_control( structTractionController, structRBE, structDTKF, tractor(timeStepNo), inputMat, nConstantMT865, timeStepNo, nTimeParam );
   end
   
%    tractorIsStuck = check_if_tractor_stuck(structDTKF,timeStepNo);
%    if tractorIsStuck == 1
%        break
%    end
   
  fprintf('Simulation Time: %f seconds \n',(timeStepNo-1)*timeStepS)       
end


% ------------------ Pack Up controlArchitecture -------------------------
controlArchitecture.structDTKF = structDTKF;
controlArchitecture.structRBE = structRBE;
controlArchitecture.structTractionController = structTractionController;
controlArchitecture.structWinchController = structWinchController;

% ------------------- Time Param Structure --------------------------------
nTimeParam.nTimeStep = timeStepNo;

end

function isStuckFlag =  check_if_tractor_stuck(structDTKF,timeStepNo)

xHatPlus = structDTKF.xHatPlus(:,timeStepNo);
vHat = xHatPlus(1);

if vHat <= 0
    isStuckFlag = 1;
else
    isStuckFlag = 0;
end

end