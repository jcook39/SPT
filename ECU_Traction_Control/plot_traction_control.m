function plot_traction_control( tractor, nConstantMT865, structTractionController, structDTKF, input, nTimeParam, tractorNo, figureNo )

% This function plots results from the traction controller implemented
% through the ECU

% ------------------- Unpack Simulation Time Paramters --------------------
nTimeStep = nTimeParam.nTimeStep;
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;
indexVector = 1:nTimeStep;
timeVector = (indexVector-1)*timeStepS;

% ------------------- Unpack Tractor Parameters ---------------------------
engineTorqueDataNM = nConstantMT865.engineTorqueDataNM;
engineSpeedDataRadPS = nConstantMT865.engineSpeedDataRadPS;
nGearRatio = nConstantMT865.nGearRatio(1:16);
finalDriveRatio = nConstantMT865.finalDriveRatio;

rollingRadiusM = nConstantMT865.rollingRadiusM;

% --------------- Unpack True Tractor States ------------------------------
for i = 1:nTimeStep
    slipLeft(i) = tractor(i).slip(1);
    slipRight(i) = tractor(i).slip(2);
    wl(i) = tractor(i).state(7);
    wr(i) = tractor(i).state(8);
end
slip = (slipLeft + slipRight)./2;
w = ( wl + wr )./2;

usedGearNo = input(1:nTimeStep,2);

% ------------------ Unpack DTKF Structure --------------------------------
xHatPlus = structDTKF.xHatPlus;
vHat = xHatPlus(1,:);
omegaHat = xHatPlus(2,:);
F_TNetHat = xHatPlus(3,:);
tau_ResHat = xHatPlus(5,:);
DBHat = xHatPlus(7,:); % notation is also R_S for drawbar pull

% slipHat Calculation
slipHat = structDTKF.slipHat;
slipHatSmooth = structDTKF.slipHatSmooth;

% ------------------ Unpack Controller Structure --------------------------
errorOmegaIntegrated = structTractionController.errorOmegaIntegrated;
errorOmega = structTractionController.errorOmega;
%throttleFeedForward = structTractionController.throttleFeedForward;

%peakSlipNoLoad = structTractionController.peakSlip;
iref = structTractionController.iref;
omegaRef = structTractionController.omegaRef; 
throttleControllerPIDF = structTractionController.throttleControllerPIDFF;
tractionControlIsOn = structTractionController.tractionControlIsOn;

gearNo = structTractionController.gearNo;
gearShiftFlag = structTractionController.gearShiftFlag;
gearShiftControlCountInt = structTractionController.gearShiftControlCountInt;


% ------- Plot Track Speed over allowable Track Speed in each GR ----------
engineSpeedRangeMinRadPS = 1400*((2*pi)/60);
engineSpeedRangeMaxRadPS = 2000*((2*pi)/60);
indexMin = find( engineSpeedDataRadPS == engineSpeedRangeMinRadPS );
indexMax = find( engineSpeedDataRadPS == engineSpeedRangeMaxRadPS );
engineSpeedRangeRadPS = engineSpeedDataRadPS( indexMin:indexMax );
engineTorqueRangeNM = engineTorqueDataNM( indexMin:indexMax );
enginePowerRangeW = engineSpeedRangeRadPS.*engineTorqueRangeNM;
totalGR = nGearRatio*finalDriveRatio;

% Driver Speed Range for Each gear,  speedMat = [gear x driverSpeed]
nRow = numel(engineSpeedRangeRadPS);
nCol = numel(totalGR);
trackSpeedMat = zeros(nRow,nCol);
for i = 1:numel(nGearRatio)
    trackSpeedMat(:,i) = engineSpeedRangeRadPS.' / totalGR(i);
end
gearNoRow = [1:16];
gearNoMat = repmat(gearNoRow,numel(engineSpeedRangeRadPS),1);

powerMat = repmat( enginePowerRangeW.' , 1 , 16 );


% Plot 
% figure(figureNo+1)
% plot( trackSpeedMat, gearNoMat, omegaHat, usedGearNo, 'k:')

% --------------- line color for plot parameters --------------------------
trueColor = 'b';
measuredColor = 'k.';
estimatedColor = 'r';
smoothColor = 'g';
refColor = 'c';


figure(figureNo)
set(gcf,'numbertitle','off','name',['Terrian Tractor',num2str(tractorNo)])
subplot(231)
    h = plot(timeVector, slip, trueColor,...
    timeVector, slipHat, estimatedColor, timeVector, slipHatSmooth, smoothColor, timeVector, iref, refColor);
    set(h(4),'linewidth',4)
    legend('slip reference','slip actual','estimated slip')
subplot(232)
    h = plot(timeVector, w, trueColor, timeVector, omegaHat, estimatedColor, timeVector, omegaRef, refColor);
    set(h(3),'linewidth',4)
    legend('\omega_{ref}','\omega','\omega_{est}')
subplot(233)
    plot(timeVector, throttleControllerPIDF)
    ylabel('throttle Controller')
subplot(234)
    plot(timeVector, errorOmega)
    ylabel(' error \omega')
subplot(235)
    plot(timeVector, errorOmegaIntegrated)
    ylabel(' error integrated \omega')
subplot(236)
    plot(timeVector, tractionControlIsOn)
    ylabel('control off/on')
    
    
% figure(figureNo+2)
% subplot(311)
%     plot(timeVector,gearNo)
% subplot(312)
%     plot(timeVector, gearShiftFlag)
% subplot(313)
%     plot(timeVector, gearShiftControlCountInt)


end

