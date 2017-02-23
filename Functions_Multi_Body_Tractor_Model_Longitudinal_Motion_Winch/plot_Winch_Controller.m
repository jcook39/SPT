function plot_Winch_Controller(structWinchController, nTimeParam, tractorColor, figureNo)

% ------------- Unpack Struct of simulation time paramters ----------------
time = nTimeParam.time;
nTimeStep = nTimeParam.nTimeStep;
timeVector = time(1:nTimeStep);

% ------------ Unpack structWinchController -------------------------------
winchControllerpSetRef = structWinchController.winchControllerpSetRef(1:nTimeStep,1);
winchControllerpSetIncrementPSI = structWinchController.winchControllerpSetIncrementPSI(1:nTimeStep,1);
psi2Pa = structWinchController.psi2Pa;

figure(figureNo)
subplot(211)
    plot(timeVector, winchControllerpSetRef./psi2Pa,tractorColor)
    xlabel('time (seconds)')
    ylabel('winchControllerpSetRef')
    hold on
subplot(212)
    plot(timeVector, winchControllerpSetIncrementPSI, tractorColor)
    xlabel('time (seconds)')
    ylabel('winchControllerpSetIncrement')
    hold on

end