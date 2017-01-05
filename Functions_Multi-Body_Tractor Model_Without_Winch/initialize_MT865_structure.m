function MT865 = initialize_MT865_structure(X,Y,theta,nConstantMT865,nConstantTerrain,startCondition,inputMat)

%Initialize Body States

if strcmp(startCondition,'zero')
    %Iniialize Body States
    phi = 0;
    si = 0;
    vx = 0;
    vy = 0;
    thetaDot = 0;
    phiDot = 0;
    siDot = 0;
    wl = 0;
    wr = 0;
    slipLeft = 0;
    slipRight = 0;
    bodyState = [X Y theta phi si vx vy thetaDot phiDot siDot wl wr].';

    % Initialize Power Train States
    engThrtl = 0.1;
    engSpeedRadPS = 1350*((2*pi)/60);
    clutchCom = 0;
    engCtrlThrtl = 0.1;
    steerAngleState = 0;
    pressureHPa = 0;
    powerTrainState = [engThrtl engSpeedRadPS clutchCom engCtrlThrtl steerAngleState pressureHPa].';

    state = [bodyState; powerTrainState];
    MT865.state = stae;
    
    % Initialize Structure
    MT865.slip = [slipLeft slipRight].';
    MT865.forces = zeros(8,1);
    MT865.transOutputSpeedRadSec = 0;
    MT865.transTorqueInputNM = 0;
    MT865.transTorqueOutputNM = 0;
    MT865.steerMotorTorqueNM = 0;
    MT865.clutchIsSlip = 1;
    MT865.torqueClutchNM = 0;
    MT865.isClutchSlip = 0;
    MT865.isGearShift = 0;
    MT865.posLeftFront = 0;
    MT865.posRightFront = 0;
    MT865.posLeftRear = 0;
    MT865.posRightRear = 0;
    MT865.terrainLeftFront = [0 0].'; % [frictionAngle cohesion].'
    MT865.terrainRightFront = [0 0].'; % [fricitonAngle cohesion].'
    MT865.terrainLeftRear = [0 0].'; % [frictionAngle cohesion.']
    MT865.terrainRightRear = [0 0].'; % [frictionANgle cohsion].'

    MT865.torqueLeftSprocketNM = 0;
    MT865.torqueRightSprocketNM = 0;
    MT865.xdot = zeros(size(state));
    MT865.sledForcexy = [0 0].';
    MT865.vSledxy = [0 0 0].';
    MT865.enginePowerW = 0;

    MT865 = position_nTrack( MT865, nConstantMT865 );
    MT865 = terrain_parametrs_track(MT865,nConstantTerrain);
end

if strcmp(startCondition,'steadyState')
    
    % Initialize Power Train States
    gearNoStart = inputMat(1,2);
    gearRatioStart = nConstantMT865.nGearRatio(gearNoStart);
    finalDriveRatio = nConstantMT865.finalDriveRatio;
    
    startEngineSpeedRadPS = 1800*(2*pi/60);
    slipStart = 0.05;
    trackSpeedStartRadPS = startEngineSpeedRadPS/(gearRatioStart*finalDriveRatio);
    startVehicleSpeedMPS = (1 - slipStart)*nConstantMT865.rollingRadiusM*trackSpeedStartRadPS;
    
    %Iniialize Body States
    phi = 0;
    si = 0;
    vx = startVehicleSpeedMPS;
    vy = 0;
    thetaDot = 0;
    phiDot = 0;
    siDot = 0;
    wl = trackSpeedStartRadPS;
    wr = trackSpeedStartRadPS;
    slipLeft = slipStart;
    slipRight = slipStart;
    bodyState = [X Y theta phi si vx vy thetaDot phiDot siDot wl wr].';
    
    engThrtl = inputMat(1,1);
    engSpeedRadPS = startEngineSpeedRadPS;
    clutchCom = 1;
    engCtrlThrtl = 0.1;
    steerAngleState = 0;
    pressureHPa = 0;
    powerTrainState = [engThrtl engSpeedRadPS clutchCom engCtrlThrtl steerAngleState pressureHPa].';

    state = [bodyState; powerTrainState];
    MT865.state = state;
    
    % Initialize Structure
    MT865.slip = [slipLeft slipRight].';
    MT865.forces = zeros(8,1);
    MT865.transOutputSpeedRadSec = 0;
    MT865.transTorqueInputNM = 0;
    MT865.transTorqueOutputNM = 0;
    MT865.steerMotorTorqueNM = 0;
    MT865.clutchIsSlip = 0;
    MT865.torqueClutchNM = 0;
    MT865.isClutchSlip = 0;
    MT865.isGearShift = 0;
    MT865.posLeftFront = 0;
    MT865.posRightFront = 0;
    MT865.posLeftRear = 0;
    MT865.posRightRear = 0;
    MT865.terrainLeftFront = [0 0].'; % [frictionAngle cohesion].'
    MT865.terrainRightFront = [0 0].'; % [fricitonAngle cohesion].'
    MT865.terrainLeftRear = [0 0].'; % [frictionAngle cohesion.']
    MT865.terrainRightRear = [0 0].'; % [frictionANgle cohsion].'

    MT865.torqueLeftSprocketNM = 0;
    MT865.torqueRightSprocketNM = 0;
    MT865.xdot = zeros(size(state));
    MT865.sledForcexy = [0 0].';
    MT865.vSledxy = [0 0 0].';
    MT865.enginePowerW = 0;

    MT865 = position_nTrack( MT865, nConstantMT865 );
    MT865 = terrain_parametrs_track(MT865,nConstantTerrain);
end

end