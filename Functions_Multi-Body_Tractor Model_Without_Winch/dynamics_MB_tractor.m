function [MT865] = dynamics_MB_tractor(MT865,input,nConstantMT865,nConstantTerrain,timeStepS)

% This function governs the multi-body dynamics for an AGCO MT865 tractor. 

%%%%%%%%%%%%%%%%%%%%%%%%%%% INTEGRATION ROUTINE %%%%%%%%%%%%%%%%%%%%%%%%%%%

integrationIsNotComplete = 1;
t0 = 0;
tf = timeStepS;
x0 = MT865.state;

while integrationIsNotComplete

    options = odeset('Events',@eventClutch,'NonNegative',[6 11 12],'reltol',1e-3,...
        'abstol',1e-3,'maxstep',timeStepS,...
        'InitialStep',0.1*abs(tf-t0)); 
    [t,x,te,xe,ie] = ode45(@(t,x) state_derivatives_MB_tractor(x,MT865,input,nConstantMT865,'xdot'),[t0 tf],x0,options);
    
    % Check Criteria that Integration across t0 to tf is complete
    integrationIsNotComplete = (tf ~= t(end));
    
    if integrationIsNotComplete
        x0 = x(end,:).';
        t0 = t(end);
        if ie, MT865.clutchIsSlip = ~MT865.clutchIsSlip; end
        fprintf('Clutch Is Slip %d \n ',MT865.clutchIsSlip)
    else % integrationIsComplete
        MT865.state = x(end,:).';
        % Gather Parameters of interest for Plotting
        [MT865] = state_derivatives_MB_tractor(MT865.state, MT865,input,nConstantMT865,'else');
        % Look Up Terrain Parameters for Next Time Step for Next Time Step
        [MT865] = position_nTrack(MT865,nConstantMT865);
        [MT865] = terrain_parametrs_track(MT865,nConstantTerrain);
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%% END INTEGRATION ROUTINE %%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%% EVENT FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [value,isTerminal,direction] = eventClutch(t,x)

    % Unwrap Necessary Tractor Body and Power-Train Parameters: (T-tractor, s - sprocket)
    nGR = nConstantMT865.nGearRatio;
    FD = nConstantMT865.finalDriveRatio;
    transDamp = nConstantMT865.transmissionDamp;
    TfMaxs = nConstantMT865.clutchStaticFrictionCapNM;
    TfMaxd = nConstantMT865.clutchDynamicFrictionCapNM;
    Jtrans = nConstantMT865.inertiaTransmissionKGM2;
    Jeng = nConstantMT865.inertiaEngineKGM2;
    engDamp = nConstantMT865.engineDamp;


    % Unpack Inputs Needed
    gear = input(2);
    GR = nGR(gear);

    % Unpack States Needed
    wl = x(11);
    wr = x(12);
    engThrtl = x(13);
    engSpdRadPS = x(14);
    engCtrlThrtl = x(16);
    steerAngleState = x(17);
    engTorqNM = engine_interp( engThrtl, engCtrlThrtl, engSpdRadPS, nConstantMT865 );
        engTorq2StrMotorNM = (steerAngleState/170)*engTorqNM;
        engTorq2TransNM = engTorqNM - engTorq2StrMotorNM;
    com = x(11);

    clutchIsSlip = MT865.clutchIsSlip;
    
    % Clutch Logic for Detecting State Change
    Tfs = com*TfMaxs;
    Tfd = com*TfMaxd;

    [Fl,Fr,~,~,~,~,~,~,~,~] = track_forces(x, MT865, nConstantMT865);
    loadTorqueNM = (Fl + Fr)/(GR*FD);
    transOutSpdRadPS = 0.5*FD*abs(wl+wr);
    transInSpdRadPS = transOutSpdRadPS*GR;

        if clutchIsSlip
            relShaftSpdRadPS = engSpdRadPS - transInSpdRadPS;
            torqClutchNM = Tfd*tanh(2*(relShaftSpdRadPS/2));
            value = relShaftSpdRadPS ; % This needs to be zero to detect state change (detection is from slip to stick)
           % fprintf(' value = %d \n' , value)
            isTerminal = 1;
            direction = -1;
        else % Clutch Is Stick
           shaftSpdRadPS = transInSpdRadPS;
           torqClutchNM = (Jtrans*engTorq2TransNM + Jeng*loadTorqueNM - (Jtrans*engDamp - Jeng*transDamp)*shaftSpdRadPS )...
               /(Jtrans + Jeng);
           value = abs(torqClutchNM) - Tfs; % Condition for stick to slip
           isTerminal = 1;
           direction = +1;
        end

    end
%%%%%%%%%%%%%%%%%%%%%%%%% END EVENT FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
end


function [isStuck] = check_if_stuck(stateUpdate)
vx = stateUpdate(4);
r = stateUpdate(6);
w6l = stateUpdate(7);
w6r = stateUpdate(8);

%Calculate vx of each track wrt. the CG of the tracked vehicle.
[ vxLeft,vxRight ] = vx_track_tractor( vx,r);
    
%Calculate Slip along the Length of Each Track (Left Track then Right Track)
slipLeftTrack = slip_ratio( vxLeft,w6l );
slipRightTrack = slip_ratio( vxRight,w6r);

isStuck = (slipLeftTrack >= 0.95)*(slipRightTrack >= 0.95);
end

