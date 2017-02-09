function [MT865] = motion_tractor(MT865,input,nConstantMT865,nConstantTerrain,timeStepS)

%% Unpack Parameters

% --------------------------- Unpack nConstantMT865 -----------------------
winchCableMaxM = nConstantMT865.winchCableMaxM;
winchRadiusM = nConstantMT865.winchRadiusM;


%% Integration Routine 
integrationIsNotComplete = 1;
t0 = 0;
tf = timeStepS;
x0 = MT865.state;

while integrationIsNotComplete
    options = odeset('Events',@Events,'NonNegative',[4 7 8 15],'reltol',1e-3,...
        'abstol',1e-3,'maxstep',timeStepS,'InitialStep',0.1*abs(tf-t0)); 
    [t,x,te,xe,ie] = ode45(@(t,x) f_of_x5_tractor(x,MT865,input,nConstantMT865,'xdot'),[t0 tf],x0,options);
    
    % Check Criteria that Integration across t0 to tf is complete
    integrationIsNotComplete = (tf ~= t(end));
    
    if integrationIsNotComplete
        x0 = x(end,:).';
        t0 = t(end);
        clutchEventTrigger = sum( ie == 1);
        winchEventTrigger = (0 < sum((ie == 2) + (ie == 3) + (ie == 4)) );
        if clutchEventTrigger 
            MT865.clutchIsSlip = ~MT865.clutchIsSlip; 
            fprintf('Clutch Is Slip %d \n ',MT865.clutchIsSlip), 
        end % end if
        if winchEventTrigger
            MT865.winchIsLocked = ~MT865.winchIsLocked;
            fprintf('Winch is Locked %d \n',MT865.winchIsLocked)
            if MT865.winchIsLocked
                cablePulledInToMinimumLengthM = sum(ie == 2);
                brakingHasStoppedWinch = sum(ie == 3);
                winchCableHasReachedMaximumLengthM = sum(ie == 4);
                if or(cablePulledInToMinimumLengthM, brakingHasStoppedWinch)
                    x0(16) = 0; % Speed of Winch Set to Zero Once locked
                    x0(14) = x0(4);
                elseif winchCableHasReachedMaximumLengthM
                    x0(16) = 0;
                    x0(14) = 0.1;
                    x0(4) = x0(14);
                end
            elseif ~MT865.winchIsLocked
                % These events do not require resetting the values of any
                % states
                winchHasBrokenLooseFromBraking = sum(ie == 2);
                winchIsReelingInPayLoad = sum(ie == 3);
            end          
            %MT865.winchIsLocked = ~MT865.winchIsLocked;
            %fprintf('Winch is Locked %d \n',MT865.winchIsLocked)
            %if MT865.winchIsLocked
            %    winchCableIsAtMaxLengthM = (winchCableMaxM <= winchRadiusM*x0(15));
            %    x0(16) = 0; % Speed of Winch Set to Zero Once locked
            %    if ~winchCableIsAtMaxLengthM
            %        x0(14) = x0(4); % Speed of Sled Equals Speed of Tractor
            %    elseif winchCableIsAtMaxLengthM
            %        x0(14) = 0.1;
            %        x0(4) = x0(14); 
            %    end
            %end
        end % end if     
    else % integrationIsComplete
        MT865.state = x(end,:).';
        % Gather Parameters of interest for Plotting
        [MT865] = f_of_x5_tractor(MT865.state, MT865,input,nConstantMT865,'else');
        % Look Up Terrain Parameters for Next Time Step for Next Time Step
        [MT865] = position_nTrack(MT865,nConstantMT865);
        [MT865] = terrain_parametrs_track(MT865,nConstantTerrain);
    end % end if

end % end While

%% Events Function
    function [value,isTerminal,direction] = Events(t,x)
    %% Unwrap Necessary Tractor Body and Power-Train Parameters: (T-tractor, s - sprocket)
    mS = nConstantMT865.massSledKG;
    mT = nConstantMT865.massTractorKG;
    RsledN = nConstantMT865.RsledN;
    nGR = nConstantMT865.nGearRatio;
    FD = nConstantMT865.finalDriveRatio;
    transDamp = nConstantMT865.transmissionDamp;
    TfMaxs = nConstantMT865.clutchStaticFrictionCapNM;
    TfMaxd = nConstantMT865.clutchDynamicFrictionCapNM;
    Jtrans = nConstantMT865.inertiaTransmissionKGM2;
    Jeng = nConstantMT865.inertiaEngineKGM2;
    engDamp = nConstantMT865.engineDamp;
    
    rw = nConstantMT865.winchRadiusM;
    winchCableMaxM = nConstantMT865.winchCableMaxM;
    Dm = nConstantMT865.displacementMotorM3;
    GRm = nConstantMT865.gearRatioMotor;

    %% Unpack Inputs Needed
    gear = input(2);
    steerAngleDeg = input(3);
    valvePos = input(5);
    GR = nGR(gear);

    %% Unpack States Needed
    wl = x(7);
    wr = x(8);
    engTorqStateNM = x(9);
        engTorq2StrMotorNM = (steerAngleDeg/170)*engTorqStateNM;
        engTorq2TransNM = engTorqStateNM - engTorq2StrMotorNM;
    engSpdStateRadPS = x(10);
    com = x(11);
    psiWinchRad = x(15);
    psiWinchRadPS = x(16);
    hydPrO = x(18);

    clutchIsSlip = MT865.clutchIsSlip;
    winchIsLocked = MT865.winchIsLocked;
    %% Clutch Logic for Detecting State Change
    Tfs = com*TfMaxs;
    Tfd = com*TfMaxd;

    [Fl,Fr,RL,RR,~,~,~,~,~,~] = track_forces(x, MT865, nConstantMT865);
    loadTorqueNM = (Fl + Fr)/(GR*FD);
    transOutSpdRadPS = 0.5*FD*abs(wl+wr);
    transInSpdRadPS = transOutSpdRadPS*GR;

        if clutchIsSlip
            relShaftSpdRadPS = engSpdStateRadPS - transInSpdRadPS + 0.01;
            torqClutchNM = Tfd*tanh(2*(relShaftSpdRadPS/2));
            value(1) = relShaftSpdRadPS ; % This needs to be zero to detect state change (detection is from slip to stick)
            isTerminal(1) = 1;
            direction(1) = -1;
        else % Clutch Is Stick
           shaftSpdRadPS = transInSpdRadPS;
           torqClutchNM = (Jtrans*engTorq2TransNM + Jeng*loadTorqueNM - (Jtrans*engDamp - Jeng*transDamp)*shaftSpdRadPS )...
               /(Jtrans + Jeng);
           value(1) = abs(torqClutchNM) - Tfs; % Condition for stick to slip
           isTerminal(1) = 1;
           direction(1) = +1;
        end % end if
    
    %% Winch Logic for Detecting Locked Winch
        if ~winchIsLocked
                % Cable Pulled In All the way
                value(2) = psiWinchRad; 
                isTerminal(2) = 1;
                direction(2) = -1;
                % Brake has stopped winch
                value(3) = psiWinchRadPS;
                isTerminal(3) = 1;
                direction(3) = 0;
                % Cable ahs reach maximum length
                value(4) = psiWinchRad*rw - winchCableMaxM;
                isTerminal(4) = 1;
                direction(4) = 1;
        else % winchIsLocked
                M = [mT 0   1;
                     0  mS -1;
                    -1  1  0];
                F(1,1) = Fl + Fr - RL - RR;
                F(2,1) = -RsledN;
                F(3,1) = 0;
                a = M\F; DBP = a(3);
                torqWinchNM = Dm*hydPrO*GRm;
                % Insufficient torque to keep winch locked
                value(2) = -torqWinchNM + DBP*rw;
                isTerminal(2) = 1;
                direction(2) = 1;
                % If enough torque to reel in winch from max length
                winchCableIsAtMaximumLength = (psiWinchRad*rw == winchCableMaxM);
                if winchCableIsAtMaximumLength
                    value(3) = -torqWinchNM + DBP*rw;
                    isTerminal(3) = 1;
                    direction(3) = -1;
                end
        end % end if
        
    end % End Events Function

            
end % End motion_tractor



