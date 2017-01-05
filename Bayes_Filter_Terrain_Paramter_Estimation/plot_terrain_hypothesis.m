function plot_terrain_hypothesis(structRBE, structDTKF, nConstantMT865, nConstantTerrain, time, plotString, figureNo)
% This function plots all terrain hypothesis for the Recursive Bayes
% Estimator

% --------------------- Color Declarations --------------------------------
trueColor = 'b.';
estimateColor = 'g.';

% ---------------------- Unpack Structures --------------------------------
terrainHypotheses = structRBE.terrainHypotheses;
terrainActual = nConstantTerrain.terrainActual;
rollingRadiusM = nConstantMT865.rollingRadiusM;


% ------------------ Declare Dimensional Parameters -----------------------
meter2inch = 39.3701;
inch2meter = 0.0254;
grouserLengthIn = 5.5;
frontSprocketRadiusM = 0.602;
maxSinkageM = frontSprocketRadiusM + grouserLengthIn*inch2meter;
maxSinkageIn = maxSinkageM*meter2inch;
maxSinkageRangeM = 1.2;


% ===================== Unpack DTKF Struct ================================
%
% --------------------- Estimated State Vector ----------------------------
% xHat = [vHat omegaHat F_T,NetHat F_T,NetHatDot tau_ResHat tau_ResHatDot 
%           R_SHat R_SHatDot].';
%
% --------------------- True State Vector ----------------------------
% x = [v omega F_T,Net F_T,NetDot tau_Res tau_ResDot R_S R_SDot].';
%
% -------------------- Measurement Vector ---------------------------------
% y = [vDot v omega R_S].'


% ---------------- Create time Vector for plotting ------------------------
indexVector = 1:numel(time);
timeVector = time;

% ----------------------- Unpack DTKF structure ---------------------------
x = structDTKF.x;
xHatPlus = structDTKF.xHatPlus;
xError = structDTKF.xError;
y = structDTKF.y;

slipVectorEstimate = 100*(1-(xHatPlus(1,indexVector)./(rollingRadiusM*xHatPlus(2,indexVector))));
slipVectorTrue = 100*(1-(x(1,indexVector)./(rollingRadiusM*x(2,indexVector))));
slipVectorMeasure = 100*(1-(y(2,indexVector)./(rollingRadiusM*y(3,indexVector))));

% =========================================================================


slip = 0.01:1:100;
nHypothesis = size(terrainHypotheses,2);
for i = 1:nHypothesis
    [F_TNetMat(:,i),F(:,i), R(:,i), tauRes(:,i), sinkage(:,i)] = net_track_force(terrainHypotheses(:,i),nConstantMT865,slip);
end

nActualTerrain = size(terrainActual,2);
for i = 1:nActualTerrain
    [F_TNetMatAct(:,i),FAct(:,i), RAct(:,i), tauResAct(:,i), sinkageAct(:,i)] = net_track_force(terrainActual(:,i),nConstantMT865,slip);
end


if strcmp(plotString,'Hypothesis')
    
figure(figureNo)
subplot(231)
    plot( slip, F_TNetMat(:,1), slip, F_TNetMat(:,2), slip, F_TNetMat(:,3), slip, F_TNetMat(:,4), slip, F_TNetMat(:,5), slip, zeros(numel(slip),1),'k:' )
    ylabel('Net Traction (N)')
    xlabel('slip ratio')
    xlim([0 100])
    legend('Hyp1','Hyp2','Hyp3','Hyp4','Hyp5','Location','SouthEast')
    hold on
subplot(232)
    plot( slip, F(:,1), slip, F(:,2), slip, F(:,3), slip, F(:,4), slip, F(:,5) )
    ylabel('Gross Traction (N)')
    xlabel('slip ratio')
    xlim([0 100])
    hold on
subplot(233)
    plot( slip, tauRes(:,1), slip, tauRes(:,2), slip, tauRes(:,3), slip, tauRes(:,4), slip, tauRes(:,5) )
    ylabel('Resistance Torque (N)')
    xlabel('slip ratio')
    xlim([0 100])
    hold on
subplot(234)
    plot( slip, R(:,1), slip, R(:,2), slip, R(:,3), slip, R(:,4), slip, R(:,5) )
    ylabel('Resistance (N)')
    xlabel('slip ratio')
    hold on
subplot(235)
    plot(slip, sinkage(:,1), slip, sinkage(:,2), slip, sinkage(:,3), slip, sinkage(:,4), slip, sinkage(:,5),... 
        slip, maxSinkageIn*inch2meter*ones(numel(slip),1),'k:',slip, frontSprocketRadiusM*ones(numel(slip),1),'k')
    xlabel('slip ratio')
    ylabel('sinkage (m)')
    hold on
subplot(236)
    plot(slip, sinkage(:,1)*meter2inch, slip, sinkage(:,2)*meter2inch, slip, sinkage(:,3)*meter2inch, slip, sinkage(:,4)*meter2inch, slip, sinkage(:,5)*meter2inch, slip, maxSinkageIn*ones(numel(slip),1),'k:')
    xlabel('slip ratio')
    ylabel('sinkage (in)')
    hold on
    
    
    
% =========================================================================    
elseif strcmp(plotString,'All')

figure(figureNo)
subplot(231)
    plot( slip, F_TNetMat(:,1), slip, F_TNetMat(:,2), slip, F_TNetMat(:,3), slip, F_TNetMat(:,4), slip, F_TNetMat(:,5), slip, zeros(numel(slip),1),'k:'...
        ,slip, F_TNetMatAct(:,1),'--', slip, F_TNetMatAct(:,2),'--', slip, F_TNetMatAct(:,3),'--', slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
    ylabel('Net Traction (N)')
    xlabel('slip ratio')
    xlim([0 100])
    legend('Hyp1','Hyp2','Hyp3','Hyp4','Hyp5','Zero Mark','Act1','Act2','Act3','True','Estimated','Location','SouthEast')
    hold on
subplot(232)
    plot( slip, F(:,1), slip, F(:,2), slip, F(:,3), slip, F(:,4), slip, F(:,5) )
    ylabel('Gross Traction (N)')
    xlabel('slip ratio')
    hold on
subplot(233)
    plot( slip, tauRes(:,1), slip, tauRes(:,2), slip, tauRes(:,3), slip, tauRes(:,4), slip, tauRes(:,5)...
        , slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
    ylabel('Resistance Torque (N)')
    xlabel('slip ratio')
    xlim([0 100])
    hold on
subplot(234)
    plot( slip, R(:,1), slip, R(:,2), slip, R(:,3), slip, R(:,4), slip, R(:,5) )
    ylabel('Resistance (N)')
    xlabel('slip ratio')
    hold on
subplot(235)
    plot(slip, sinkage(:,1), slip, sinkage(:,2), slip, sinkage(:,3), slip, sinkage(:,4), slip, sinkage(:,5),... 
        slip, maxSinkageIn*inch2meter*ones(numel(slip),1),'k:',slip, frontSprocketRadiusM*ones(numel(slip),1),'k')
    xlabel('slip ratio')
    ylabel('sinkage (m)')
    hold on
subplot(236)
    plot(slip, sinkage(:,1)*meter2inch, slip, sinkage(:,2)*meter2inch, slip, sinkage(:,3)*meter2inch, slip, sinkage(:,4)*meter2inch, slip, sinkage(:,5)*meter2inch, slip, maxSinkageIn*ones(numel(slip),1),'k:')
    xlabel('slip ratio')
    ylabel('sinkage (in)')
    hold on  
    
end

% =========================================================================
end