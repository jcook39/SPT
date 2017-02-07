function plot_terrain_hypothesis_2(structRBE, structDTKF, nConstantMT865, nConstantTerrain, nTimeParam, tractorColor, hypothesisString, figureNo)
% This function plots all terrain hypothesis for the Recursive Bayes
% Estimator

% --------------------- Color Declarations --------------------------------
trueColor = 'b.';
estimateColor = tractorColor;

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
nTimeStep = nTimeParam.nTimeStep;
timeStepS = nTimeParam.timeStepS;
time = nTimeParam.time;

indexVector = 1:numel(time);
timeVector = time;

% ----------------------- Unpack DTKF structure ---------------------------
x = structDTKF.x;
xHatPlus = structDTKF.xHatPlus;
xError = structDTKF.xError;
y = structDTKF.y;

slipHat = structDTKF.slipHat;
slipHatSmooth = structDTKF.slipHatSmooth;

slipVectorEstimate = 100*(1-(xHatPlus(1,indexVector)./(rollingRadiusM*xHatPlus(2,indexVector))));
slipVectorTrue = 100*(1-(x(1,indexVector)./(rollingRadiusM*x(2,indexVector))));
slipVectorMeasure = 100*(1-(y(2,indexVector)./(rollingRadiusM*y(3,indexVector))));

% =========================================================================
slip = 0.01:1.5:100;

nHypothesis = size(terrainHypotheses,2);
for i = 1:nHypothesis
    [F_TNetMat(:,i),F(:,i), R(:,i), tauRes(:,i), sinkage(:,i)] = net_track_force(terrainHypotheses(:,i),nConstantMT865,slip);
                 
end % end for

%     figure(figureNo)
%     subplot(231)
%         plot( slip, F_TNetMat, slip, zeros(numel(slip),1),'k:' )
%         ylabel('Net Traction (N)')
%         xlabel('slip ratio')
%         xlim([0 100])
%         legend('Hyp1','Hyp2','Hyp3','Hyp4','Hyp5','Location','SouthEast')
%         hold on
%     subplot(232)
%         plot( slip, F )
%         ylabel('Gross Traction (N)')
%         xlabel('slip ratio')
%         xlim([0 100])
%         hold on
%     subplot(233)
%         plot( slip, tauRes(:,1) )
%         ylabel('Resistance Torque (N)')
%         xlabel('slip ratio')
%         xlim([0 100])
%         hold on
%     subplot(234)
%         plot( slip, R)
%         ylabel('Resistance (N)')
%         xlabel('slip ratio')
%         hold on
%     subplot(235)
%         plot(slip, sinkage, slip, maxSinkageIn*inch2meter*ones(numel(slip),1),'k:')
%         xlabel('slip ratio')
%         ylabel('sinkage (m)')
%         hold on
%     subplot(236)
%         plot(slip, sinkage(:,1)*meter2inch, slip, maxSinkageIn*ones(numel(slip),1),'k:')
%         xlabel('slip ratio')
%         ylabel('sinkage (in)')
%         hold on

if strcmp(hypothesisString, 'plotHypothesis')
fontLabel = 16;        
    figure(figureNo+1)
    subplot(121)
        plot( slip, F_TNetMat./1000)
        ylabel('Net Traction $F_{net}$ (kN)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on
    subplot(122)
        plot( slip, tauRes./1000 )
        ylabel('Resistance Torque $\tau_{res}$ (kN)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on
end

lineWidthSize = 2;
fontLabel = 16;        
    figure(figureNo+2)
    subplot(121)
        if strcmp(hypothesisString, 'plotHypothesis')
            plot(slip, F_TNetMat./1000);
            hold on
        end
        h = plot(slipHatSmooth, xHatPlus(3,indexVector)./1000, estimateColor);% slip, F_TNetMat);
        set(h(1),'linewidth',lineWidthSize);
        ylabel('Net Traction $F_{net}$ (kN)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on
        
    subplot(122)
        if strcmp(hypothesisString, 'plotHypothesis')
            plot(slip, tauRes./1000);
            hold on
        end
        h = plot(slipHatSmooth, xHatPlus(5,indexVector)./1000, estimateColor );
        set(h(1),'linewidth',lineWidthSize);
        ylabel('Resistance Torque $\tau_{res}$ (kNm)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on
        
end