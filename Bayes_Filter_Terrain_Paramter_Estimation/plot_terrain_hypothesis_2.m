function plot_terrain_hypothesis_2(structRBE, structDTKF, nConstantMT865, nConstantTerrain, time, plotString, figureNo)
% This function plots all terrain hypothesis for the Recursive Bayes
% Estimator

% --------------------- Color Declarations --------------------------------
trueColor = 'b.';
estimateColor = 'r.';

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
slip = 0.01:1.5:100;

if strcmp(plotString, 'Hypotheses')
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

fontLabel = 16;        
    figure(figureNo)
    subplot(121)
        plot( slip, F_TNetMat)
        ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on
    subplot(122)
        plot( slip, tauRes )
        ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on

lineWidthSize = 2;
fontLabel = 16;        
    figure(figureNo+5)
    subplot(121)
        plot(slip, F_TNetMat);
        hold on
        h = plot(slipVectorEstimate, xHatPlus(3,indexVector), estimateColor);% slip, F_TNetMat);
        set(h(1),'linewidth',lineWidthSize);
        ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        xlim([0 100])
        hold on
        
    subplot(122)
        h = plot(slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slip, tauRes );
        set(h(1),'linewidth',lineWidthSize);
        ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
        xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        legend('Estimate')
        xlim([0 100])
        hold on
%     subplot(121)
%         plot(slipVectorEstimate, xHatPlus(3,indexVector), estimateColor)
%         ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         xlim([-10 100])
%         ylim([0 15E4])
%         legend('Estimate')
%         hold on
%     subplot(122)
%         plot(slipVectorEstimate, xHatPlus(5,indexVector), estimateColor)
%         ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
%         xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
%         xlim([-10 100])
%         ylim([0 15E4])
%         legend('Estimated')
%         hold on        
        
        
end % end if




if strcmp(plotString, 'Actual')
nActualTerrain = size(terrainActual,2);
for i = 1:nActualTerrain
    [F_TNetMatAct(:,i),FAct(:,i), RAct(:,i), tauResAct(:,i), sinkageAct(:,i)] = net_track_force(terrainActual(:,i),nConstantMT865,slip);
    [~, ~, netTractionLoadMatTrue(:,i), ~] = peak_traction(nConstantMT865, terrainActual(:,i), slip, 'AllTraction');
end % end for

    figure(figureNo)
    subplot(231)
        plot( slip, zeros(numel(slip),1),'k:',slip, F_TNetMatAct,...
            slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
        ylabel('Net Traction (N)')
        xlabel('slip ratio')
        xlim([0 100])
        legend('Zero Mark','Terrain1','Terrain2','Terrain3','Terrain4','Terrain5','Terrain6',...
            'Terrain7','Terrain8','Terrain9','Terrain10','Terrain11','Terrain12',...
            'Terrain13','Terrain14','Terrain15','Estimate','True','Location','SouthEast')
        hold on
    subplot(232)
        plot( slip, FAct)
        legend('Act1','Act2','Act3')
        ylabel('Gross Traction (N)')
        xlabel('slip ratio')
        hold on
    subplot(233)
        plot(slip, tauResAct,slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
        ylabel('Resistance Torque (N)')
        xlabel('slip ratio')
        xlim([0 100])
        hold on
    subplot(234)
        plot( slip, RAct )
        ylabel('Resistance (N)')
        xlabel('slip ratio')
        hold on
    subplot(235)
        plot(slip, sinkageAct,...
            slip, maxSinkageIn*inch2meter*ones(numel(slip),1),'k:')
        xlabel('slip ratio')
        ylabel('sinkage (m)')
        hold on
    subplot(236)
        plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadMatTrue)
        ylabel('Net Traction with PayLoad (N)')
        xlabel('slip ratio')        
%     subplot(236)
%         plot(slip, sinkageAct(:,1)*meter2inch, slip, sinkageAct(:,2)*meter2inch, slip, sinkageAct(:,3)*meter2inch,...
%              slip, maxSinkageIn*ones(numel(slip),1),'k:')
%         xlabel('slip ratio')
%         ylabel('sinkage (in)')
%         hold on  

end % end if

% =========================================================================


fontLabel = 16;
nActualTerrain = size(terrainActual,2);
for i = 1:nActualTerrain
    [F_TNetMatAct(:,i),FAct(:,i), RAct(:,i), tauResAct(:,i), sinkageAct(:,i)] = net_track_force(terrainActual(:,i),nConstantMT865,slip);
    [~, ~, netTractionLoadMatTrue(:,i), ~] = peak_traction(nConstantMT865, terrainActual(:,i), slip, 'AllTraction');
end % end for

figure(figureNo+2)
subplot(131)
    plot( slip, F_TNetMatAct(:,1), slip, F_TNetMatAct(:,2),...
        slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
    ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
    xlim([-10 100])
    ylim([0 15E4])
    legend('Terrain 1','Terrain 2','Estimate','True','Location','SouthEast')
    hold on

subplot(132)
    plot(slip, tauResAct(:,1), slip, tauResAct(:,2),...
        slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
    ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
    xlim([-10 100])
    ylim([0 15E4])
    legend('Terrain 1','Terrain 2','Estimated','True','Location','SouthEast')
    hold on

subplot(133)
    plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadMatTrue(:,1), slip, netTractionLoadMatTrue(:,2))
    ylabel('Net Traction with Payload (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
        

figure(figureNo+3)
subplot(121)
    plot( slip, F_TNetMatAct(:,1), slip, F_TNetMatAct(:,2),...
        slipVectorEstimate, xHatPlus(3,indexVector), estimateColor, slipVectorTrue, x(3,indexVector),trueColor)
    ylabel('Net Traction no Payload (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
    xlim([-10 100])
    ylim([0 15E4])
    legend('Terrain 1','Terrain 2','Estimate','True','Location','SouthEast')
    hold on

subplot(122)
    plot(slip, tauResAct(:,1), slip, tauResAct(:,2),...
        slipVectorEstimate, xHatPlus(5,indexVector), estimateColor, slipVectorTrue, x(5,indexVector),trueColor)
    ylabel('Resistance Torque (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
    xlim([-10 100])
    ylim([0 15E4])
    legend('Terrain 1','Terrain 2','Estimated','True','Location','SouthEast')
    hold on


figure(figureNo+4)
plot(slip, zeros(numel(slip),1),'k:',slip, netTractionLoadMatTrue(:,1), slip, netTractionLoadMatTrue(:,2))
    ylabel('Net Traction with Payload (N)','interpreter','latex','fontsize',fontLabel)
    xlabel('slip ratio (\%)','interpreter','latex','fontsize',fontLabel)
    legend('zero mark','Terrain 1','Terrain 2')

end