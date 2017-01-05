function simulation_video( nTractor, nTractorColor, nConstantMT865, nConstantTerrain, waypoint, timeStepS, nTimeStep, videoType, plotTerrain, filename)
% This function creates a video to play back simulation results

%%%%%%%%%%%%%%%%%% Evaluate Tractor Structure Size %%%%%%%%%%%%%%%%%%%%%%%%
nTractorName = fieldnames(nTractor);
nTractorColorName = fieldnames(nTractorColor);
nTractorNum = size(nTractorName,1); % Determine number of tractors

%%%%%%%%%%%%%%%%%%%%%%%%% Unpack Terrain %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gridSizeXM = nConstantTerrain.gridSizeXM;
gridSizeYM = nConstantTerrain.gridSizeYM;
contourX = nConstantTerrain.X;
contourY = nConstantTerrain.Y;
frictionAngle = nConstantTerrain.frictionAngle;
cohesion = nConstantTerrain.cohesion;

%%%%%%%%%%%%%%%%%%%%%%%%%%% FRAME RATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timePerFrame = timeStepS; % 1 = 1 second
nTimeStepPerFrame = timePerFrame/timeStepS;

videoTimeArrayIndex = 1:nTimeStepPerFrame:nTimeStep;
videoTimeArray = (videoTimeArrayIndex.*timeStepS) - timeStepS;
videoTimeArraySize = numel(videoTimeArray);

%%%%%%%%%%%%%%%%%%%%%%%% UNPACK WAY POINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nWayPointFollowEvalMatTime = waypoint.nWayPointFollowEvalMatTime;
nWayPointAllTime = waypoint.nWayPointAllTime;


%%%%%%%%%%%%%%%%%%%%%%%%% Simulation Video %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if strcmp(videoType,'video')
    video = VideoWriter(filename);
    %video.FrameRate = 1/timeStepS;
    video.FrameRate = 1/timePerFrame;
    video.Quality = 20;
    open(video);
end


%%%%%%%%%%%%%%%%%%%%%%%% Declare Arrays %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X = zeros(nTractorNum,videoTimeArraySize);
Y = zeros(nTractorNum,videoTimeArraySize);
theta = zeros(nTractorNum,videoTimeArraySize);
phi = zeros(nTractorNum,videoTimeArraySize);
si = zeros(nTractorNum,videoTimeArraySize);
vx = zeros(nTractorNum,videoTimeArraySize);
vy = zeros(nTractorNum,videoTimeArraySize);
dtheta = zeros(nTractorNum,videoTimeArraySize);
dphi = zeros(nTractorNum,videoTimeArraySize);
dsi = zeros(nTractorNum,videoTimeArraySize);
wl = zeros(nTractorNum,videoTimeArraySize);
wr = zeros(nTractorNum,videoTimeArraySize);

engSpeedRadPS = zeros(nTractorNum,videoTimeArraySize);

speed = zeros(nTractorNum,videoTimeArraySize);
slipLeft = zeros(nTractorNum,videoTimeArraySize);
slipRight = zeros(nTractorNum,videoTimeArraySize);

%%%%%%%%%%%%%%%%%%%%%%%% Figure Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fontLabel = 16;
FigHandle = figure;
set(FigHandle, 'Position', [0, 0, 1920, 1200]);
    for i = 1:nTimeStepPerFrame:nTimeStep
        if strcmp(plotTerrain,'yes')
            subplot(221)
            contourf(contourX,contourY,frictionAngle,'edgecolor','none')
            %caxis([0 30])
            %colormap gray
            %colormap bone
            colormap winter
            %colorbar('EastOutside')
            hold on
        end
        
        
        for j = 1:nTractorNum
            % Plot Each Tractor
            tractor = nTractor.(nTractorName{j});
            tractorColor = nTractorColor.(nTractorColorName{j});
            
            % Gather Tractor States
            X(j,i) = tractor(i).state(1);
            Y(j,i) = tractor(i).state(2);
            theta(j,i) = tractor(i).state(3);
            phi(j,i) = tractor(i).state(4);
            si(j,i) = tractor(i).state(5);
            vx(j,i) = tractor(i).state(6);
            vy(j,i) = tractor(i).state(7);
            dtheta(j,i) = tractor(i).state(8);
            dphi(j,i) = tractor(i).state(9);
            dsi(j,i) = tractor(i).state(10);
            wl(j,i) = tractor(i).state(11);
            wr(j,i) = tractor(i).state(12);
            
            engSpeedRadPS(j,i) = tractor(i).state(14);
            
            % Augmented States
            speed(j,i) = sqrt( vx(j,i)^2 + vy(j,i)^2 );       
            slipLeft(j,i) = tractor(i).slip(1);
            slipRight(j,i) = tractor(i).slip(2);
            
            % Plots
            subplot(221)
                plot(nWayPointAllTime(1:i,1), nWayPointAllTime(1:i,2),'wo','linewidth',4)
                hold on
                plot(nWayPointFollowEvalMatTime(i,1),nWayPointFollowEvalMatTime(i,2),'bx','linewidth',2)
                hold on
                plot_tractor_on_map(tractor,i,nConstantMT865,tractorColor)
                %title('Friction Angle of Terrain')
                ylabel('East Position (meters)','interpreter','latex','FontSize',fontLabel)
                xlabel('North Position (meters)','interpreter','latex','FontSize',fontLabel)
                title(['t = ',num2str((i-1)*timeStepS)])
                hold on
                plot( X(j,1:i), Y(j,1:i), tractorColor, 'linewidth', 2)
                hold on
            
            subplot(422)
                plot(videoTimeArray(1:i),speed(j,1:i),tractorColor)
                ylabel('Vehicle Speed (m/s)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                xlim([0 videoTimeArray(end)])
                ylim([0 4])
                hold on
                
            subplot(424)
                plot(videoTimeArray(1:i),dtheta(j,1:i),tractorColor)
                ylabel('Yaw Rate (rad/s)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                xlim([0 videoTimeArray(end)])
                ylim([-0.5 0.5])
                hold on
                
            subplot(426)
                plot(videoTimeArray(1:i),slipLeft(j,1:i),tractorColor)
                ylabel('Slip Ratio Left','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                xlim([0 videoTimeArray(end)])
                ylim([0 10])
                hold on
                
            subplot(428)
                plot(videoTimeArray(1:i),slipRight(j,1:i),tractorColor)
                ylabel('Slip Ratio Right','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                xlim([0 videoTimeArray(end)])
                ylim([0 10])
                hold on
                
            subplot(425)
                plot(videoTimeArray(1:i),engSpeedRadPS(j,1:i).*(60/(2*pi)),tractorColor)
                ylabel('Engine Speed (RPM)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                xlim([0 videoTimeArray(end)])
                ylim([1000 2300])
                hold on
                
            subplot(427)
                plot(videoTimeArray(1:i),phi(j,1:i).*(180/pi),tractorColor)
                ylabel('DrawBar Angle (Degrees)','interpreter','latex','FontSize',fontLabel)
                xlabel('time (seconds)','interpreter','latex','FontSize',fontLabel)
                xlim([0 videoTimeArray(end)])
                ylim([-10 10])
                hold on
     
        end % END FOR nTractorNum
        
        subplot(221)
            xlim([min(X(2,i))-50 max(X(2,i))+75])
            ylim([min(Y(2,i))-50 max(Y(2,i))+50])
            hold off
            
        if strcmp(videoType,'video')
            movieVar(i) = getframe(gcf);
            writeVideo(video,movieVar(i));
        elseif strcmp(videoType,'matlab')
            pause(0.01)
        end
        hold off
    end % END FOR Figure Loop for each time step
    
    if strcmp(videoType,'video')
        close(video);
    end
end

