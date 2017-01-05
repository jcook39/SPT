function plot_trajSmooth(trajSmooth,timeArray)

yawRateCmdTraj = trajSmooth.yawRateCmdTraj;

figure(15)
subplot(321)
    plot(timeArray,yawRateCmdTraj)
    ylabel('Commanded yaw Rate')
end