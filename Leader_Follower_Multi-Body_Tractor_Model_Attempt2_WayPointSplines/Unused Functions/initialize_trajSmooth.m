function trajSmooth = initialize_trajSmooth(timeArray)

trajSmooth.tI = 0;
trajSmooth.tF = 0;
trajSmooth.yawRateFuncCoeff = zeros(1,4);
trajSmooth.yawRateCmdTraj = zeros(size(timeArray,1),1);

end