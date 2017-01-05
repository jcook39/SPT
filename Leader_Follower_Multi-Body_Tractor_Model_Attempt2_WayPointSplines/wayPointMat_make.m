function [ wayPointFlagMatNew ] = wayPointMat_make(wayPointFlagMat,timeScheduleInput, timeStepS )
% This function takes the input Matrix and creates a new input Matrix made
% appropriate for the simulation and timeStep


columnNo = size(wayPointFlagMat,2);
timeEnd = timeScheduleInput(end); 
rowNo = timeEnd/timeStepS + 1;
wayPointFlagMatNew = zeros(rowNo, columnNo+1);
m = numel(timeScheduleInput);

count = 1;
    for i = 1:m-1
        tspan = [timeScheduleInput(i):timeStepS:(timeScheduleInput(i+1)-timeStepS)].';
        length = numel(tspan);
        endRange = count+length-1;
        wayPointFlagMatNew(count:endRange,1) = wayPointFlagMat(i,1);
        wayPointFlagMatNew(count:endRange,2) = wayPointFlagMat(i,2);
        wayPointFlagMatNew(count:endRange,3) = tspan;
        count = count+length;
    end
wayPointFlagMatNew(end,1:2) = wayPointFlagMat(end-1,1:2);
wayPointFlagMatNew(end,3) = timeScheduleInput(end-1)+timeStepS;


end