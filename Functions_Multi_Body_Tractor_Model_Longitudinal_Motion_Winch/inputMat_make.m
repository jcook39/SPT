function [ inputMatNew ] = inputMat_make(inputMat, timeStepS )
% This function takes the input Matrix and creates a new input Matrix made
% appropriate for the simulation and timeStep

%inputMat = [throttleTractorOne gearTractorOne steerAngleDegTractorOne clutchCmdOne timeScheduleInput];

timeScheduleInput = inputMat(:,end);
[m, n] = size(inputMat);
timeEnd = inputMat(m,n);
rows = timeEnd/timeStepS + 1;
inputMatNew = zeros(round(rows),n);

count = 1;
    for i = 1:m-1
        tspan = [timeScheduleInput(i):timeStepS:(timeScheduleInput(i+1)-timeStepS)].';
        length = numel(tspan);
        endRange = count+length-1;
        inputMatNew(count:endRange,1) = inputMat(i,1);
        inputMatNew(count:endRange,2) = inputMat(i,2);
        inputMatNew(count:endRange,3) = inputMat(i,3);
        inputMatNew(count:endRange,4) = inputMat(i,4);
        inputMatNew(count:endRange,5) = inputMat(i,5);
        inputMatNew(count:endRange,6) = inputMat(i,6);
        inputMatNew(count:endRange,7) = tspan;
        count = count+length;
    end
inputMatNew(end,1:end-1) = inputMat(end-1,1:end-1);
inputMatNew(end,end) = inputMat(end,end);


% for j = 1:nTimeStep
%     tractor(j).throttle = inputMatNew(j,1);
%     tractor(j).gear = inputMatNew(j,2);
%     tractor(j).steerAngleDeg = inputMatNew(j,3);
%     tractor(j).clutchCmd = inputMatNew(j,4);
% end

end

