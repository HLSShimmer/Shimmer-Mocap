function motionMasked = ExtractMotion(motionSeries,para)
%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  提取有效动作  %%%%
%%%%%%%%%%%%%%%%%%%%%%%%
% motionSeries             input         动作
% para                     input         一些参数
% motionMasked             output        提取后的动作信息

%% declare some values
motionModule = sqrt(sum(motionSeries.^2,2));
dataNum = size(motionSeries,1);
motionDetect = zeros(dataNum,1);
variance = zeros(dataNum,1);
motionBeginIndex = [];
motionEndIndex = [];
%% get motionDetect based on variance
for i=para.populationSize:dataNum
    variance(i) = var(motionModule(i-para.populationSize+1:i));
end
motionDetect(variance>para.lamdaV) = 1;
%% determine each motion, calculate the range of each motion
count = 1;
while count<=dataNum
    if motionDetect(count)==1
        isMotion = true;
         motionBeginIndex = [motionBeginIndex;count];
        endIndex = count;
    else
        count = count + 1;
        continue;
    end
    while isMotion
        updated = endIndex;
        for j=1:para.lamdaM
            if endIndex+j>dataNum
                isMotion = false;
                break;
            elseif motionDetect(endIndex + j)==1
                updated = endIndex + j;
            end
        end
        if ~isMotion
            motionEndIndex = [motionEndIndex;updated];
            count = dataNum;
            break;
        end
        if updated>endIndex
            isMotion = true;
            endIndex = updated;
        else
            isMotion = false;
            motionEndIndex = [motionEndIndex;updated];
            count = updated;
        end
    end
    count = count + 1;
end
%% generate a mask based on the range of each motion
motionMasked = zeros(size(motionSeries));
for i=1:length(motionBeginIndex)
    motionMasked(motionBeginIndex(i):motionEndIndex(i),:) = motionSeries(motionBeginIndex(i):motionEndIndex(i),:);
end