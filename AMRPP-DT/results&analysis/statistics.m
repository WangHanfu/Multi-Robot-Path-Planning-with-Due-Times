function [valueMat,timeMat] = statistics(results)
largeNumber = 10000000;
typeSize = size(results,1);
instanceSize = size(results,2);
valueMat = zeros(typeSize,instanceSize);
timeMat = zeros(typeSize,instanceSize);
for i=1:typeSize
    for j=1:instanceSize
        result = results{i,j};
        vecLen = size(result,1);
        valueVec = zeros(vecLen,1);
        timeVec = zeros(vecLen,1);
        for k=1:vecLen
            temp = result{k,1};
            timeVec(k,1)=temp.ComputeTime;
            if ~isempty(temp.ObjectiveValue)
                valueVec(k,1)=temp.ObjectiveValue;
            else
                valueVec(k,1)=largeNumber;
            end
        end
        valueVec(valueVec==largeNumber)=[];
        if ~isempty(valueVec)
            valueMat(i,j) = min(valueVec);
        else
            valueMat(i,j) = largeNumber;
        end
        timeMat(i,j) = sum(timeVec);
    end
end

end

