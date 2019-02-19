m = matfile('arr.mat'); 
emg = m.arr; 
stim = m.windows; 


% Used to place signal data in the right 3 dimension (the layer) 
stimMark = 1; 

maxStimNum = 120; 
windowLength = 50; 
signalArr = zeros(maxStimNum, 1, windowLength); 
signalLabel = zeros(maxStimNum*2, 1); 

stimNumber = 1; 
sigIndex = 0; 
n = 1; 
marked = 0; 
while n < length(stim)-50
    tempArr = emg(n:n+49); 
    for k = 1:numel(tempArr)
        if tempArr(k) > 260
            signalLabel(stimMark) = 1; 
            signalArr(stimMark, 1, :) = tempArr; 
            break;
        end
        if k == 49
            signalLabel(stimMark) = 0; 
            signalArr(stimMark, 1, :) = tempArr; 
            break;
        end
    end
    stimMark = stimMark + 1;
    n = n + 50; 
            
%     while stim(n) > 0 && n < length(stim)
%         if marked == 0
%             sigIndex = 0;
%             % There is an action occurring
%             signalLabel(stimMark) = 1; 
%             stimMark = stimMark + 1;
%             marked = 1; 
%         end
%             
%         sigIndex = sigIndex + 1; 
%         signalArr(stimMark, :, sigIndex) = emg(n);
%         n = n + 1;  
%     end
%     
%     while stim(n) == 0 && n < length(stim)
%         if marked == 1
%             stimNumber = stimNumber + 1; 
%             sigIndex = 0;
%             signalLabel(stimMark) = 0; 
%             stimMark = stimMark + 1; 
%             marked = 0; 
%         end
%         sigIndex = sigIndex + 1; 
%         signalArr(stimMark, :, sigIndex) = emg(n);
%         n = n + 1;
%     end 
end
% 
% marked = 0; 
% for n = 2:length(stim)
%     if stim(n) == 0
%         if n < 100
%             marked = 1; 
%         elseif stim(n) ~= stim(n - 1)
%             
%         end
%     elseif stim(n) == 1
%         
%     end
%     
% end
% 
% 
% 
% 
