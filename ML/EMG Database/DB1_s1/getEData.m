m = matfile('S1_A1_E2.mat'); 
stim = m.stimulus; 
emg = m.emg;

% stimLength = zeros(stimCount, endColumn); 

stimReps = 10; 
stimCount = stimReps*max(stim); 
nChannels = 8; 
nSubjects = 1; 
nExercises = max(stim); 

% stimLength = zeros(1, stimCount); 
% stimNumber = 1; 
maxSignalLength = 540; 

signalArr = zeros(maxSignalLength, 2, stimCount*(nChannels/2));
% stimLabel = zeros(stimCount*(nChannels/2)*nSubjects, 1); 

% Used to place signal data in the right 3 dimension (the layer) 
stimMark = 1; 

% for chan = 1:nChannels
    stimNumber = 1; 
    sigIndex = 0; 
    % Get length of stimuli 
    % for n = 2:length(stim)
    n = 1; 
    marked = 0; 
    while n <= length(stim)
        while stim(n) > 0
%             stimLength(stimNumber) = stimLength(stimNumber) + 1; 
            sigIndex = sigIndex + 1; 
            signalArr(sigIndex, :, stimMark) = emg(n, 1:2);
            signalArr(sigIndex, :, stimMark+1) = emg(n, 3:4);
            signalArr(sigIndex, :, stimMark+2) = emg(n, 5:6);
            signalArr(sigIndex, :, stimMark+3) = emg(n, 7:8);
            
            n = n + 1; 
            marked = 1; 
        end
        
        if marked == 1
            stimNumber = stimNumber + 1; 
            sigIndex = 0;
            stimMark = stimMark + 4; 
            marked = 0; 
        end 
        
        n = n + 1; 
        
%         elseif stim(n) == 0
%             lenCount = 0;
%              if stim(n - 1) ~= stim(n)
%                  stimNumber = stimNumber + 1; 
%                  sigIndex = 0;
%                  stimMark = stimMark + 4; 
%         end
    end
% end



