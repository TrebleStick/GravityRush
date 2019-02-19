m = matfile('S2_A1_E1.mat'); 
stim = m.stimulus; 
emg = m.emg;

stimReps = 10; 
stimCount = stimReps*max(stim); 
eChannels = 2; 
endColumn = 8; 
nExercises = max(stim); 

stimLength = zeros(1, stimCount); 
stimNumber = 1; 
maxSignalLength = 540; 
signalArr = zeros(maxSignalLength, 2, stimCount);
% signalArr = zeros(maxSignalLength, 1);

sigIndex = 0; 
stimMark = 1; 

% Get length of stimuli 
for n = 2:length(stim)
    if stim(n) > 0
%         lenCount = 1; 
        stimLength(stimNumber) = stimLength(stimNumber) + 1; 
        sigIndex = sigIndex + 1; 
        signalArr(sigIndex, :, stimMark) = emg(n, 1:2);
%         stimLength(stimNumber) = stimLength(stimNumber) + 1; 

    elseif stim(n) == 0
        lenCount = 0;
         if stim(n - 1) ~= stim(n)
             stimNumber = stimNumber + 1; 
             stimMark = stimMark + 1; 
             sigIndex = 0; 
         end
    end
end



