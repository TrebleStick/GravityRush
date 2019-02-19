clear all;
sFolder = uigetdir('C:\Documents\Imperial');

folderList = getEnvironment(); 
% Get only the folders containing the string "DB"
folderList = filterEnv(folderList, "DB", 1); 

nChannels = 8; 
nSubjects = numel(folderList); 
maxSignalLength = 540;

% E1 consists of 12 different hand gestures
nExercises = 12; 

stimReps = 10; 
stimCount = stimReps*nExercises; 
signalArr = zeros(stimCount*(nChannels/2)*nSubjects, 2, maxSignalLength);
signalLabel = zeros(stimCount*(nChannels/2)*nSubjects, 1); 

% Used to place signal data in the right 3rd dimension (the layer) 
stimMark = 1; 

for n = 1:nSubjects
    cd(folderList(n)); 
    subFolderList = getEnvironment(); 
    subFolderList = filterEnv(subFolderList, "E1", 0); 
    
    m = matfile(subFolderList); 
    stim = m.stimulus; 
    emg = m.emg;

    sigIndex = 0; 
    s = 1; 
    marked = 0; 
    while s <= length(stim)
        while stim(s) > 0
            if marked == 0
                signalLabel(stimMark:stimMark+3) = stim(s); 
                marked = 1; 
            end
            
            sigIndex = sigIndex + 1; 
            signalArr(stimMark, :, sigIndex)    = emg(s, 1:2);
            signalArr(stimMark+1, :, sigIndex)  = emg(s, 3:4);
            signalArr(stimMark+2, :, sigIndex)  = emg(s, 5:6);
            signalArr(stimMark+3, :, sigIndex)  = emg(s, 7:8);
            
            s = s + 1;  
        end

        if marked == 1
            % stimNumber = stimNumber + 1; 
            sigIndex = 0;
            stimMark = stimMark + 4; 
            marked = 0; 
        end 
        
        s = s + 1; 
    end
    cd ..
end

%% Functions 

function [dirList] = getEnvironment()
    dirList = dir(); 
    dirList = struct2cell(dirList); 
    dirList = dirList(1, :); 
    dirList = string(dirList); 
end

function [filterList] = filterEnv(dirList, pattern, filetype)
    n = 1; 
    while n <= length(dirList)
        if (~contains(dirList(n), pattern)) || (filetype == 1 && ~isfolder(dirList(n)))
            dirList(n) = []; 
            n = n - 1; 
        end 
        n = n + 1; 
    end
    filterList = dirList; 
end

% (find(signalArr(520:540,1,:)~=0))




