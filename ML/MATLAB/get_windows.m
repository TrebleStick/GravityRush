function [windows] = get_windows(array)
    start_len = numel(array); 
%     windowLength = 30; 
%     stimMark = 1; 
%     maxNumStim = 100; 
%     windows = zeros(maxNumStim, 1, windowLength); 
%     
%     while n <= start_len - 14
%         if (n > 15) && (array(n)> 250) && (array(n) > array(n + 1)) && (array(n) > array(n + 2)) && (array(n) > array(n + 3)) && (array(n) > array(n + 4))
%             windows(stimMark, 1, :) = array(n - 15:n + 14); 
%             stimMark = stimMark + 1; 
%             n = n + 14; 
%         else 
%             n = n + 1; 
%         end
%     end

    stimWindow = zeros(start_len, 1); 
    
    for n = 51:(start_len-49)
        if mod(n, 100) == 0
            stimWindow((n-25):(n+24)) = ones(50, 1); 
        end
    end
    windows = stimWindow; 
end