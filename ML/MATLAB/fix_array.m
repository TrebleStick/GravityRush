function [fixed] = fix_array(array)
    n = 1; 
    start_len = numel(array); 
    while n < start_len
        if n > numel(array)
            break; 
        end
        
        if array(n) < 100 && array(n + 1) < 10
            array(n) = (array(n)*10) + array(n + 1); 
            array(n + 1) = []; 
        elseif array(n) < 10 && array(n + 1) < 100
            array(n) = (array(n)*100) + array(n + 1); 
        end
        
        if isnan(array(n))
            array(n) = []; 
        end
        n = n + 1; 
    end
    fixed = array; 
end