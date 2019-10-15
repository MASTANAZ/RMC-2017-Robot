function priority = P(o_i, ET)

    % Create a list of tasks for the objective
    T = o_i{1};
    
    % Here, o_n is the number of tasks in the objective
    [o_m, o_n]= size(T);
    [et_m, et_n] = size(ET);
    
    % Transpose so we can sum the row of times
    ETt = ET.';
    
    % only use the second index of avgET
    avgET = sum(ETt,2) / et_m;
    
    pre_square = 0.0;
    for i = 1:1:o_n
        pre_square = pre_square + (ET(T(i),2) - avgET(2));
    end
    
    square = pre_square^2;
    
    pre_root = square/o_n;
    
    priority = sqrt(pre_root);
end
