%create_objective_list
% 
% FORMAT: objective_cell_array is a 5x3 cell array
%     
% index | objective |   priority    | completed
% -------------------------------------------------
%   n   |  string   | integer [1:n] | integer [0,1]
%

global objective_cell_array

for idxX = 1:5
    for idxY = 1:3
       if (idxY ~= 1)
          objective_cell_array{idxX, idxY} = 0;
       end
       
       if (idxY == 2)
          objective_cell_array{idxX,idxY} = idxX;
       end
    end
    if idxX == 1
        objective_cell_array{1,1} = 'None';
    end
    
    if idxX == 2
        objective_cell_array{2,1} = 'Move to Dig Site';
    end
    
    if idxX == 3
        objective_cell_array{3,1} = 'Dig';
    end
    
    if idxX == 4
        objective_cell_array{4,1} = 'Move to Dump Site';
    end
    
    if idxX == 5
        objective_cell_array{5,1} = 'Dump';
    end

end
    