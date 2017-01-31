%plan_path
function path = plan_path(start, goal, esti_grid)
addpath('MATLAB/D-Star') % The path to the D-Star folder

ds = Dstar(esti_grid);

ds.plan(goal);
path = ds.path(start);

end