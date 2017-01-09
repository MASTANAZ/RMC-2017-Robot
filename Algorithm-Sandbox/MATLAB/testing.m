  map = zeros(24,12)
addpath('D-Star')    % Adding path directory for all d-star file

load map1;
goal = [6,24];
start=[1,1];
ds = Dstar(map);    % create navigation object
ds.plan(goal)       % create plan for specified goal
ds.plot(goal)
%        ds.path(start)      % animate path from this start location