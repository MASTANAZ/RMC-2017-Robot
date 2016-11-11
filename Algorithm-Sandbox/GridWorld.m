% GridWorld.m
clear all;

% Create the set of actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%
create_actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Some simulaton parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gridDims = [12 4 16 4 4 2]; % number of rows, number of columns in the three regions, number of rows and columns of unload station
pObstacle = 0.1; % per-cell probability of obstacle being present (in region 2)
pStartTop = 0.5; % probability of rover starting in top-left (vs bottom-left) corner
pTransSpd = 0.01; % rover's translational speed (assuming flat terrain, in units of world width per decision stage)
pAngleSpd = 5.0; % rover's angular speed (in units of degrees per decision stage)
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialze true state of 3-region grid
region1 = ones(gridDims(1),gridDims(2));
region1(floor(gridDims(1)/2 - gridDims(5)/2)+(1:gridDims(5)),1:gridDims(6)) = 0;
region2 = rand(gridDims(1),gridDims(3)); region2(region2<pObstacle) = nan;
region2 = 2 + (region2 - pObstacle/2);
region3 = 3*ones(gridDims(1),gridDims(4));
trueCells = [region1 region2 region3];
% Initialize true state of rover
if rand < pStartTop % set initial position within top-left corner
  vPos = (floor(gridDims(1)/2 - gridDims(5)/2) + gridDims(5) + rand*(gridDims(1) - (floor(gridDims(1)/2 - gridDims(5)/2) + gridDims(5))))/gridDims(1);
else
  vPos = rand*(floor(gridDims(1)/2 - gridDims(5)/2))/gridDims(1);
end
trueRover.pos = [vPos*gridDims(1)/sum(gridDims(2:4)); ... % vertical position in units of world height
                 rand*gridDims(6)/sum(gridDims(2:4)); ... % horizontal position in units of world width
                 rand*360]; % angular position in degrees (with 0 = facing due east, 90 = facing due north)
% Initialze estimated state of 3-region grid
estiCells = [region1 2.5*ones(size(region2)) region3];
% Initialize estimated state of rover
estiRover = [];  
clear region1 region2 region3 vPos;

k = 0;
while(1) % Enter control loop
  visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
  disp(['Decision stage k = ' num2str(k) ': This is where we can prompt user to enter a next goal/command/control, eventually assuming the true state is hidden and so it can be based only on the estimated state']);
  result = input('--> So, what is the control space and (when eventually assuming imperfect state information) how will the (estimated) state that drives the control policy be generated? [for now, just hit enter to proceed]');
  
  %Here, take input [1:9] and translate it to rover.pos
  trueRover.pos = [gridDims(1)/sum(gridDims(2:4)); ... % vertical position in units of world height
                 rand*gridDims(6)/sum(gridDims(2:4)); ... % horizontal position in units of world width
                 rand*360];
  
  disp('--> This is where the outcome of that goal/command/control is simulated (including uncertainty due to system disturbance, sensor noise, etc.), the true state is updated and the single-stage cost is recorded');
  k = k + 1;
  disp(' ');
end
