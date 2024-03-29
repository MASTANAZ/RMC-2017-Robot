% GridWorld.m
clear all;
clc;
clf;

% Adding path directory for all d-star files
addpath('D-Star');

% Adding path for simulated sensor data
addpath('Simulated-Data');

% Adding path for objective prioritization
addpath('Objective-Priority');


% Create the set of actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%
create_actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Display new restriction message to user
%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('As of 11/11/2016, the input received only corresponds to what angle the robot is to move to.')
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Some simulaton parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gridDims = [12 5 10 9 4 2]; % number of rows, number of columns in the three regions, number of rows and columns of unload station
pObstacle = 0.1;  % per-cell probability of obstacle being present (in region 2)
pStartTop = 0.5;  % probability of rover starting in top-left (vs bottom-left) corner
pTransSpd = 0.01; % rover's translational speed (assuming flat terrain, in units of world width per decision stage)
pAngleSpd = 5.0;  % rover's angular speed (in units of degrees per decision stage)
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialze true state of 3-region grid
region1 = ones(gridDims(1),gridDims(2));
region1(floor(gridDims(1)/2 - gridDims(5)/2)+(1:gridDims(5)),1:gridDims(6)) = 0;
region2 = rand(gridDims(1),gridDims(3)); region2(region2<pObstacle) = nan;
region2 = 2 + (region2 - pObstacle/2);
region3 = 3*ones(gridDims(1),gridDims(4));

% Global for now to apply weights when calculating steps to traverse obstacle
global trueCells;
trueCells = [region1 region2 region3];  
trueMap = [region1 region2 region3]


% Initialize true state of rover
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if rand < pStartTop % set initial position within top-left corner
  vPos = (floor(gridDims(1)/2 - gridDims(5)/2) + gridDims(5) + rand*(gridDims(1) - (floor(gridDims(1)/2 - gridDims(5)/2) + gridDims(5))))/gridDims(1);
else
  vPos = rand*(floor(gridDims(1)/2 - gridDims(5)/2))/gridDims(1);
end

trueRover.pos = [vPos*gridDims(1)/sum(gridDims(2:4)); ... % vertical position in units of world height
                 rand*gridDims(6)/sum(gridDims(2:4)); ... % horizontal position in units of world width
                 rand*360]; % angular position in degrees (with 0 = facing due east, 90 = facing due north)

while (trueRover.pos(1) > 0.5 || ...
       trueRover.pos(1) < 0)
       trueRover.pos(1) = vPos*gridDims*(1)/sum(gridDims(2:4));
end

while (trueRover.pos(1) > 0.5 || ...
       trueRover.pos(1) < 0)
       trueRover.pos(2) = rand*gridDims(6)/sum(gridDims(2:4));
end
% Insert current position of rover at step k = 0
true_coordinate_matrix(1,1) = trueRover.pos(2);
true_coordinate_matrix(2,1) = trueRover.pos(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Initialze estimated state of 3-region grid
global estiCells;
estiCells = [region1 2.5*ones(size(region2)) region3];

% estiRegion1 = ones(gridDims(1),gridDims(2));
% estiRegion1(floor(gridDims(1)/2 - gridDims(5)/2)+(1:gridDims(5)),1:gridDims(6)) = 0;
% region2 = rand(gridDims(1),gridDims(3)); region2(region2<pObstacle) = nan;
% region2 = 2 + (region2 - pObstacle/2);
% region3 = 3*ones(gridDims(1),gridDims(4));
% 
% estiMap = 
% Initialize estimated state of rover
estiRover.pos = [0 0 0];  

% Clear cached data that is unnecessary at this state in the program.
clear region1 region2 region3 vPos;



%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ask user if they would 
% like to run in manual 
% control, or simulation
disp('\n\n')
progCmd = input('Would you like to run in manual controls(1) or in AI simulation(2)? ');
disp('\n\n')

% if progCmd == 2
%   disp('Simulation mode not available, switching to manual controls mode.\n')
%   disp('*****************************************************************')
%   disp('************************ MANUAL CONTROLS ************************')
%   disp('*****************************************************************\n')
%   
%   progCmd = 1;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%


if progCmd == 1
  
  % Display manual controls.
  disp('COMMANDS TO CORRESPONDING ANGLES:'  )
  disp('-----------------------------------')
  disp('    1 = 0 degrees   (east)'         )
  disp('    2 = 45 degrees  (north east)'   )
  disp('    3 = 90 degrees  (north)'        )
  disp('    4 = 135 degrees (north west)'   )
  disp('    5 = 180 degrees (west)'         )
  disp('    6 = 225 degrees (south west)'   )
  disp('    7 = 270 degrees (south)'        )
  disp('    8 = 315 degrees (south east)'   )
  disp('    9 = remain stationary'          )
  disp('-----------------------------------')
  disp('\n')
  
  distVtime = [];
  
  inter_dvt = zeros(2,1)
  k = 0;
  while(1) % Enter control loop
    visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
    
    
    addpath('D-Star')    % Adding path directory for all d-star file

  % Draw the optimal path
  %**********************
%   load trueMap;

  % Create initial cost map (initializer)
  graph = zeros(12,24);
  
  
  goal = [23,11]; % Goal will eventually be dynamically determined
  start=[round(trueRover.pos(2)*24),round(trueRover.pos(1)*24)];
  
  
  
  % Initialize Algorithm with empty cost map
  ds = Dstar(graph);    % create navigation object
  
  
  % Modify the cost map to reflect the costs of the actual map
  for r=1:12
    for c=6:15
        % If there is a NaN, make the cost Infinity
        if (isnan(trueMap(r,c)))
            ds.modify_cost([c,r], Inf)
        else
            ds.modify_cost([c,r], trueMap(r,c)); 
        end
    end
  end
  
  ds.plan(goal)        % create plan for specified goal

  thePath = ds.path(start)
  
  subplot(3,1,1);
  %ds.plot  % use this to visualize cost map
  hold on;
  plot(thePath(:,1)/24 - 1/48,thePath(:,2)/24 - 1/48,'r-', 'LineWidth',2);
  hold off;
  %%%%%%%%%%%%%%%%%%%%%%%
    
    %distVtime = [];
    
    % If current theta > 360, subtract 360 iteratively to prevent
    % reduntant theta representations
    if trueRover.pos(3) >= 360
      for test = trueRover.pos(3):-360:0
        trueRover.pos(3) = test
      end
    end
    
    % If theta is negative, make it the equivalent positive theta
    if trueRover.pos(3) < 0
      trueRover.pos(3) = 360 - abs(trueRover.pos(3))
    end
    
    % disp(['Decision stage k = ' num2str(k) ': This is where we can prompt user to enter a next goal/command/control, eventually assuming the true state is hidden and so it can be based only on the estimated state']);
    
    disp(['--> So, what is the control space and (when eventually assuming imperfect state information) how will the (estimated) state that drives the control policy be generated? [for now, just hit enter to proceed]'])
    
    % Receive command from user
    result = input(['\n\nCommand at k=' num2str(k) ': ']);
    
    % Display rovers current real position (result = 0)
    if result == 0    
      printf('\n*** TRUE POSITION ***\n\n')
      disp(trueRover.pos)
      printf('*********************\n\n\n\n')
    end
    
    % Receive number of grid spaces to move from user
    spacesToMove = input('Number of grid spaces to move: ');
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Here, take input [1:9] and translate it to rover.pos
    
    
    % Command does not ask to display current real position
    if result ~= 0
      
      % Command exists in 'actions' list
      if find(actions==result)
        [nextX, nextY, nextTheta, deltaTheta, deltaStep_Theta, deltaStep_Move] = user_update_position(result, spacesToMove, trueRover, actions, pTransSpd, pAngleSpd);
        
        % Create persistent distVtime matrix to plot net translation over time steps

        % Cycle through the steps here.
        % First, rotate the rover
        for iStep = k+1:1:ceil(deltaStep_Theta + k)
          
          %Plot lateral distance vs time steps
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          subplot(3,1,2)
          %hold on;
          title('Net Translation vs Time-Steps');
          %distVtime(2,k+1) = 0; % 0 translational delta when rotating
          graphX = 0;
          if (k == 0) 
              graphX = iStep;
          else 
              graphX = k;
          end
          
          distVtime(1,graphX) = graphX;
          distVtime(2,graphX) = inter_dvt(2,1);
          plot(distVtime(1,:), distVtime(2,:));
          %hold off;
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
          % Correctly represent rotation to new theta
          if (sqrt(((trueRover.pos(3) - nextTheta))^2)) <= 180
            
            % No change in theta
            if deltaTheta == 0
              disp('\n*** No change in theta\n')
              trueRover.pos(3) = trueRover.pos(3) + 0;
            
            % Iteratively add to current theta
            % Negative delta-theta = clockwise rotation
            % Positive delta-theta = counter-clockwise rotation
            else 
              %disp('\n*** delta < 180 : Rotate clockwise, Theta_1 >= Theta_2\n')
              trueRover.pos(3) = trueRover.pos(3) + (((deltaTheta)/pAngleSpd) / deltaStep_Theta) * pAngleSpd;  
            
            end
           
          else 
            %trueRover.pos(3) += abs((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360);
            % Subtract angle to rotate clockwise          
            if nextTheta >= trueRover.pos(3)
              %disp('\n*** delta >180 : Rotate clockwise, Theta_1 <= Theta_2\n')
              trueRover.pos(3) = sqrt((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360)^2) - trueRover.pos(3);
             
            % Add angle to rotate counter clockwise
            else 
              %disp('\n*** delta > 180 : Rotate counter-clockwise, Theta_1 >= Theta_2\n') 
              trueRover.pos(3) = trueRover.pos(3) + sqrt((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360)^2);
            end
          end
          
          k = iStep;
          
          
          % Load updated values
          true_coordinate_matrix;
          
          % Update coordinate matrix 
          % NOTE: These coordinates should remain the same,
          %       but will be inserted to maintain continuity 
          %       of position at any k-step
          %true_coordinate_matrix(1,k) = trueRover.pos(2);
          %true_coordinate_matrix(2,k) = trueRover.pos(1);
          
          
          % pause for animation
          pause(.1)
          % SPRITE ANIMATION FUNCTION HERE
          % Animate current step in iteration
          visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
        end
        
        % Translate the rover
        for jStep = k:1:ceil(deltaStep_Move + k)
          
          %Plot lateral distance vs time steps
          % THIS IS BUGGED
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          subplot(3,1,2)
          %hold on;
          
          distVtime(2,jStep) = sqrt(((nextY - trueRover.pos(1))^2) + ((nextX - trueRover.pos(1))^2)) + distVtime(2,jStep-1);
          distVtime(1,jStep) = jStep;
          plot(distVtime(1,:), distVtime(2,:));
          %hold off;
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
          trueRover.pos(2) = trueRover.pos(2) + (((nextX-trueRover.pos(2))/pTransSpd) / deltaStep_Move) * pTransSpd;
          trueRover.pos(1) = trueRover.pos(1) + (((nextY-trueRover.pos(1))/pTransSpd) / deltaStep_Move) * pTransSpd;
          
          k = ceil(jStep);
          
          
          % Load updated values
          %true_coordinate_matrix;
          
          % Update coordinate matrix 
          %true_coordinate_matrix(1,k) = trueRover.pos(2);
          %true_coordinate_matrix(2,k) = trueRover.pos(1);          
          
          
          % pause for animation
          pause(.1)
          % SPRITE ANIMATION FUNCTION HERE
          % Animate current step in iteration
          visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
          
        end
        
        % steps are integers >= 0, round k towards positive infinity.
        k = ceil(k);
        
        
        
        
        
      % The command is invalid
      else
        disp('\n\n*** Error: Invalid input- input must be any integer 1 through 9.\n\n')
      end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    if (k~= 0) 
      inter_dvt(1,1) = distVtime(1,k)
      inter_dvt(2,1) = distVtime(2,k)
    end
    inter_dvt(inter_dvt == 0) = [];
    pause(.5);
    
    disp('--> This is where the outcome of that goal/command/control is simulated (including uncertainty due to system disturbance, sensor noise, etc.), the true state is updated and the single-stage cost is recorded');
    k = k + 1;
    disp(' ');
    
    
    
  end
  
else
    
  disp('*****************************************************************')
  disp('************************* AI SIMULATION *************************')
  disp('*****************************************************************\n')
  
  
  % put AI function calls here. 
  
  distVtime = [];
  
  inter_dvt = zeros(2,1)
  k = 0;
  
  path_index = 1;
  aggData = createSimulatedSensorData([trueRover.pos(2), trueRover.pos(1), trueRover.pos(3)], trueMap);
  estiRover.pos = [aggData(2); ... % vertical position in units of world height
                    aggData(1); ... % horizontal position in units of world width
                    aggData(3)];
  
%   load trueMap;
  goal = [20,9];
  
  % Create initial cost map (initializer)
  graph = zeros(12,24)
  
  % Create sensor map; it should start with the general cost
  % because the rover does not know anything about the environment yet
  sensorMap = zeros(12,24);
  sensorMap(:) = 1;
  
  %if (k+2 ~= length(thePath))
        %theta = sim_update_theta(estiRover.pos(2), estiRover.pos(1), thePath(1,1), thePath(1,2));
  %end
  while(2)
      % UPDATE ESTIMATED CELLS HERE
     %simSensorData = createSimulatedSensorData([estiRover.pos(2), estiRover.pos(1), estiRover.pos(3)], trueMap)
%     
%     estiCells(round(estiRover.pos(2) * 24), round(estiRover.pos(1) * 24)) = (simSensorData(4) + simSensorData(5)) / 2;
%     
    visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);

    % Plan the path every go around
    
    start=[round(estiRover.pos(2)*24),round(estiRover.pos(1)*24)];
    ds = Dstar(graph);    % create navigation object
    
    look_ahead_cost = 0;
    if (estiRover.pos(3) > 90 && estiRover.pos(3)< 270)
        look_ahead_cost = trueCells(round((estiRover.pos(2)-2*cos(estiRover.pos(3)*pi/180)+1)),round((estiRover.pos(1)-sin(estiRover.pos(3)*pi/180)+2)));
        ds.modify_cost([round((estiRover.pos(2)-2*cos(estiRover.pos(3)*pi/180)+1)),round((estiRover.pos(1)-2*sin(estiRover.pos(3)*pi/180)+2))], look_ahead_cost);
        estiCells(round((estiRover.pos(2)-2*cos(estiRover.pos(3)*pi/180)+1)),round((estiRover.pos(1)-2*sin(estiRover.pos(3)*pi/180)+2))) = look_ahead_cost;

    else
        look_ahead_cost = trueCells(round((estiRover.pos(2)+2*cos(estiRover.pos(3)*pi/180)+1)),round((estiRover.pos(1)+sin(estiRover.pos(3)*pi/180)+2)));
        ds.modify_cost([round((estiRover.pos(2)+2*cos(estiRover.pos(3)*pi/180)+1)),round((estiRover.pos(1)+2*sin(estiRover.pos(3)*pi/180)+2))], look_ahead_cost);
        estiCells(round((estiRover.pos(2)+2*cos(estiRover.pos(3)*pi/180)+1)),round((estiRover.pos(1)+2*sin(estiRover.pos(3)*pi/180)+2))) = look_ahead_cost;
    end
    
    
    % Modify cost at current position
    %ds.modify_cost([round((estiRover.pos(2)+cos(estiRover.pos(3)*pi/180))),round((estiRover.pos(1)+cos(estiRover.pos(3)*pi/180)+2))], look_ahead_cost);
    
    % Update estimated state grid
    % Adding 2 to look ahead of rover
    %estiCells(round(estiRover.pos(1)*24),round(estiRover.pos(2)*24+2)) = look_ahead_cost;
    
    % Modify the cost map to reflect the costs of the actual map
  for r=1:12
    for c=6:15
        % If there is a NaN, make the cost Infinity
        if (isnan(sensorMap(r,c)))
            ds.modify_cost([c,r], Inf)
        else
            ds.modify_cost([c,r], sensorMap(r,c)); 
        end
    end
  end
    
    ds.plan(goal)       % create plan for specified goal
    thePath = ds.path(start)
    subplot(3,1,3);
    hold on;
%   ds.path(start);
    plot(thePath(:,1)/24,thePath(:,2)/24,'r-', 'LineWidth',2);
    hold off;
  
    %sim = SimData(trueMap, trueRover);
 
  
    % CheckTerrain will be modified to return a matrix of cells that
    % need to be updated. From here, those updated cells will be put
    % into a new function similar to userUpdatePostion.m called 
    % updatedEstimatedStateMap that will then perform essentially what 
    % userUpdatePosition does, but is tailored to updating the map based on 
    % the robot's estimation rather than true-state data. The function can 
    % also be used in the real rover to update the shared map as each rover 
    % discovers new terrain.
    %vals = sim.checkTerrain(trueMap, trueRover);
    
    if (k+2 ~= length(thePath))
        theta = sim_update_theta(thePath(1,1), thePath(1,2), thePath(2,1), thePath(2,2));
    end
    [nextX, nextY, nextTheta, deltaTheta, deltaStep_Theta, deltaStep_Move] = sim_update_position(theta, 1, estiRover, actions, pTransSpd, pAngleSpd);
    
    
    for iStep = k+1:1:ceil(deltaStep_Theta + k)
        
        k
       graphX = 0;
          if (k == 0) 
              graphX = iStep;
          else 
              graphX = k;
          end
          subplot(3,1,2);
          distVtime(1,graphX) = graphX;
          distVtime(2,graphX) = inter_dvt(2,1);
          plot(distVtime(1,:), distVtime(2,:));
%           hold off;
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
          % Correctly represent rotation to new theta
          if (sqrt(((estiRover.pos(3) - nextTheta))^2)) <= 180
            subplot(3,1,3);
            % No change in theta
            if deltaTheta == 0
              disp('\n*** No change in theta\n')
              estiRover.pos(3) = estiRover.pos(3) + 0;
            
            % Iteratively add to current theta
            % Negative delta-theta = clockwise rotation
            % Positive delta-theta = counter-clockwise rotation
            else 
              %disp('\n*** delta < 180 : Rotate clockwise, Theta_1 >= Theta_2\n')
              estiRover.pos(3) = estiRover.pos(3) + (((deltaTheta)/pAngleSpd) / deltaStep_Theta) * pAngleSpd;  
            
            end
           
          else 
            %trueRover.pos(3) += abs((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360);
            % Subtract angle to rotate clockwise          
            if nextTheta >= trueRover.pos(3)
              %disp('\n*** delta >180 : Rotate clockwise, Theta_1 <= Theta_2\n')
              estiRover.pos(3) = sqrt((min([nextTheta, estiRover.pos(3)]) - max([nextTheta, estiRover.pos(3)]) + 360)^2) - estiRover.pos(3);
             
            % Add angle to rotate counter clockwise
            else 
              %disp('\n*** delta > 180 : Rotate counter-clockwise, Theta_1 >= Theta_2\n') 
              estiRover.pos(3) = estiRover.pos(3) + sqrt((min([nextTheta, estiRover.pos(3)]) - max([nextTheta, estiRover.pos(3)]) + 360)^2);
            end
          end
          
          k = iStep;
          
          
          % Load updated values
          true_coordinate_matrix;
          
          % Update coordinate matrix 
          % NOTE: These coordinates should remain the same,
          %       but will be inserted to maintain continuity 
          %       of position at any k-step
          %true_coordinate_matrix(1,k) = trueRover.pos(2);
          %true_coordinate_matrix(2,k) = trueRover.pos(1);
          
          
          % pause for animation
          pause(.1)
          % SPRITE ANIMATION FUNCTION HERE
          % Animate current step in iteration
          subplot(3,1,3);
          hold on;
%           ds.path(start);
            plot(thePath(:,1)/24,thePath(:,2)/24,'r-', 'LineWidth',2);
            hold off;
          visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
        end
        
        % Translate the rover
        for jStep = k:1:ceil(deltaStep_Move + k)
          
          %Plot lateral distance vs time steps
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          subplot(3,1,2)
          %hold on;
          
          distVtime(2,jStep) = sqrt(((nextY - estiRover.pos(1))^2) + ((nextX - estiRover.pos(1))^2)) + distVtime(2,jStep-1);
          distVtime(1,jStep) = jStep;
          plot(distVtime(1,:), distVtime(2,:));
          %hold off;
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
           estiRover.pos(2) = estiRover.pos(2) + (((nextX-estiRover.pos(2))/pTransSpd) / deltaStep_Move) * pTransSpd;
           estiRover.pos(1) = estiRover.pos(1) + (((nextY-estiRover.pos(1))/pTransSpd) / deltaStep_Move) * pTransSpd;
            %estiRover.pos(2) = thePath(1,1) / 24
            %estiRover.pos(1) = thePath(1,2) / 24
          k = ceil(jStep);
          
          sensorMap(round(estiRover.pos(2) * 24.0)) = trueMap(round(estiRover.pos(2) * 24.0));
          
          
          % Load updated values
          %true_coordinate_matrix;
          
          % Update coordinate matrix 
          %true_coordinate_matrix(1,k) = trueRover.pos(2);
          %true_coordinate_matrix(2,k) = trueRover.pos(1);          
          
          
          % pause for animation
          pause(.1)
          % SPRITE ANIMATION FUNCTION HERE
          % Animate current step in iteration
          subplot(3,1,3);
%           ds.path(start);
          plot(thePath(:,1)/24 - 1/48,thePath(:,2)/24 - 1/48,'r-', 'LineWidth',2);
          visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
          
        end
        
        %estiCells = look_ahead(trueCells, estiRover)

        % steps are integers >= 0, round k towards positive infinity.
        k = ceil(k); 
        
    if (k~= 0) 
      inter_dvt(1,1) = distVtime(1,k)
      inter_dvt(2,1) = distVtime(2,k)
    end
    
    inter_dvt(inter_dvt == 0) = [];
    %pause(.5);
    
    disp('--> This is where the outcome of that goal/command/control is simulated (including uncertainty due to system disturbance, sensor noise, etc.), the true state is updated and the single-stage cost is recorded');
    k = k + 1;
    disp(' ');
    
     path_index = path_index+1
    
  end;
%   clear goal;

end
