% GridWorld.m
clear all;
clc;
clf;


% Create the set of actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%
create_actions
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Display new restriction message to user
%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp("As of 11/11/2016, the input received only corresponds to what angle the robot is to move to.")
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
global trueCells = [region1 region2 region3];  



% Initialize true state of rover
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if rand < pStartTop % set initial position within top-left corner
  vPos = (floor(gridDims(1)/2 - gridDims(5)/2) + gridDims(5) + rand*(gridDims(1) - (floor(gridDims(1)/2 - gridDims(5)/2) + gridDims(5))))/gridDims(1);
else
  vPos = rand*(floor(gridDims(1)/2 - gridDims(5)/2))/gridDims(1);
endif


trueRover.pos = [vPos*gridDims(1)/sum(gridDims(2:4)); ... % vertical position in units of world height
                 rand*gridDims(6)/sum(gridDims(2:4)); ... % horizontal position in units of world width
                 rand*360]; % angular position in degrees (with 0 = facing due east, 90 = facing due north)
                 
% Insert current position of rover at step k = 0
true_coordinate_matrix(1,1) = trueRover.pos(2);
true_coordinate_matrix(2,1) = trueRover.pos(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Initialze estimated state of 3-region grid
estiCells = [region1 2.5*ones(size(region2)) region3];

% Initialize estimated state of rover
estiRover = [];  

% Clear cached data that is unnecessary at this state in the program.
clear region1 region2 region3 vPos;



%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ask user if they would 
% like to run in manual 
% control, or simulation
disp("\n\n")
progCmd = input("Would you like to run in manual controls(1) or in AI simulation(2)? ");
disp("\n\n")

if progCmd == 2
  disp("Simulation mode not available, switching to manual controls mode.\n")
  disp("*****************************************************************")
  disp("************************ MANUAL CONTROLS ************************")
  disp("*****************************************************************\n")
  
  progCmd = 1;
endif
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%


if progCmd == 1
  
  % Display manual controls.
  disp("COMMANDS TO CORRESPONDING ANGLES:"  )
  disp("-----------------------------------")
  disp("    1 = 0 degrees   (east)"         )
  disp("    2 = 45 degrees  (north east)"   )
  disp("    3 = 90 degrees  (north)"        )
  disp("    4 = 135 degrees (north west)"   )
  disp("    5 = 180 degrees (west)"         )
  disp("    6 = 225 degrees (south west)"   )
  disp("    7 = 270 degrees (south)"        )
  disp("    8 = 315 degrees (south east)"   )
  disp("    9 = remain stationary"          )
  disp("-----------------------------------")
  disp("\n")
  
  persistent distVtime = [];
  inter_dvt = zeros(2,1)
  k = 0;
  while(1) % Enter control loop
    visualizeGrid(trueCells,trueRover,k,estiCells,estiRover);
    
    %distVtime = [];
    
    % If current theta > 360, subtract 360 iteratively to prevent
    % reduntant theta representations
    if trueRover.pos(3) >= 360
      for test = trueRover.pos(3):-360:0
        trueRover.pos(3) = test
      endfor
    endif
    
    % If theta is negative, make it the equivalent positive theta
    if trueRover.pos(3) < 0
      trueRover.pos(3) = 360 - abs(trueRover.pos(3))
    endif
    
    % disp(['Decision stage k = ' num2str(k) ': This is where we can prompt user to enter a next goal/command/control, eventually assuming the true state is hidden and so it can be based only on the estimated state']);
    
    disp(["--> So, what is the control space and (when eventually assuming imperfect state information) how will the (estimated) state that drives the control policy be generated? [for now, just hit enter to proceed]"])
    
    % Receive command from user
    result = input(["\n\nCommand at k=" num2str(k) ": "]);
    
    % Display rovers current real position (result = 0)
    if result == 0    
      printf("\n*** TRUE POSITION ***\n\n")
      disp(trueRover.pos)
      printf("*********************\n\n\n\n")
    endif
    
    % Receive number of grid spaces to move from user
    spacesToMove = input("Number of grid spaces to move: ");
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Here, take input [1:9] and translate it to rover.pos
    
    
    % Command does not ask to display current real position
    if result != 0
      
      % Command exists in "actions" list
      if find(actions==result)
        [nextX, nextY, nextTheta, deltaTheta, deltaStep_Theta, deltaStep_Move] = user_update_position(result, spacesToMove, trueRover, actions, pTransSpd, pAngleSpd)
        
        % Create persistent distVtime matrix to plot net translation over time steps
        
  
        % Cycle through the steps here.
        % First, rotate the rover
        for iStep = k+1:1:ceil(deltaStep_Theta + k)
          
          %Plot lateral distance vs time steps
          % THIS IS BUGGED
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          subplot(3,1,2)
          %hold on;
          title('Net Translation vs Time-Steps');
          %distVtime(2,k+1) = 0; % 0 translational delta when rotating
          distVtime(1,iStep) = iStep;
          distVtime(2,iStep) = inter_dvt(2,1);
          plot(distVtime(1,:), distVtime(2,:));
          %hold off;
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
          % Correctly represent rotation to new theta
          if (sqrt(((trueRover.pos(3) - nextTheta))^2)) <= 180
            
            % No change in theta
            if deltaTheta == 0
              disp("\n*** No change in theta\n")
              trueRover.pos(3) += 0;
            
            % Iteratively add to current theta
            % Negative delta-theta = clockwise rotation
            % Positive delta-theta = counter-clockwise rotation
            else 
              %disp("\n*** delta < 180 : Rotate clockwise, Theta_1 >= Theta_2\n")
              trueRover.pos(3) += (((deltaTheta)/pAngleSpd) / deltaStep_Theta) * pAngleSpd;  
            endif
           
          else 
            %trueRover.pos(3) += abs((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360);
            % Subtract angle to rotate clockwise          
            if nextTheta >= trueRover.pos(3)
              %disp("\n*** delta >180 : Rotate clockwise, Theta_1 <= Theta_2\n")
              trueRover.pos(3) -= sqrt((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360)^2);;
             
            % Add angle to rotate counter clockwise
            else 
              %disp("\n*** delta > 180 : Rotate counter-clockwise, Theta_1 >= Theta_2\n") 
              trueRover.pos(3) += sqrt((min([nextTheta, trueRover.pos(3)]) - max([nextTheta, trueRover.pos(3)]) + 360)^2);
            endif
          endif
          
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
        endfor
        
        % Translate the rover
        for jStep = k+1:1:ceil(deltaStep_Move + k)
          
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
          
          trueRover.pos(2) += (((nextX-trueRover.pos(2))/pTransSpd) / deltaStep_Move) * pTransSpd;
          trueRover.pos(1) += (((nextY-trueRover.pos(1))/pTransSpd) / deltaStep_Move) * pTransSpd;
          
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
          
        endfor
        
        % steps are integers >= 0, round k towards positive infinity.
        k = ceil(k);
        
        
        
        
        
      % The command is invalid
      else
        disp("\n\n*** Error: Invalid input- input must be any integer 1 through 9.\n\n")
      endif
    endif
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if (k!= 0) 
      inter_dvt(1,1) = distVtime(1,k)
      inter_dvt(2,1) = distVtime(2,k)
    endif
    inter_dvt(inter_dvt == 0) = [];
    pause(.5);
    
    disp('--> This is where the outcome of that goal/command/control is simulated (including uncertainty due to system disturbance, sensor noise, etc.), the true state is updated and the single-stage cost is recorded');
    k = k + 1;
    disp(' ');
    
    
    
  end

else 
  
  disp("*****************************************************************")
  disp("************************* AI SIMULATION *************************")
  disp("*****************************************************************\n")
  
  
  % put AI function calls here. 
endif