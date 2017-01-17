%sim_upate_position
% Algorithnm-Sandbox
%
% Created by Blake Nazario-Casey on 11/11/2016
% Last updated by Blake Nazario-Casey on 11/11/2016
%
%
% DESCRIPTION:
%     Updates the rover's true position based on user input.
% 
% INPUT PARAMS   :             DESCRIPTION              : TYPE
% ------------------------------------------------------------------
%       command  :  element of global set 'actions'     : integer
%        spaces  :  copy of 'spacesToMove' integer      : 1x1 struct
%     estiRover  :  copy of 'trueRover' struct          : 1x1 struct
%    actionList  :  local copy of global set 'actions'  : 1x9 list
%    transSpeed  :  copy of 'pTransSpd' float           : float
%    angleSpeed  :  copy of 'pAngleSpd' float           : float
%
%
% RETURNS        :             DESCRIPTION              : TYPE
% ------------------------------------------------------------------
%          newX  :  an updated rover's x-position       : float 
%          newY  :  an updated rover's y-position       : float            
%      newTheta  :  an updated rover's orientation      : integer
%    deltaTheta  :  change in theta after command       : float
% newThetaSteps  :  num-of-steps to rotate to new theta : integer
%  newMoveSteps  :  num-of-steps to move to new (x,y)   : integer
%
%
% COMMANDS TO CORRESPONDING ANGLES:
% -----------------------------------
%    1 = 0 degrees   (east)
%    2 = 45 degrees  (north east)
%    3 = 90 degrees  (north)
%    4 = 135 degrees (north west)
%    5 = 180 degrees (west)
%    6 = 225 degrees (south west)
%    7 = 270 degrees (south)
%    8 = 315 degrees (south east)
%    9 = remain stationary
%
function [newX, newY, newTheta, deltaTheta, newThetaSteps, newMoveSteps] = user_update_position(theta, spaces, estiRover, actionList, transSpeed, angleSpeed)

  disp('Estimated position before executing manual movement command (y, x, orientation): \n')
  disp(estiRover.pos) 

  % Command to modify current position and/or orientation
  
  
    % 1) Get current angle by estiRover.pos(3)
    % 2) Get corresponding angle for command using cmdToAngle(command)
    currentAngle = estiRover.pos(3)
    newTheta = theta
    
    %   2.a) Store both thetas in a set for perfomring min/max 
    thetaSet = [estiRover.pos(3), newTheta]
    
    % 3) Move to corresponding command angle 
    %   3.a) Movement to command angle will take some number k-steps,
    %        and is retrieved by: (cmdToAngle(command) - estiRover.pos(3)) / angleSpeed
    %   3.b) Add number of steps it takes to move to new angle to 'totalSteps'
    if (sqrt(((estiRover.pos(3) - newTheta))^2)) <= 180
      deltaTheta = (newTheta - estiRover.pos(3))
    else 
      deltaTheta = (min(thetaSet) - max(thetaSet) + 360)
    end    
    
    % 4) Calculate new estiRover's (x,y)
    %   4.a) new pos.x = estiRover.x + (spaces/20) * cos(cmdToAngle(command) radians) 
    %   4.b) new pos.y = estiRover.y + (spaces/12) * sin(cmdToAngle(command) radians)
    %
    %   NOTE: using (spaces/m) to account for m grid spaces on each axis
    newX = estiRover.pos(2) + (spaces/24) * cos(theta*pi/180)
    newY = estiRover.pos(1) + (spaces/24) * sin(theta*pi/180)
    
    
    
    %
    % Minimize delta-theta & steps to move to new theta
    %   4.c) Movement to new position will take some number k-steps,
    %        and is retrieved by: delta theta / angleSpeed
    %
    %   NOTE: Steps must be >= 0, therefore totalSteps must be >= 0
    %         sqrt(n^2) >= 0 for all n
    if (sqrt(((estiRover.pos(3) - newTheta))^2)) <= 180
      newThetaSteps = sqrt(((newTheta - estiRover.pos(3)) / angleSpeed)^2)
    else 
      newThetaSteps = sqrt(((min(thetaSet) - max(thetaSet) + 360) / angleSpeed)^2)
    end
  
    %   4.d) Add number of steps it takes to move to new position to 'totalSteps'
    newMoveSteps = sqrt(((spaces/20) / transSpeed)^2)
    

end