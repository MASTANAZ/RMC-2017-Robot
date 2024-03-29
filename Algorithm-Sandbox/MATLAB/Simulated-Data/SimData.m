% 
% SimData.m
% Algorithnm-Sandbox
%
% Created by Blake Nazario-Casey on 01/10/2017
% Last updated by Blake Nazario-Casey on 01/10/2017
%
%
% DESCRIPTION:
%     SimData is a class that manages all simulation data and 
%     detection
% 
% PROPERTIES         :             DESCRIPTION              : TYPE
% ------------------------------------------------------------------
%    aggregatedData  :  matrix equivalent of data tensor    : MxN matrix
%  left_motor_speed  :  rotational velocity of left motor   : float
% right_motor_speed  :  rotational velocity of right motor  : float
%   currentPosition  :  current (x,y) position of rover     : 1x2 list
%     deltaPosition  :  change in position of rover         : 1x2 list
%       observedMap  :  map of observed terrain by robot    : 24x12 matrix
%           trueMap  :  true-state map of universe          : 24x12 matrix
% detected_obstacles :  matrix of detected obstalces        : 24x22 matrix
%
%
%
%
% INPUTS           :             DESCRIPTION              : TYPE
% ------------------------------------------------------------------
%         trueMap  :  actual map of true state universe   : 24x12 matrix
%           rover  :  rover position information          : 1x3 list
%
%
%
% RETURNS        :             DESCRIPTION              : TYPE
% ------------------------------------------------------------------
%       simData  :  class of simulation data objects    : SimData 
% 
%
% The checkForObstacles function checks all at 8 points around the rover
% for each step, returns terrain data that is used to update the 
% observed map.
%
% All other function information can be found above and within each
% function
%


classdef SimData
    % All matrix properties are matrices representing data at any
    % k-step (the index)
    %
    % NOTE: All properties of the sim data are percieved by the
    %       robot in the simulation environment, NOT the actual
    %       values
        properties (SetAccess=private, GetAccess=public)
            
            % The aggregated data that represents the aggregated
            % data tensor that is retrieved from the data model
            % on the actual robot. This is the collection of all
            % sensor data.
            aggregatedData
            
            
            %*** Individual Data ***
            % Locomotion Motor Data
            left_motor_speed
            right_motor_speed
            %***
            
            % Positioning Data
            current_position
            delta_position
            trueMap
            observedMap
            %***
            
            % Object Detection
            detected_obstacles % 2D matrix where each element is a coord
            %***********************
            
        end
        
    methods
        
        %Constructor & Initializer
        %
        % Input:
        %
        %   estimatedPosition : [x,y] matrix
        %       roverPosition : [x,y,r]
        function simData = SimData(trueMap, rover)
            
            % Update values based on current position
            simData.current_position(1) = rover.pos(2);
            simData.current_position(2) = rover.pos(1);
            simData.current_position(3) = rover.pos(3);
            
            simData.trueMap = trueMap
            
            simData.observedMap = zeros(12,24);

            simData.aggregatedData = createSimulatedSensorData(simData.current_position, trueMap);
            
            checkTerrain(simData, trueMap, rover);
        end
        
        
        % Scan area around robot given 
        % This function needs to be modified to take in the current
        % estimated map so it can return the updated cells
        %
        % The returned updated cells will be run into another function
        function checkTerrain (simData, trueMap, rover)
            % Scan for obstacles in immediate path of Rover
            % Get rover's current orientation
            
            % There should be 8 grid values around the robot like so:
            %
            %  [ ] [ ] [ ]
            %  [ ]  R  [ ]
            %  [ ] [ ] [ ]
            
            %gridValues = [];
            %right
            %gridValues(1) = trueMap(round(rover.pos(2) * 24));
            
            % This is only temporary, as this defeats the purpose of the
            % AI. 
            % This would be an example of a partially observable universe
            % in which the robot can directly compare with the known
            % universe to itss estimated universe at its given position. 
            observedMap(round(rover.pos(2) * 24)) = trueMap(round(rover.pos(2) * 24)) 
            observedMap(round(rover.pos(1) * 24)) = trueMap(round(rover.pos(1) * 24))
            
            % Here a function will be called to update the observed graph
            % OR
            % A matrix will be returned that is the updated map and will be
            % returned to the caller.
        end
    end
end