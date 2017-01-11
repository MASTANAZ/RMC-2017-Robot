% Sim Data

classdef SimData
    % All matrix properties are matrices representing data at any
    % k-step (the index)
    %
    % NOTE: All properties of the sim data are percieved by the
    %       robot in the simulation environment, NOT the actual
    %       values
        properties (SetAccess=private, GetAccess=private)
            
            % The aggregated data that represents the aggregated
            % data tensor that is retrieved from the data model
            % on the actual robot. This is the collection of all
            % sensor data.
            aggregatedData
            
            
            %** The observed grid
            observedMap
            
            
            %*** Individual Data ***
            % Locomotion Motor Data
            left_motor_speed
            left_delta_rotation
            
            right_motor_speed
            right_delta_rotation
            %***
            
            % Positioning Data
            current_position
            delta_position
            trueMap
            
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
            
            simData.observedMap = zeros(24,12);
        end
        
        
        % Scan area around robot given 
        function checkForObstacles (simData, trueMap, rover)
            % Scan for obstacles in immediate path of Rover
            % Get rover's current orientation
            
            % There should be 8 grid values around the robot like so:
            %
            %  [ ] [ ] [ ]
            %  [ ]  R  [ ]
            %  [ ] [ ] [ ]
            
            gridValues = [];
            %right
            gridValues(1) = trueMap(round(rover.pos(2) * 24));
            
            % This is only temporary, as this defeats the purpose of the
            % AI. 
            % This would be an example of a partially observable universe
            % in which the robot can directly compare with the known
            % universe to itss estimated universe at its given position. 
            observedMap(round(rover.pos(2) * 24)) = trueMap(round(rover.pos(2) * 24)) 
            
            % Here a function will be called to update the observed graph
            % OR
            % A matrix will be returned that is the updated map and will be
            % returned to the caller.
        end
    end
end