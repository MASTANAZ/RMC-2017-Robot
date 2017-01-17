% Get Sensor Data
% Simulate sensor data 
%
% The generated data will be derived from the immediate position
% of the rover, then a random deviation will be applied within a 
% standard error in order to simulate noise and inaccuracy of
% measurements.
%
% Initial Position takes a 1x3 matrix of [x, y, r]
function aggregatedData = createSimulatedSensorData(initialPosition, trueMap)
    detectorData = generateDetectorData(initialPosition, trueMap);
    positionData = generatePositionData(initialPosition);
    
    aggregatedData = [positionData detectorData];
   
end

% Generated detector data assumes a specified standard error in 
% measurement that may not reflect actual positioning accuracy.
function detectorData = generateDetectorData(initialPosition, trueMap)

    % Display WIP warning message
    disp('WARNING ***createSimulatedSensorData - generateDetectorData***');
    disp('Not currently able to detect patch values, functionality being developed');

    devFR = 0; % Detector deviation on the front, right of robot
    devFL = 0; % Detector deviation on the front, left of robot

    % Assuming that the real detector data is a float [0:1]
    %
    % Use orientation of robot to find what patches should be 
    % should be detected in front of the robot. Detection spans
    % One unit outward from the robot.
    
    % get patch values here
    leftDetectorValue = 0;
    rightDetectorValue = 0;


    % Apply random deviation within standard deviation of .100
    deviations = round(((-0.100 - 0.100).*rand(1000,1) + 0.100), 4);
    devFR = deviations(randi(1000));
    devFL = deviations(randi(1000));

    % clear unneeded variable
    clear deviations;
    
    % Add detector deviation to patch values
    leftDetectorValue = leftDetectorValue + devFL;
    rightDetectorValue = rightDetectorValue + devFR;  

    % Insert noisy detector values in the detector data variable
    detectorData = [leftDetectorValue rightDetectorValue];

end


% Generated position data assumes a specified standard error in 
% measurement that may not reflect actual positioning accuracy.
function positionData = generatePositionData(initialPosition)
    devX = 0; % Noisy x-coordinate
    devY = 0; % Noisy y-coordinate
    devR = 0;
    
    % Apply random deviation within standard deviation of 1.000
    deviations = round(((-1.000 - 1.000).*rand(1000,1) + 1.000), 4);
    devX = deviations(randi(1000));
    devY = deviations(randi(1000));
    devR = deviations(randi(1000));
    
    % clear unneeded variable
    clear deviations;
    
    % Add detector deviation to patch values
    xValue = initialPosition(1) + devX;
    yValue = initialPosition(2) + devY;  
    rValue = intitialPostion(3) + devR;
    
    % Insert noisy detector values in the detector data variable
    positionData = [xValue yValue rValue];

end