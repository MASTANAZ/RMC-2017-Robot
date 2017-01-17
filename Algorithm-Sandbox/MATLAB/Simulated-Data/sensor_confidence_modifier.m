% sensor_confidence_modifier

function confidence_interval = sensor_confidence_modifier(rover, patchCenter)

    % The confidence interval for measuring a patch should be a function
    % of the distance between the rover and the patch, and the angular
    % offset of the robot to the patch. 
    
    % This function should be able to be retrieved from the data sheet for 
    % the IR rangefinder sensor. 

    % Note that the confidence is inversely related to the distance. Then,
    % the angular offset should be factored, in which case the confidence
    % due to the offset is inversely related to the angle. 
    
end