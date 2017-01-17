%sim_update_theta
% 
% Update the theta of the simulated rover for the Estimated State Map when
% traversing.

function newSimTheta = sim_update_theta(x1,y1,x2,y2)

    rad_theta = atan((y2-y1)/(x2-x1));
    
    deg_theta = rad_theta * 180 / pi;
    
    newSimTheta = deg_theta;
end