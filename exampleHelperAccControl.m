function [roll,pitch,thrust] = exampleHelperAccControl(acc,yaw,DroneMass,Gravity)
% Copyright 2022 The MathWorks, Inc.

    xddot = acc(1);
    yddot = acc(2);
    zddot = acc(3);
    
    pitch = atan2((-xddot*cos(yaw)-yddot*sin(yaw)),(-zddot+Gravity));
   
    roll = atan2(((-xddot*sin(yaw)+yddot*cos(yaw))*cos(pitch)),(-zddot+Gravity));
    
    sat_level = 25; % max roll and pitch angle allowed in degrees
    pitch = min(sat_level*pi/180,max(-sat_level*pi/180,pitch));
    roll = min(sat_level*pi/180,max(-sat_level*pi/180,roll));
    
    thrust = DroneMass*(-zddot+Gravity)/(cos(pitch)*cos(roll));
end