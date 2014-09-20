%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS 4733 - Computational Aspect of Robotics
%
%	Columbia University FALL 2014 
%
%	Team    : 19
%	Members : Xusheng Gao 				        (xg2193)
%		      Yuxuan Xie 				        (yx2284)
%		      Zachary Hideichi Watanabe-Gastel	(zhw2102)
%
%	Homework Assignment 1
%	Using the iRobot Create Matlab Simulator and the iRobot Create
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function finalRad = hw1_sim(serPort)
    
    robotRadius = 0.2;
    maxDuration = 30000;
    distTravel = 0;
    deviation = 0.05;
    time = tic;
    firstBumped = false;
    
    forward_velocity = 0.1;
    forward_step = 0;
    current_state = 0;
    time_step = 0.4;
    turn_step = 0;
    angle_accu = 0;
    x = 0;
    y = 0;
    angle = 0;
    
    while (toc(time) < maxDuration) && (power(x, 2) + power(y, 2) >= power(distTravel*deviation, 2))
        
        pause(time_step);
        
        %SetFwdVelRadiusRoomba(serPort, 0.0, inf);
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
        
        %this is an epic fail sensor, need a perfect perpendicular >"<
        %hasWall = WallSensorReadRoomba(serPort);
        
        if ~firstBumped
            if bumped
                firstBumped = true;
                
                %set point for distance and angle variable (reference point)
                DistanceSensorRoomba(serPort);
                AngleSensorRoomba(serPort);
                
                bump_turn(serPort, BumpRight, BumpLeft, BumpFront);
            else
                SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
            end
        else
            if bumped
                forward_step = 0;
                % distance from last position
                dist = DistanceSensorRoomba(serPort);
                
                %calculating new parameters
                distTravel = distTravel + dist;
                angle = angle + AngleSensorRoomba(serPort);
                x = x + dist*cos(angle);
                y = y + dist*sin(angle);
                
                fprintf('%.3f, %.3f, %.3f, %.3f\n', x, y, angle, dist);
                fprintf('Turning State \n\n');
                bump_turn(serPort, BumpRight, BumpLeft, BumpFront);
            else
                if forward_step >= (3 * time_step)
                    forward_step = 0;
                    % distance from last position
                    dist = DistanceSensorRoomba(serPort);
                    
                    %calculating new parameters
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);
                    
                    fprintf('%.3f, %.3f, %.3f, %.3f\n', x, y, angle, dist);
                    fprintf('Turning State \n\n');
                    turnAngle(serPort, 0.1, -30);
                else
                    forward_step = forward_step + time_step;
                    % distance from last position
                    dist = DistanceSensorRoomba(serPort);
                    
                    %calculating new parameters
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);
                    
                    fprintf('%.3f, %.3f, %.3f, %.3f\n', x, y, angle, dist);
                    fprintf('Moving State \n\n');
                    SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
                end
            end  % roomba movement
        end % first bumped check        
    end %end of while loop
    
    SetFwdVelRadiusRoomba(serPort, 0.0, inf);
end

function bump_turn(serPort, BumpRight, BumpLeft, BumpFront)
    if BumpRight
        turnAngle(serPort, 0.1, 15);
    elseif BumpLeft
        turnAngle(serPort, 0.1, 105);
    elseif BumpFront
        turnAngle(serPort, 0.1, 90);
    end
end