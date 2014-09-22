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

    %define state for the state machine
    INITIAL        = 0;
    MOVING         = 1;
    TURNING        = 2;
    BUMP           = 3;
    FINDING_ORIGIN = 4;
    
    current_state  = INITIAL;
    next_state     = INITIAL;
    
    maxDuration = 30000;
    distTravel = 0;
    deviation = 0.05;
    time = tic;
    
    forward_velocity = 0.05;
    forward_step = 0;
    time_step = 0.1;
    
    x = 0;
    y = 0;
    angle = 0;
    
    while (toc(time) < maxDuration) && (power(x, 2) + power(y, 2) >= power(distTravel*deviation, 2))
        pause(time_step);
        
        %this is an epic fail sensor, need a perfect perpendicular >"<
        %hasWall = WallSensorReadRoomba(serPort);
        
        switch current_state
            
            case INITIAL
                
                [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                bumped = BumpRight || BumpLeft || BumpFront;
                
                if bumped
                    SetFwdVelRadiusRoomba(serPort, 0, inf);
                    %set point for distance and angle variable (reference point)
                    DistanceSensorRoomba(serPort);
                    AngleSensorRoomba(serPort);

                    bump_turn(serPort, BumpRight, BumpLeft, BumpFront);
                    
                    next_state = MOVING;
                else
                    SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
                    next_state = INITIAL;
                end
                
                fprintf('initial state\n');
                
            case MOVING
                
                [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                bumped = BumpRight || BumpLeft || BumpFront;
                
                if bumped
                    next_state = BUMP;
                else
                    forward_step = forward_step + time_step;
                    % distance from last position
                    dist = DistanceSensorRoomba(serPort);

                    %calculating new parameters
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);

                    print_status(x, y, angle, dist, distTravel);
                    SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
                    
                    if forward_step >= 10
                        next_state = TURNING;
                    else
                        next_state = MOVING;
                    end
                end
                
                fprintf('moving state\n');
                
            case TURNING
                
                forward_step = 0;
                % distance from last position
                dist = DistanceSensorRoomba(serPort);

                %calculating new parameters
                distTravel = distTravel + dist;
                angle = angle + AngleSensorRoomba(serPort);
                x = x + dist*cos(angle);
                y = y + dist*sin(angle);

                print_status(x, y, angle, dist, distTravel);
                turnAngle(serPort, 0.1, -45);

                SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
                pause(4*time_step);

                [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
                bumped = BumpRight || BumpLeft || BumpFront;

                if ~bumped 
                    turnAngle(serPort, 0.1, -45);
                end
                
                next_state = MOVING;
                
                fprintf('turning state\n');
                
            case BUMP
                
                 %stop the robot if bumped
                SetFwdVelRadiusRoomba(serPort, 0, inf);
                forward_step = 0;
                % distance from last position
                dist = DistanceSensorRoomba(serPort);
                
                %calculating new parameters
                distTravel = distTravel + dist;
                angle = angle + AngleSensorRoomba(serPort);
                x = x + dist*cos(angle);
                y = y + dist*sin(angle);
                
                print_status(x, y, angle, dist, distTravel);
                bump_turn(serPort, BumpRight, BumpLeft, BumpFront);
                
                next_state = MOVING;
                
                fprintf('bump state\n');
                
            case FINDING_ORIGIN
                
            otherwise
                fprintf('ERR: Should not end up here.');
        end
        
        current_state = next_state;  
    end %end of while loop
    
    SetFwdVelRadiusRoomba(serPort, 0.0, inf);
end

function bump_turn(serPort, BumpRight, BumpLeft, BumpFront)
    if BumpRight
        turnAngle(serPort, 0.1, 20);
    elseif BumpLeft
        turnAngle(serPort, 0.1, 105);
    elseif BumpFront
        turnAngle(serPort, 0.1, 90);
    end
end

function print_status(x, y, angle, dist, distTravel)
    fprintf('%.3f, %.3f, %.3f, %.3f, %.3f\n', x, y, angle, dist, distTravel);
end