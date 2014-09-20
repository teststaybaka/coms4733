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
    deviation = 0.01;
    time = tic;
    firstBumped = false;
    
    forward_velocity = 0.05;
    forward_step = 0;
    current_state = 0;
    time_step = 0.2;
    turn_step = 0;
    angle_accu = 0;
    
    while toc(time) < maxDuration
        SetFwdVelRadiusRoomba(serPort, 0.0, inf);
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
        hasWall = WallSensorReadRoomba(serPort);
        
        if ~firstBumped
            if bumped
                firstBumped = true;
                DistanceSensorRoomba(serPort);
                AngleSensorRoomba(serPort);
                x = 0;
                y = 0;
                angle = 0;
                bump_turn(serPort, BumpRight, BumpLeft, BumpFront);
            else
                SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
            end
        else
            if bumped
                forward_step = 0;
                dist = DistanceSensorRoomba(serPort);
                distTravel = distTravel + dist;
                angle = angle + AngleSensorRoomba(serPort);
                x = x + dist*cos(angle);
                y = y + dist*sin(angle);
                disp([x, y, angle, dist]);
                bump_turn(serPort, BumpRight, BumpLeft, BumpFront);
            else
                if ~hasWall
                    disp('no wall');
                    forward_step = 0;
                    dist = DistanceSensorRoomba(serPort);
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);
                    disp([x, y, angle, dist]);
                    turnAngle(serPort, 0.1, -30);
                else
                    %forward_step = forward_step + time_step;
                    dist = DistanceSensorRoomba(serPort);
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);
                    disp([x, y, angle, dist]);
                    SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
                end
            end
            % first bumped check
        end
        pause(time_step);
    end
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