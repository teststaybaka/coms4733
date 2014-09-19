% COMS 4733 - Computational Aspect of Robotics
%
%	Columbia University FALL 2014 
%	Team    : 19
%	Members : Xusheng Gao 				(xg2193)
%		  Yuxuan Xie 				()
%		  Zachary Hideichi Watanabe-Gastel	(zhw2102)
%
%	Homework Assignment 1
%	Using the iRobot Create Matlab Simulator and the iRobot Create

function finalRad = hw1_sim(serPort)
    
    maxDuration = 2400;
    distTravel = 0;
    deviation = 0.01;
    time = tic;
    firstBumped = false;
    % START!!!
    SetFwdVelRadiusRoomba(serPort, 0.2, inf);
    forwad_step = 0;
    current_state = 0;
    time_step = 0.02;
    turn_step = 0;
    angle_accu = 0;
    
    while toc(time) < maxDuration
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
     
        bumped = BumpRight || BumpLeft || BumpFront;
        
        if ~firstBumped
            if bumped
                firstBumped = true;
                DistanceSensorRoomba(serPort);
                AngleSensorRoomba(serPort);
                x = 0;
                y = 0;
                angle = 0;
                turnAngle(serPort, 0.2, 10);
            end
        else
            if current_state == 0
                if bumped
                    forwad_step = 0;
                    dist = DistanceSensorRoomba(serPort);
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);
                    disp([x, y, angle, dist]);
                    turnAngle(serPort, 0.2, 10);
                else
                    if forwad_step > 15*time_step
                        forwad_step = 0;
                        current_state = 1;
                        dist = DistanceSensorRoomba(serPort);
                        distTravel = distTravel + dist;
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*cos(angle);
                        y = y + dist*sin(angle);
                        disp([x, y, angle, dist]);
                        turnAngle(serPort, 0.2, -10);
                    else
                        forwad_step = forwad_step + time_step;
                        dist = DistanceSensorRoomba(serPort);
                        distTravel = distTravel + dist;
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*cos(angle);
                        y = y + dist*sin(angle);
                        disp([x, y, angle, dist]);
                        SetFwdVelRadiusRoomba(serPort, 0.2, inf);
                    end
                end
            else
                if bumped
                    current_state = 0;
                    forwad_step = 0;
                    turn_step = 0;
                    angle_accu = 0;
                    dist = DistanceSensorRoomba(serPort);
                    distTravel = distTravel + dist;
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*cos(angle);
                    y = y + dist*sin(angle);
                    disp([x, y, angle, dist]);
                    turnAngle(serPort, 0.2, 10);
                else
                    if forwad_step > 4*time_step
                        forwad_step = 0;
                        turn_step = turn_step + 1;
                        dist = DistanceSensorRoomba(serPort);
                        distTravel = distTravel + dist;
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*cos(angle);
                        y = y + dist*sin(angle);
                        disp([x, y, angle, dist]);
                        if angle_accu + 10*turn_step > 180
                            a = 180 - angle_accu;
                            angle_accu = 180;
                        else
                            a = 10*turn_step;
                            angle_accu = angle_accu + a;
                        end
                        disp([angle_accu, a]);
                        turnAngle(serPort, 0.2, -a);
                    else
                        forwad_step = forwad_step + time_step;
                        dist = DistanceSensorRoomba(serPort);
                        distTravel = distTravel + dist;
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*cos(angle);
                        y = y + dist*sin(angle);
                        disp([x, y, angle, dist]);
                        SetFwdVelRadiusRoomba(serPort, 0.2, inf);
                    end
                end
            end
            % first bumped check
        end
        pause(time_step);
    end
end

