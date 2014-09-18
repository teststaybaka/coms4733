function finalRad = hw1_sim(serPort)
    
    maxDuration = 1200;
    time = tic;
    firstBumped = false;
    % START!!!
    SetFwdVelRadiusRoomba(serPort, 0.2, inf);
    count = 0;
    current_state = 0;
    
    while toc(time) < maxDuration
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
     
        bumped = BumpRight || BumpLeft || BumpFront;
        
        if ~firstBumped
            if bumped
                firstBumped = true;
                x = 0;
                y = 0;
                angle = 0;
                turnAngle(serPort, 0.1, 10);
            end
        else
            if current_state == 0
                if bumped
                    count = 0;
                    dist = DistanceSensorRoomba(serPort);
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*sin(angle);
                    y = y + dist*cos(angle);
                    disp([x, y, dist, angle]);
                    turnAngle(serPort, 0.1, 10);
                else
                    if count > 0.5
                        count = 0;
                        current_state = 1;
                        dist = DistanceSensorRoomba(serPort);
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*sin(angle);
                        y = y + dist*cos(angle);
                        disp([x, y, dist, angle]);
                        turnAngle(serPort, 0.1, -10);
                    else
                        count = count + 0.1;
                        dist = DistanceSensorRoomba(serPort);
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*sin(angle);
                        y = y + dist*cos(angle);
                        disp([x, y, dist, angle]);
                        SetFwdVelRadiusRoomba(serPort, 0.2, inf);
                    end
                end
            else
                if bumped
                    current_state = 0;
                    count = 0;
                    dist = DistanceSensorRoomba(serPort);
                    angle = angle + AngleSensorRoomba(serPort);
                    x = x + dist*sin(angle);
                    y = y + dist*cos(angle);
                    disp([x, y, dist, angle]);
                    turnAngle(serPort, 0.1, 10);
                else
                    if count > 0.2
                        count = 0;
                        current_state = 1;
                        dist = DistanceSensorRoomba(serPort);
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*sin(angle);
                        y = y + dist*cos(angle);
                        disp([x, y, dist, angle]);
                        turnAngle(serPort, 0.1, -10);
                    else
                        count = count + 0.1;
                        dist = DistanceSensorRoomba(serPort);
                        angle = angle + AngleSensorRoomba(serPort);
                        x = x + dist*sin(angle);
                        y = y + dist*cos(angle);
                        disp([x, y, dist, angle]);
                        SetFwdVelRadiusRoomba(serPort, 0.2, inf);
                    end
                end
            end
            
        end
        pause(0.1);
    end
end

