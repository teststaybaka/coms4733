function hw4_team_19(serPort)
    global X
    global Y
    global ANGLE
    global time_step
    time_step = 0.1;
    
    turn_velocity = 0.075;
    accept_dist_error = 0.05;
    accept_angle_error = 0.05;
    forward_velocity = 0.25;
    
    f = fopen('route.res');
    sizeA = [2 Inf];
    A = fscanf(f, '%f %f',sizeA);
    fclose(f);
    
    len = size(A, 2);
    X = A(1, len);
    Y = A(2, len);
    ANGLE = pi/2.0;
    rotate_count = 0;
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    
    for i = 1:len-1
        t_x = A(1, len-i);
        t_y = A(2, len-i);
        
        fprintf('tx = %f, ty = %f\n', t_x, t_y);
        
        if i == len-1
            t_y = t_y + 0.1;
        end
        
        while sqrt((t_x - X)*(t_x - X) + (t_y - Y)*(t_y - Y)) > accept_dist_error
            t_a = atan2(t_y - Y, t_x - X);
            count = 0;
            while abs(ANGLE - t_a) > accept_angle_error*1/sqrt((t_x - X)*(t_x - X) + (t_y - Y)*(t_y - Y))
                SetFwdVelAngVelCreate(serPort, 0, 0);
                a = t_a - ANGLE;
                %turn_angle(serPort, turn_velocity, a/pi*180);
                turnAngle(serPort, turn_velocity, a/pi*180.0);
                calculate_coord(serPort);
                rotate_count = rotate_count + 1;
                fprintf('turning a = %f\n', ANGLE/pi*180.0);
                count = count + 1;
                
                if count > 2
                    break;
                end
            end
            SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
            pause(time_step);
            calculate_coord(serPort);
        end
        
        fprintf('x = %f, y = %f\n', X, Y);
        
    end
    
    SetFwdVelAngVelCreate(serPort, 0, 0);
   %rotate_count
end

function calculate_coord(serPort)
    global X
    global Y
    global ANGLE
    dist = DistanceSensorRoomba(serPort);
    a = AngleSensorRoomba(serPort);
    ANGLE = ANGLE + a;
    
	while ANGLE > pi
		ANGLE = ANGLE - 2.0*pi;
	end
	while ANGLE < -pi
		ANGLE = ANGLE + 2.0*pi;
	end
	X = X + dist*cos(ANGLE);
	Y = Y + dist*sin(ANGLE);
end

function turn_angle(serPort, turn_velocity, angle)
    global ANGLE
    global time_step
    fprintf('turning %.4f degress', angle);
    a = 0;
    reverse = 1;
    while angle > 360
        angle = angle - 360.0;
    end
    while angle <= 0
        angle = angle + 360;
    end
    if angle > 180
        angle = angle - 360;
        reverse = -1;
    end
    fprintf(', modified %.4f degress\n', angle);
    while abs(a) < abs(angle/360*2*pi)
        SetFwdVelAngVelCreate(serPort, 0, reverse*turn_velocity);
        pause(time_step);
        a = a + AngleSensorRoomba(serPort);
        while a > pi
            a = a - 2*pi;
        end
        while a < -pi
            a = a + 2*pi;
        end
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
    a = a + AngleSensorRoomba(serPort);
    ANGLE = ANGLE + a;
end
