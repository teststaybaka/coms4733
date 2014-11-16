function hw4_team_19(serPort)
    global X
    global Y
    global ANGLE
    global time_step
    time_step = 0.1;
    
    turn_velocity = 0.1;
    accept_dist_error = 0.1;
    accept_angle_error = 0.05;
    forward_velocity = 0.1;
    
    f = fopen('route.res');
    sizeA = [2 Inf];
    A = fscanf(f, '%f %f',sizeA);
    fclose(f);
    
    len = size(A, 2);
    X = A(1, len);
    Y = A(2, len);
    ANGLE = pi/2;
    rotate_count = 0;
    for i = 1:len-1
        t_x = A(1, len-i);
        t_y = A(2, len-i);
        
        while sqrt((t_x - X)*(t_x - X) + (t_y - Y)*(t_y - Y)) > accept_dist_error
            calculate_coord(serPort);
            t_a = atan2(t_y - Y, t_x - X);
            while abs(ANGLE - t_a) > accept_angle_error
                SetFwdVelAngVelCreate(serPort, 0, 0);
                a = t_a - ANGLE;
                turn_angle(serPort, turn_velocity, a/pi*180);
                rotate_count = rotate_count + 1;
            end
            SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
            pause(time_step);
        end
    end
    rotate_count
end

function calculate_coord(serPort)
    global X
    global Y
    global ANGLE
    dist = DistanceSensorRoomba(serPort);
    a = AngleSensorRoomba(serPort);
    ANGLE = ANGLE + a;
    
	while ANGLE > pi
		ANGLE = ANGLE - 2*pi;
	end
	while ANGLE < -pi
		ANGLE = ANGLE + 2*pi;
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
        angle = angle - 360;
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
