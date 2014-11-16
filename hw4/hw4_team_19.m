function hw4_team_19()
    global X
    global Y
    global ANGLE
    global time_step
    time_step = 0.1;
    
    f = fopen('route.res');
    sizeA = [2 Inf];
    A = fscanf(f, '%f %f',sizeA);
    fclose(f);
    
    len = size(A, 2);
    X = A(1, len);
    Y = A(2, len);
    ANGLE = pi/2;
    for i = 1:len-1
        t_x = A(1, len-i);
        t_y = A(2, len-i);
        
        atan2(t_y - y, t_x - x);
    end
    
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
