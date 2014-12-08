function hw5_team_19_part2( serPort )
    global time_step
    turn_velocity = 0.075;
    time_step = 0.1;
    forward_velocity = 0.1;
    [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        
    %manually determine the color to track
    im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
    for i = 1:3
        smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
    end
    figure(1)
    imshow(uint8(smim));
    [x, y] = ginput
    x = uint8(x);
    y = uint8(y);
    im_hsl = rgb2hsl(smim);
    figure(2)
    imshow(uint8(im_hsl(:,:,1)*255))

    threshold = 0.02;
    H_max = 0;
    H_min = 1;
    for i = 1:size(x, 1)
        if im_hsl(y(i), x(i), 1) > H_max
            H_max = im_hsl(y(i), x(i), 1);
        end
        if im_hsl(y(i), x(i), 1) < H_min
            H_min = im_hsl(y(i), x(i), 1);
        end
    end
    H_max = H_max + threshold;
    if (H_max > 1)
        H_max = 1;
    end
    H_min = H_min - threshold;
    if (H_min < 0)
        H_min = 0;
    end
    H_max
    H_min

    input('press enter key to start', 's')
    
    %navigate to a door
    while 1
        im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
        for i = 1:3
            smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
        end
        im_hsl = rgb2hsl(smim);
        ch = 0;
        cw = 0;
        total_num = 0.0;
        max_width = 0;
        for i = 1:size(im, 1)
            width = 0;
            for j = 1:size(im, 2)
                if im_hsl(i, j, 1) > H_min && im_hsl(i, j, 1) < H_max
                    ch = ch + i;
                    cw = cw + j;
                    width = width + 1;
                    total_num = total_num + 1;
                end
            end
            if width > max_width
                max_width = width;
            end
        end
        ch = ch/total_num;
        cw = cw/total_num;

        smim(ch, cw, :) = [0,0,0];
        smim(ch, cw-1, :) = [0,0,0];
        smim(ch-1, cw, :) = [0,0,0];
        smim(ch, cw+1, :) = [0,0,0];
        smim(ch+1, cw, :) = [0,0,0];
        figure(3);
        imshow(uint8(smim));
        
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if BumpRight
            turn_angle(serPort, turn_velocity, 15);
            forward_paces(serPort, forward_velocity, 10);
            turn_angle(serPort, turn_velocity, -15);
        elseif BumpLeft
            turn_angle(serPort, turn_velocity, -15);
            forward_paces(serPort, forward_velocity, 10);
            turn_angle(serPort, turn_velocity, 15);
        elseif max_width > size(im, 2) - size(im, 2)*margin_w
            if BumpFront
                break
            else
                turn_angle(serPort, turn_velocity, 90);
                forward_paces(serPort, forward_velocity, 10);
                turn_angle(serPort, turn_velocity, -90);
            end
        elseif cw == 0 && ch == 0
            turn_angle(serPort, turn_velocity, 180*rand() - 90);
            forward_paces(serPort, forward_velocity, 16*rand() + 2);
        elseif cw < size(im, 2)/2 - size(im, 2)*margin_w
            turn = (size(im, 2)/2 - cw)*turn_k*turn_velocity;
            SetFwdVelAngVelCreate(serPort, 0, turn);
            pause(time_step);
            SetFwdVelAngVelCreate(serPort, 0, 0);
        elseif cw > size(im, 2)/2 + size(im, 2)*margin_w
            turn = (cw - size(im, 2)/2)*turn_k*turn_velocity;
            SetFwdVelAngVelCreate(serPort, 0, -turn);
            pause(time_step);
            SetFwdVelAngVelCreate(serPort, 0, 0);
        else
            forward_paces(serPort, forward_velocity, 1);
        end
    end
    
    %knock the door
    
    
    %check if it's opened
    while 1
        im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
        for i = 1:3
            smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
        end
        im_hsl = rgb2hsl(smim);
        ch = 0;
        cw = 0;
        total_num = 0.0;
        max_width = 0;
        for i = 1:size(im, 1)
            width = 0;
            for j = 1:size(im, 2)
                if im_hsl(i, j, 1) > H_min && im_hsl(i, j, 1) < H_max
                    ch = ch + i;
                    cw = cw + j;
                    width = width + 1;
                    total_num = total_num + 1;
                end
            end
            if width > max_width
                max_width = width;
            end
        end
        ch = ch/total_num;
        cw = cw/total_num;

        smim(ch, cw, :) = [0,0,0];
        smim(ch, cw-1, :) = [0,0,0];
        smim(ch-1, cw, :) = [0,0,0];
        smim(ch, cw+1, :) = [0,0,0];
        smim(ch+1, cw, :) = [0,0,0];
        figure(4);
        imshow(uint8(smim));
        
        if max_width < size(im, 2)/3.0*2 && ...
          (cw < size(im, 2)/2 - size(im, 2)*margin_w || cw > size(im, 2)/2 + size(im, 2)*margin_w)
            break
        end
    end
    
    %enter the room
    forward_paces(serPort, forward_velocity, 20);
end

function forward_paces(serPort, forward_velocity, paces)
    global time_step
    SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
    for i = 1:paces
        pause(time_step);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
end

function turn_angle(serPort, turn_velocity, angle)
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
end