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
    x = uint32(x);
    y = uint32(y);
    im_hsl = rgb2hsl(smim);
    figure(2)
    imshow(uint8(im_hsl(:,:,1)*255))

    threshold = 0.05;
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
        [im_label, index] = blob_find(im_hsl, H_min, H_max);
        
        ch = 0;
        cw = 0;
        total_num = 0.0;
        max_width = 0;
        for i = 1:size(im, 1)
            max_j = 0;
            min_j = size(im, 2);
            for j = 1:size(im, 2)
                if im_label == index
                    ch = ch + i;
                    cw = cw + j;
                    if j < min_j
                        min_j = j;
                    end
                    if j > max_j
                        max_j = j;
                    end
                    total_num = total_num + 1;
                end
            end
            width = max_j - min_j;
            if width > max_width
                max_width = width;
            end
        end
        ch = ch/total_num;
        cw = cw/total_num;

        if ch > 1 && ch < size(im, 1) && cw > 1 && cw < size(im, 2)
            smim(ch, cw, :) = [0,0,0];
            smim(ch, cw-1, :) = [0,0,0];
            smim(ch-1, cw, :) = [0,0,0];
            smim(ch, cw+1, :) = [0,0,0];
            smim(ch+1, cw, :) = [0,0,0];
        end
        figure(3);
        imshow(uint8(smim));
        
        angle = (double(size(im, 2)/2) - double(cw))*turn_k;
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if BumpRight
            turnAngle(serPort, turn_velocity, 15);
            forward_paces(serPort, forward_velocity, 20);
            turnAngle(serPort, turn_velocity, -15);
        elseif BumpLeft
            turnAngle(serPort, turn_velocity, -15);
            forward_paces(serPort, forward_velocity, 20);
            turnAngle(serPort, turn_velocity, 15);
        elseif max_width > size(im, 2) - size(im, 2)*margin_w && BumpFront
            break
        elseif BumpFront
            turnAngle(serPort, turn_velocity, 90);
            forward_paces(serPort, forward_velocity, 20);
            turnAngle(serPort, turn_velocity, -90);
        elseif cw == 0
            turnAngle(serPort, turn_velocity, 180*rand() - 90);
            forward_paces(serPort, forward_velocity, 16*rand() + 2);
        elseif abs(angle) > angle_error
            angle
            turnAngle(serPort, turn_velocity, angle);
        else
            fprintf('not supposed to be here\n');
            forward_paces(serPort, forward_velocity, 1);
        end
    end
    
    %knock the door
    state = 0;
    count = 0;
    while 1
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
        if state == 0
            if bumped
                SetFwdVelAngVelCreate(serPort, 0, 0);
                BeepRoomba(serPort);
                state = 1;
            else
                SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                pause(time_step);
            end
        else
            forward_paces(serPort, -forward_velocity, 40);
            state = 0;
            count = count + 1;
        end
        if count > 3
            break
        end
    end
    
    %check if it's opened
    while 1
        im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
        for i = 1:3
            smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
        end
        im_hsl = rgb2hsl(smim);
        [im_label, index] = blob_find(im_hsl, H_min, H_max);
        
        ch = 0;
        cw = 0;
        total_num = 0.0;
        max_width = 0;
        for i = 1:size(im, 1)
            max_j = 0;
            min_j = size(im, 2);
            for j = 1:size(im, 2)
                if im_label == index
                    ch = ch + i;
                    cw = cw + j;
                    if j < min_j
                        min_j = j;
                    end
                    if j > max_j
                        max_j = j;
                    end
                    total_num = total_num + 1;
                end
            end
            width = max_j - min_j;
            if width > max_width
                max_width = width;
            end
        end
        ch = ch/total_num;
        cw = cw/total_num;

        if ch > 1 && ch < size(im, 1) && cw > 1 && cw < size(im, 2)
            smim(ch, cw, :) = [0,0,0];
            smim(ch, cw-1, :) = [0,0,0];
            smim(ch-1, cw, :) = [0,0,0];
            smim(ch, cw+1, :) = [0,0,0];
            smim(ch+1, cw, :) = [0,0,0];
        end
        figure(3);
        imshow(uint8(smim));
        
        if max_width < size(im, 2)/3.0*2 && ...
          (cw < size(im, 2)/2 - size(im, 2)*margin_w || cw > size(im, 2)/2 + size(im, 2)*margin_w)
            break
        end
    end
    
    %enter the room
    count = 0;
    while 1
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if BumpRight
            turnAngle(serPort, turn_velocity, 15);
        elseif BumpLeft
            turnAngle(serPort, turn_velocity, -15);
        elseif BumpFront
            turnAngle(serPort, turn_velocity, 90);
        else
            SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
            count = count + 1;
        end
            
        if count > 80
            break
        end
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
end
