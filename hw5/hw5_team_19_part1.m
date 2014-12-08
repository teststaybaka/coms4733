function hw5_team_19_part1( serPort )
    turn_velocity = 0.075;
    time_step = 0.1;
    forward_velocity = 0.1;
    margin_w = 0.1;
    margin_h = 0.05;
    turn_k = 0.001;
    forward_k = 0.1; 
    
    import java.util.LinkedList
    
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

    max_height = 0;
    for j = 1:size(im, 2)
        height = 0;
        for i = 1:size(im, 1)
            if im_hsl(i, j, 1) > H_min && im_hsl(i, j, 1) < H_max
                height = height + 1;
            end
        end
        if height > max_height
            max_height = height;
        end
    end
    ori_height = max_height;

    while 1
        im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
        for i = 1:3
            smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
        end
        im_hsl = rgb2hsl(smim);
        ch = 0;
        cw = 0;
        total_num = 0.0;
        max_height = 0;
        for j = 1:size(im, 2)
            height = 0;
            for i = 1:size(im, 1)
                if im_hsl(i, j, 1) > H_min && im_hsl(i, j, 1) < H_max
                    ch = ch + i;
                    cw = cw + j;
                    height = height + 1;
                    total_num = total_num + 1;
                end
            end
            if height > max_height
                max_height = height;
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
        
        if cw < size(im, 2)/2 - size(im, 2)*margin_w
            turn = (size(im, 2)/2 - cw)*turn_k*turn_velocity;
            SetFwdVelAngVelCreate(serPort, 0, turn);
        elseif cw > size(im, 2)/2 + size(im, 2)*margin_w
            turn = (cw - size(im, 2)/2)*turn_k*turn_velocity;
            SetFwdVelAngVelCreate(serPort, 0, -turn);
        elseif max_height > ori_height + size(im, 1)*margin_h
            forward = (max_height - ori_height)*forward_k*forward_velocity;
            SetFwdVelAngVelCreate(serPort, -forward, 0);
        elseif max_height < ori_height - size(im, 1)*margin_h
            forward = (ori_height - max_height)*forward_k*forward_velocity;
            SetFwdVelAngVelCreate(serPort, forward, 0);
        end
        pause(time_step);
        SetFwdVelAngVelCreate(serPort, 0, 0);
    end
end
