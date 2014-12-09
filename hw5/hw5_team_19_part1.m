function hw5_team_19_part1( serPort )
    turn_velocity = 0.05;
    time_step = 0.1;
    margin_w = 0.1;
    margin_h = 0.05;
    turn_k = 0.1;
    forward_k = 1.5;
    forward_velocity = 0.1;
    angle_error = 0.7;

    im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
    %figure(1)
    imshow(uint8(im))
    for i = 1:3
        smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
    end
    figure(2)
    imshow(uint8(smim));
    [x, y] = ginput
    x = uint32(x);
    y = uint32(y);
    im_hsl = rgb2hsl(smim);
    %figure(3)
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

    [im_label, index] = blob_find(im_hsl, H_min, H_max);
    max_height = 0;
    for j = 1:size(im, 2)
        min_i = size(im, 1);
        max_i = 0;
        for i = 1:size(im, 1)
            if im_label(i, j) == index
                if i < min_i
                    min_i = i;
                end
                if i > max_i
                    max_i = i;
                end
            end
        end
        height = max_i - min_i;
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
        [im_label, index] = blob_find(im_hsl, H_min, H_max);
        
        ch = 0;
        cw = 0;
        total_num = 0.0;
        max_height = 0;
        for j = 1:size(im, 2)
            min_i = size(im, 1);
            max_i = 0;
            for i = 1:size(im, 1)
                if im_label(i, j) == index
                    ch = ch + i;
                    cw = cw + j;
                    if i < min_i
                        min_i = i;
                    end
                    if i > max_i
                        max_i = i;
                    end
                    total_num = total_num + 1;
                end
            end
            height = max_i - min_i;
            if height > max_height
                max_height = height;
            end
        end
        ch = uint32(ch/total_num);
        cw = uint32(cw/total_num)

        smim(ch, cw, :) = [0,0,0];
        smim(ch, cw-1, :) = [0,0,0];
        smim(ch-1, cw, :) = [0,0,0];
        smim(ch, cw+1, :) = [0,0,0];
        smim(ch+1, cw, :) = [0,0,0];
        figure(5)
        imshow(uint8(smim));
        
        angle = (double(size(im, 2)/2) - double(cw))*turn_k
        if abs(angle) > angle_error
            turnAngle(serPort, turn_velocity, angle);
        else
            ori_height
            max_height
            paces = (double(ori_height) - double(max_height))*forward_k
            if abs(paces) > 8
                forward_v = (paces / abs(paces)) * forward_velocity;
                paces = abs(paces);
                paces = int32(paces+0.5);
                forward_paces(serPort, forward_v, paces);
            end
        end

    end
end
