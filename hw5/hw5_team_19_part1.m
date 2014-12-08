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
       
            
%             SetFwdVelAngVelCreate(serPort, 0, -turn);
%         elseif max_height > ori_height + size(im, 1)*margin_h
%             forward = (max_height - ori_height)*forward_k*forward_velocity;
%             SetFwdVelAngVelCreate(serPort, -forward, 0);
%         elseif max_height < ori_height - size(im, 1)*margin_h
%             forward = (ori_height - max_height)*forward_k*forward_velocity;
%             SetFwdVelAngVelCreate(serPort, forward, 0);

%         pause(time_step);
%         SetFwdVelAngVelCreate(serPort, 0, 0);
    end
end

function [im_label, index] = blob_find(im_hsl, H_min, H_max)
    im_label = zeros(size(im_hsl, 1), size(im_hsl, 2));
    k = 1;
    equivalent_pair = [];
    for j = 1:size(im_hsl, 2)
        for i = 1:size(im_hsl, 1)
            if im_hsl(i, j, 1) > H_min && im_hsl(i, j, 1) < H_max
                if i-1 > 0 && j-1 > 0
                    if im_label(i-1, j) ~= 0 && im_label(i, j-1) == 0
                        im_label(i, j) = im_label(i-1, j);
                    end
                    if im_label(i, j-1) ~= 0 && im_label(i-1, j) == 0
                        im_label(i, j) = im_label(i, j-1);
                    end
                    if im_label(i, j-1) ~= 0 && im_label(i-1, j) ~= 0
                        im_label(i, j) = im_label(i, j-1);
                        if im_label(i, j-1) < im_label(i-1, j)
                            s = im_label(i, j-1);
                            l = im_label(i-1, j);
                        else
                            s = im_label(i-1, j);
                            l = im_label(i, j-1);
                        end
                        m = equivalent_pair(s);
                        count = 0;
                        while l ~= equivalent_pair(l)
                            l = equivalent_pair(l);
                            count = count + 1;
                        end
    %                     count
                        assert(count < 2);
                        if m ~= l
                            assert(s ~= l);
                            if m > l
                                t = m;
                                m = l;
                                l = t;
                            end
                            for z = 1:size(equivalent_pair, 2)
                                if equivalent_pair(z) == m
                                    equivalent_pair(z) = l;
                                end
                            end
                            equivalent_pair(m) = l;
    %                         equivalent_pair
    %                         equivalent_pair(m)
                        end
                    end
                    if im_label(i, j-1) == 0 && im_label(i-1, j) == 0
                        im_label(i, j) = k;
                        equivalent_pair(k) = k;
                        k = k + 1;
                    end
                elseif i-1 > 0 && j-1 <= 0
                    if im_label(i-1, j) ~= 0
                        im_label(i, j) = im_label(i-1, j);
                    end
                    if im_label(i-1, j) == 0
                        im_label(i, j) = k;
                        equivalent_pair(k) = k;
                        k = k + 1;
                    end
                elseif i-1 <= 0 && j-1 > 0
                    if im_label(i, j-1) ~= 0
                        im_label(i, j) = im_label(i, j-1);
                    end
                    if im_label(i, j-1) == 0
                        im_label(i, j) = k;
                        equivalent_pair(k) = k;
                        k = k + 1;
                    end
                else
                    im_label(i, j) = k;
                    equivalent_pair(k) = k;
                    k = k + 1;
                end
            end
        end
    %     figure(4);
    %     k
    %     imshow(uint8(im_label*255.0/(k-1)));
    %     figure(4);
    %     k
    %     imshow(uint8(im_label*255.0/(k-1)));
    end
    % equivalent_pair
    % ch = ch/total_num;
    % cw = cw/total_num;

    blob_size = zeros(1, k);
    for j = 1:size(im_hsl, 2)
        for i = 1:size(im_hsl, 1)
            if im_label(i, j) ~= 0
                im_label(i, j) = equivalent_pair(im_label(i, j));
                blob_size(im_label(i, j)) = blob_size(im_label(i, j)) + 1;
            end
        end
    end
    figure(4);
    k
    imshow(uint8(im_label*255.0/(k-1)));

    [c, index] = max(blob_size);
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

function forward_paces(serPort, forward_velocity, paces)
    global time_step
    for i = 1:paces
        
    SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
        pause(time_step);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
end
