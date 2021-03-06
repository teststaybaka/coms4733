%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS 4733 - Computational Aspect of Robotics
%
%	Columbia University FALL 2014 
%
%	Team    : 19
%	Members : Yuxuan Xie 				        (yx2284)
%             Xusheng Gao 				        (xg2193)
%		      Zachary Hideichi Watanabe-Gastel	(zhw2102)
%
%	Homework Assignment 5
%	Using the iRobot Create Matlab Simulator and the iRobot Create
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    [~, ~, height, ~] = center_point(im_label, index);
    ori_height = height;

    while 1
        im = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
        for i = 1:3
            smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
        end
        im_hsl = rgb2hsl(smim);
        [im_label, index] = blob_find(im_hsl, H_min, H_max);
        [ch, cw, height, ~] = center_point(im_label, index);
        ch = uint32(ch);
        cw = uint32(cw);

        if ch > 1 && ch < size(im, 1) && cw > 1 && cw < size(im, 2)
            smim(ch, cw, :) = [0,0,0];
            smim(ch, cw-1, :) = [0,0,0];
            smim(ch-1, cw, :) = [0,0,0];
            smim(ch, cw+1, :) = [0,0,0];
            smim(ch+1, cw, :) = [0,0,0];
        end
        figure(5)
        imshow(uint8(smim));
        
        angle = (double(size(im, 2)/2) - double(cw))*turn_k
        if abs(angle) > angle_error
            turnAngle(serPort, turn_velocity, angle);
        else
            ori_height
            height
            paces = (double(ori_height) - double(height))*forward_k
            if abs(paces) > 8
                forward_v = (paces / abs(paces)) * forward_velocity;
                paces = abs(paces);
                paces = int32(paces+0.5);
                forward_paces(serPort, forward_v, paces);
            end
        end

    end
end
