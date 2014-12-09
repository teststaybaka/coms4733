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
    end
    % equivalent_pair

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
