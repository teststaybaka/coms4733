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
clear
im = double(imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd='));
figure(1)
imshow(uint8(im))
threshold = 0.02;
for i = 1:3
    smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
end
figure(2)
imshow(uint8(smim));
[x, y] = ginput
% x = [124;125;152]
% y = [204;172;192]
x = uint32(x);
y = uint32(y);
im_hsl = rgb2hsl(smim);
figure(3)
imshow(uint8(im_hsl(:,:,1)*255))

H_max = 0;
H_min = 1;
for i = 1:size(x, 1)
    if im_hsl(y(i), x(i), 1) > H_max
        H_max = im_hsl(y(i), x(i), 1)
    end
    if im_hsl(y(i), x(i), 1) < H_min
        H_min = im_hsl(y(i), x(i), 1)
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

%while 1
    %im = double(imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd='));
%for i = 1:3
%    smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
%end
%im_hsl = rgb2hsl(smim);
im_label = zeros(size(im, 1), size(im, 2));
k = 1;
equivalent_pair = [];
for j = 1:size(im, 2)
    for i = 1:size(im, 1)
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
k
% equivalent_pair

blob_size = zeros(1, k);
for j = 1:size(im, 2)
    for i = 1:size(im, 1)
        if im_label(i, j) ~= 0
            im_label(i, j) = equivalent_pair(im_label(i, j));
            blob_size(im_label(i, j)) = blob_size(im_label(i, j)) + 1;
        end
    end
end
figure(4);
k
imshow(uint8(im_label*255.0/(k-1)));

[c, index] = max(blob_size)
ch = 0;
cw = 0;
total_num = 0.0;
max_height = 0;
for j = 1:size(im, 2)
    height = 0;
    for i = 1:size(im, 1)
        if im_label(i, j) == index
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
ch = uint32(ch/total_num)
cw = uint32(cw/total_num)

smim(ch, cw, :) = [0,0,0];
smim(ch, cw-1, :) = [0,0,0];
smim(ch-1, cw, :) = [0,0,0];
smim(ch, cw+1, :) = [0,0,0];
smim(ch+1, cw, :) = [0,0,0];
figure(5)
imshow(uint8(smim));
%end
