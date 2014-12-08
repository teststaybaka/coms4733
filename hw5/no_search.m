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
%	Homework Assignment 4
%	Using the iRobot Create Matlab Simulator and the iRobot Create
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import java.util.LinkedList

% im = double(imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd='));
figure(1)
imshow(im)
threshold = 0.02;
for i = 1:3
    smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
end
figure(2)
imshow(uint8(smim));
[x, y] = ginput
x = uint8(x);
y = uint8(y);
im_hsl = rgb2hsl(smim);
figure(3)
imshow(uint8(im_hsl(:,:,1)*255))

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

%while 1
    %im = double(imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd='));
%for i = 1:3
%    smim(:,:,i) = gaussfilt(im(:,:,i), 0.8);
%end
%im_hsl = rgb2hsl(smim);
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
figure(4);
imshow(uint8(smim));
%end
