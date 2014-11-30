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
figure(4)
imshow(im)
threshold = 0.02;
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
marks = zeros(size(im, 1), size(im, 2));
res = [];
count = 0;
for i = 1:size(im, 1)
    for j = 1:size(im, 2)
        if (marks(i, j) == 0)
            count = count + 1;
            q = LinkedList();
            q.add([i, j]);
            if (im_hsl(i, j, 1) > H_min && im_hsl(i, j, 1) < H_max)
                kind = 1;
            else
                kind = 0;
            end
            num = 0;
            c_i = 0;
            c_j = 0;
            marks(i, j) = 1;
            while q.size() ~= 0
                ij = q.pollFirst();
                num = num + 1;
                c_i = c_i + ij(1);
                c_j = c_j + ij(2);
                smim(ij(1), ij(2), :) = [255/20*count, 255/20*count, 255/20*count];
                if (ij(1) + 1 <= size(im_hsl, 1) && marks(ij(1) + 1, ij(2)) == 0) && ...
                    ((kind == 1 && im_hsl(ij(1) + 1, ij(2), 1) > H_min && im_hsl(ij(1) + 1, ij(2), 1) < H_max) ...
                    || (kind == 0 && (im_hsl(ij(1) + 1, ij(2), 1) < H_min || im_hsl(ij(1) + 1, ij(2), 1) > H_max)))
                    q.add([ij(1)+1, ij(2)]);
                    marks(ij(1)+1, ij(2)) = 1;
                end
                if (ij(1) - 1 > 0 && marks(ij(1) - 1, ij(2)) == 0) && ...
                    ((kind == 1 && im_hsl(ij(1) - 1, ij(2), 1) > H_min && im_hsl(ij(1) - 1, ij(2), 1) < H_max) ...
                    || (kind == 0 && (im_hsl(ij(1) - 1, ij(2), 1) < H_min || im_hsl(ij(1) - 1, ij(2), 1) > H_max)))
                    q.add([ij(1)-1, ij(2)]);
                    marks(ij(1)-1, ij(2)) = 1;
                end
                if (ij(2) + 1 <= size(im_hsl, 2) && marks(ij(1), ij(2) + 1) == 0) && ...
                    ((kind == 1 && im_hsl(ij(1), ij(2) + 1, 1) > H_min && im_hsl(ij(1), ij(2) + 1, 1) < H_max) ...
                    || (kind == 0 && (im_hsl(ij(1), ij(2) + 1, 1) < H_min || im_hsl(ij(1), ij(2) + 1, 1) > H_max)))
                    q.add([ij(1), ij(2)+1]);
                    marks(ij(1), ij(2)+1) = 1;
                end
                if (ij(2) - 1 > 0 && marks(ij(1), ij(2) - 1) == 0) && ...
                    ((kind == 1 && im_hsl(ij(1), ij(2) - 1, 1) > H_min && im_hsl(ij(1), ij(2) - 1, 1) < H_max) ...
                    || (kind == 0 && (im_hsl(ij(1), ij(2) - 1, 1) < H_min || im_hsl(ij(1), ij(2) - 1, 1) > H_max)))
                    q.add([ij(1), ij(2)-1]);
                    marks(ij(1), ij(2)-1) = 1;
                end
            end
            c_i = c_i/num;
            c_j = c_j/num;
            
            res = [res; c_i, c_j, num, kind];
        end
    end
end
max_index = 0;
max_num = 0;
for i = 1:size(res, 1)
    if res(i, 3) > max_num && res(i, 4) == 1
        max_num = res(i, 3);
        max_index = uint8(i);
    end
end
smim(uint8(res(max_index, 1)), uint8(res(max_index, 2)), :) = [0,0,0];
smim(uint8(res(max_index, 1))+1, uint8(res(max_index, 2)), :) = [0,0,0];
smim(uint8(res(max_index, 1))-1, uint8(res(max_index, 2)), :) = [0,0,0];
smim(uint8(res(max_index, 1)), uint8(res(max_index, 2))+1, :) = [0,0,0];
smim(uint8(res(max_index, 1)), uint8(res(max_index, 2))-1, :) = [0,0,0];
figure(3);
imshow(uint8(smim));
%end
