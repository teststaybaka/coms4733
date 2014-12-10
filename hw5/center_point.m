function [ ch, cw, max_height, max_width ] = center_point( im_label, index )
    final_max_j = 0;
    final_min_j = 0;
    max_width = 0;
    for i = 1:size(im_label, 1)
        max_j = 0;
        min_j = size(im_label, 2);
        for j = 1:size(im_label, 2)
            if im_label(i, j) == index
                if j < min_j
                    min_j = j;
                end
                if j > max_j
                    max_j = j;
                end
            end
        end
        width = max_j - min_j;
        if width > max_width
            max_width = width;
            final_max_j = max_j;
            final_min_j = min_j;
        end
    end
    cw = (final_max_j + final_min_j)/2;
    
    final_max_i = 0;
    final_min_i = 0;
    max_height = 0;
    for j = 1:size(im_label, 2)
        min_i = size(im_label, 1);
        max_i = 0;
        for i = 1:size(im_label, 1)
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
            final_min_i = min_i;
            final_max_i = max_i;
        end
    end
    ch = (final_max_i + final_min_i)/2;
end

