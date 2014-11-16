function map_converter(file_goal, file_obstacles)
    f = fopen(file_goal);
    sizeA = [2 Inf];
    A = fscanf(f, '%f %f',sizeA);
    fclose(f);
    
    f = fopen(file_obstacles);
    num_obstacles = fscanf(f, '%d', 1);
    
    obs(num_obstacles) = struct();
    
    for i = 1:num_obstacles
        num_vertices = fscanf(f, '%d', 1);
        obs(i).num_vertices = num_vertices;
        
        sizeV = [2 obs(i).num_vertices];
        v = fscanf(f, '%f %f',sizeV);
        obs(i).vertices = v';
    end
    fclose(f);
    
    f = fopen('obs_map.txt', 'w');
    
    for j = 1:num_obstacles
        for k = 1:obs(j).num_vertices
            
            if k == obs(j).num_vertices
                fprintf(f, 'wall %.3f %.3f %.3f %.3f\n', obs(j).vertices(k,:), obs(j).vertices(1,:));
            else
                fprintf(f, 'wall %.3f %.3f %.3f %.3f\n', obs(j).vertices(k,:), obs(j).vertices(k+1,:));
            end
            
        end
    end
    
    fclose(f);
end