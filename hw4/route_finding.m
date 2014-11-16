function route_finding(file_goal, file_obstacles)
    f = fopen(file_goal);
    sizeA = [2 Inf];
    A = fscanf(f, '%f %f',sizeA);
    fclose(f);
    
    f = fopen(file_obstacles);
    num_obstacles = fscanf(f, '%d', 1);
    
    obs(num_obstacles) = struct();
    
    for i = 1:num_obstacles
        num_vertices = fscanf(f, '%d', 1);
        obs(1).num_vertices = num_vertices;
        
        sizeV = [2 obs(1).num_vertices];
        v = fscanf(f, '%f %f',sizeV);
        obs(i).vertices = v';
    end
    fclose(f);
    
    c = 'w';
    figure;
    
    for j = 1:num_obstacles
        h = patch(reshape(obs(j).vertices(:,1), [], 1), reshape(obs(j).vertices(:,2), [], 1), c);
    end
    
    
end