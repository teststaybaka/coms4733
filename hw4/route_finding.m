function route_finding(file_goal, file_obstacles)
    f = fopen(file_goal);
    sizeA = [2 Inf];
    A = fscanf(f, '%f %f',sizeA)
    fclose(f);
    
    f = fopen(file_obstacles);
    num_obstacles = fscanf(f, '%d', 1)
    for i = 1:num_obstacles
        num_vertices = fscanf(f, '%d', 1)
    end
    fclose(f);
end