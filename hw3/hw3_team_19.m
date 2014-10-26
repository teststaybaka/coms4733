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
%	Homework Assignment 3
%	Using the iRobot Create Matlab Simulator and the iRobot Create
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw3_team_19(serPort)
    global time_step
    global DIST_CORRECTION
    
    try
        arg_check = strcmp(serPort.type, 'serial');
        
        if arg_check
            fprintf('Running on Roomba.\n');
            flag = 1;
            accept_error = 0.03;

            turn_velocity = 0.03;
            forward_velocity = 0.05;
            forward_limit = 10;
            step_limit = 2;
            time_step = 0.1;
            right_search_limit = 5;
            DIST_CORRECTION = 1;
        end
    catch 
        fprintf('Running on simulator.\n');
        flag = 0;
        accept_error = 0.2;

        turn_velocity = 0.4;
        forward_velocity = 0.1;
        forward_limit = 10;
        step_limit = 10;
        time_step = 0.1;
        right_search_limit = 30;
        DIST_CORRECTION = 1.2;
        stop_condition = 2000;
        change_condition = 160;
    end 
    
    global LEN
    global GRID
    global BASE_X
    global BASE_Y
    global NOT_CHANGE_COUNT
    global STOP_COUNT
    global BOUND_LOWER_X
    global BOUND_LOWER_Y
    global BOUND_UPPER_X
    global BOUND_UPPER_Y
    global MARKS
    MARKS = zeros(1, 1);
    LEN = 0.4;
    GRID = zeros(2000, 2000);
    BASE_X = -200;
    BASE_Y = -200;
    [BOUND_LOWER_X, BOUND_LOWER_Y] = grid_index(0, 0);
    BOUND_UPPER_X = BOUND_LOWER_X;
    BOUND_UPPER_Y = BOUND_LOWER_Y;
    NOT_CHANGE_COUNT = 0;
    STOP_COUNT = 0;
    %define state for the state machine
    LINGERING           = 0;
    M_MOVING            = 1;
    N_FORWARDING        = 2;
    N_TURNING           = 3;
    N_STEP              = 4;
    W_FORWARDING     	= 5;
    W_TURNING           = 6;
    W_STEP              = 7;

    current_state  = LINGERING;
    next_state = LINGERING;
    
    forward_count = 0;
    right_search_count = 0;
    
    [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    hasWall = WallSensorReadRoomba(serPort);
    hasWall = 0;
    left = 0;
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    global X
    global Y
    global ANGLE
    X = 0;
    Y = 0;
    ANGLE = 0;
	m_x = 0;
	m_y = 0;
    m_a = 0;
    
    %stop condition
    while STOP_COUNT ~= stop_condition
        pause(time_step);
        
        if ~(abs(m_x - X) < 2*accept_error && abs(m_y - Y) < 2*accept_error)
            left = 1;
        end
                
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
                
        switch current_state
            
            case LINGERING
                if bumped
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    forward_count = 0;
                    if occupied(X, Y, BumpRight, BumpLeft, BumpFront) || occupied(X, Y, 0, 0, 0)
                        detemineBounding();
                        if withinBound(X, Y) && ~occupied(X, Y, 0, 0, 0);
                            update_marks(X, Y);
                            [t_x, t_y, find] = pick_a_target(X, Y);
                            if find
                                turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                                next_state = M_MOVING;
                            else
                                break;
                            end
                        else
                            [t_x, t_y, find] = pick_a_target(X, Y);
                            if find
                                turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                                next_state = M_MOVING;
                            elseif BumpRight
                                turn_angle(serPort, turn_velocity, 15+rand()*180);
                                next_state = LINGERING;
                            elseif BumpLeft
                                turn_angle(serPort, turn_velocity, 120+rand()*180);
                                next_state = LINGERING;
                            elseif BumpFront
                                turn_angle(serPort, turn_velocity, 90+rand()*180);
                                next_state = LINGERING;
                            end
                        end
                    else
                        m_x = X;
                        m_y = Y;
                        m_a = ANGLE;
                        bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                        left = 0;
                        next_state = W_FORWARDING;
                    end
                elseif left && occupied(X, Y, 0, 0, 0)
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    forward_count = 0;
                    m_x = X;
                    m_y = Y;
                    m_a = ANGLE;
                    left = 0;
                    [t_x, t_y, find] = pick_a_target(X, Y);
                    if find
                        turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                        next_state = M_MOVING;
                    else
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = LINGERING;
                    end
                elseif NOT_CHANGE_COUNT > change_condition
                    NOT_CHANGE_COUNT = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    forward_count = 0;
                    detemineBounding();
                    if withinBound(X, Y) && ~occupied(X, Y, 0, 0, 0);
                        update_marks(X, Y);
                        [t_x, t_y, find] = pick_a_target(X, Y);
                        if find
                            turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                            next_state = M_MOVING;
                        else
                            break;
                        end
                    else
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = LINGERING;
                    end
                elseif forward_count > forward_limit
                    forward_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    update_grid(X, Y, 2);
                    next_state = LINGERING;
                else
                    forward_count = forward_count + 1;
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                    update_grid(X, Y, 2);
                    next_state = LINGERING;
                end
                fprintf('LINGERING : ');
                print_status(m_x, m_y, m_a);
                
            case W_FORWARDING
                if bumped
                    bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                    forward_count = 0;
                    next_state = W_FORWARDING;
                elseif left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error
                    fprintf('Wall Following finished.\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
                    detemineBounding();
                    if withinBound(X, Y) && ~occupied(X, Y, 0, 0, 0);
                        update_marks(X, Y);
                        [t_x, t_y, find] = pick_a_target(X, Y);
                        if find
                            turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                            next_state = M_MOVING;
                        else
                            break;
                        end
                    else
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = LINGERING;
                    end
                elseif forward_count > forward_limit
					SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
					next_state = W_TURNING;
                else
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    next_state = W_FORWARDING;
                end
                fprintf('W_FORWARDING : ');
                print_status(m_x, m_y, m_a);
                
            case W_TURNING
                hasWall = WallSensorReadRoomba(serPort);
                if hasWall
                    right_search_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    next_state = W_FORWARDING;
                elseif right_search_count > right_search_limit
                    right_search_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    next_state = W_STEP;
                else
                    fprintf('right count:%d %d\n', right_search_count, right_search_limit)
                    SetFwdVelAngVelCreate(serPort, 0, -turn_velocity);
                    right_search_count = right_search_count + 1;
                    next_state = W_TURNING;
                end
                fprintf('W_TURNING : ');
                print_status(m_x, m_y, m_a);
                
            case W_STEP
                if bumped
                    bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                    forward_count = 0;
                    next_state = W_FORWARDING;
                elseif left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error
                    fprintf('Wall Following finished.\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
                    detemineBounding();
                    if withinBound(X, Y) && ~occupied(X, Y, 0, 0, 0);
                        update_marks(X, Y);
                        [t_x, t_y, find] = pick_a_target(X, Y);
                        if find
                            turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                            next_state = M_MOVING;
                        else
                            break;
                        end
                    else
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = LINGERING;
                    end
                elseif forward_count > step_limit
					SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
					next_state = W_TURNING;
                else
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    next_state = W_STEP;
                end
                fprintf('W_STEP : ');
                print_status(m_x, m_y, m_a);
                
            case M_MOVING
                if bumped
                    bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                    m_x = X;
                    m_y = Y;
                    forward_count = 0;
                    left = 0;
                    next_state = N_FORWARDING;
                elseif abs(X - t_x) < accept_error && abs(Y - t_y) < accept_error
                    forward_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    next_state = LINGERING;
                elseif forward_count > forward_limit
                    forward_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    update_grid(X, Y, 2);
                    next_state = M_MOVING;
                else
                    forward_count = forward_count + 1;
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                    update_grid(X, Y, 2);
                    next_state = M_MOVING;
                end
                fprintf('M_MOVING : ');
                print_status(t_x, t_y, m_a);
                
            case N_FORWARDING
                if bumped
                    bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                    forward_count = 0;
                    next_state = N_FORWARDING;
                elseif left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error
                    fprintf('Target unreachable. %.4f %.4f\n', t_x, t_y);
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
                    detemineBounding();
                    if withinBound(X, Y) && ~occupied(X, Y, 0, 0, 0);
                        update_marks(X, Y);
                        [t_x, t_y, find] = pick_a_target(X, Y);
                        if find
                            turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                            m_x = X;
                            m_y = Y;
                            m_a = ANGLE;
                            next_state = M_MOVING;
                        else
                            break;
                        end
                    else
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = LINGERING;
                    end
                elseif left && abs(atan2(t_y-m_y, t_x-m_x) - atan2(t_y-Y, t_x-X)) < accept_error/2 && (abs(t_x - m_x) - accept_error > abs(t_x - X) || abs(t_y - m_y) - accept_error > abs(t_y - Y))
                    SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    forward_count = 0;
                    fprintf('change to m_moving: (%.4f %.4f) (%.4f %.4f) (%.4f %.4f) a:%.4f\n', X, Y, m_x, m_y, t_x, t_y, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                    turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                    next_state = M_MOVING;
                elseif forward_count > forward_limit
					SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
					next_state = N_TURNING;
                else
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    update_grid(X+LEN/2*sin(ANGLE), Y-LEN/2*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    next_state = N_FORWARDING;
                end
                fprintf('N_FORWARDING : ');
                print_status(m_x, m_y, m_a);
                
            case N_TURNING
                hasWall = WallSensorReadRoomba(serPort);
                if hasWall
                    right_search_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    next_state = N_FORWARDING;
                elseif right_search_count > right_search_limit
                    right_search_count = 0;
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    next_state = N_STEP;
                else
                    fprintf('right count:%d %d\n', right_search_count, right_search_limit)
                    SetFwdVelAngVelCreate(serPort, 0, -turn_velocity);
                    right_search_count = right_search_count + 1;
                    next_state = N_TURNING;
                end
                fprintf('N_TURNING : ');
                print_status(m_x, m_y, m_a);
                
            case N_STEP
                if bumped
                    bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                    forward_count = 0;
                    next_state = N_FORWARDING;
                elseif left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error
                    fprintf('Target unreachable.\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X+LEN*sin(ANGLE), Y-LEN*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
                    detemineBounding();
                    if withinBound(X, Y) && ~occupied(X, Y, 0, 0, 0);
                        update_marks(X, Y);
                        [t_x, t_y, find] = pick_a_target(X, Y);
                        if find
                            turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                            m_x = X;
                            m_y = Y;
                            m_a = ANGLE;
                            next_state = M_MOVING;
                        else
                            break;
                        end
                    else
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = LINGERING;
                    end
                elseif left && abs(atan2(t_y-m_y, t_x-m_x) - atan2(t_y-Y, t_x-X)) < accept_error/2 && (abs(t_x - m_x) - accept_error > abs(t_x - X) || abs(t_y - m_y) - accept_error > abs(t_y - Y))
                    SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    forward_count = 0;
                    turn_angle(serPort, turn_velocity, -ANGLE/2/pi*360+atan2(t_y - Y, t_x - X)/2/pi*360);
                    next_state = M_MOVING;
                elseif forward_count > step_limit
					SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    update_grid(X+LEN*sin(ANGLE), Y-LEN*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    forward_count = 0;
					next_state = N_TURNING;
                else
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    update_grid(X+LEN*sin(ANGLE), Y-LEN*cos(ANGLE), 1);
                    update_grid(X, Y, 2);
                    next_state = N_STEP;
                end
                fprintf('N_STEP : ');
                print_status(m_x, m_y, m_a);
                
            otherwise
                fprintf('ERR: Should not end up here.');
        end
        
        % update current state with next state
        current_state = next_state;
    end %end of while loop
    
	SetFwdVelAngVelCreate(serPort, 0, 0);
	fprintf('navigation over!\n');
end

function calculate_coord(serPort)
    global X
    global Y
    global ANGLE
    global DIST_CORRECTION
    dist = DistanceSensorRoomba(serPort)*DIST_CORRECTION;
    a = AngleSensorRoomba(serPort);
    ANGLE = ANGLE + a;
    
	while ANGLE > pi
		ANGLE = ANGLE - 2*pi;
	end
	while ANGLE < -pi
		ANGLE = ANGLE + 2*pi;
	end
	X = X + dist*cos(ANGLE);
	Y = Y + dist*sin(ANGLE);
end

function bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront)
	fprintf('bumped\n');
	
	SetFwdVelAngVelCreate(serPort, 0, 0);
    calculate_coord(serPort);
	
    if BumpRight
        turn_angle(serPort, turn_velocity, 15);
    elseif BumpLeft
        turn_angle(serPort, turn_velocity, 120);
    elseif BumpFront
        turn_angle(serPort, turn_velocity, 90);
    end
end

function turn_angle(serPort, turn_velocity, angle)
    global ANGLE
    global time_step
    fprintf('turning %.4f degress', angle);
    a = 0;
    reverse = 1;
    while angle > 360
        angle = angle - 360;
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
    ANGLE = ANGLE + a;
end

function print_status(m_x, m_y, m_a)
    global X
    global Y
    global ANGLE
    global NOT_CHANGE_COUNT
    global STOP_COUNT
    global BOUND_LOWER_X
    global BOUND_LOWER_Y
    global BOUND_UPPER_X
    global BOUND_UPPER_Y
    fprintf('(%.4f, %.4f), ANGLE = %.4f, not_change_count: %d; (%.4f, %.4f), a = %.4f\n\n', X, Y, ANGLE, NOT_CHANGE_COUNT, m_x, m_y, m_a);
    % fprintf('(%.4f, %.4f), ANGLE = %.4f, %d %d %d %d\n\n', X, Y, ANGLE, BOUND_LOWER_X, BOUND_LOWER_Y, BOUND_UPPER_X, BOUND_UPPER_Y);
end

function [x_i, y_i] = grid_index(x, y)
    global LEN
    global BASE_X
    global BASE_Y
    x_i = floor((x - BASE_X)/LEN) + 1;
    y_i = floor((y - BASE_Y)/LEN) + 1;
end

function detemineBounding()
    global BOUND_LOWER_X
    global BOUND_LOWER_Y
    global BOUND_UPPER_X
    global BOUND_UPPER_Y
    global GRID
    for i = 1:2000
        for j = 1:2000
            if GRID(j, i) == 1
                if i < BOUND_LOWER_X
                    BOUND_LOWER_X = i;
                elseif i > BOUND_UPPER_X
                    BOUND_UPPER_X = i;
                end
                if j < BOUND_LOWER_Y
                    BOUND_LOWER_Y = j;
                elseif j > BOUND_UPPER_Y
                    BOUND_UPPER_Y = j;
                end
            end
        end
    end
    fprintf('%d %d %d %d\n\n', BOUND_LOWER_X, BOUND_LOWER_Y, BOUND_UPPER_X, BOUND_UPPER_Y);
end

function is = withinBound(x, y)
    global BOUND_LOWER_X
    global BOUND_LOWER_Y
    global BOUND_UPPER_X
    global BOUND_UPPER_Y
    [x_i, y_i] = grid_index(x, y);
    if x_i > BOUND_UPPER_X || x_i < BOUND_LOWER_X || y_i > BOUND_UPPER_Y || y_i < BOUND_LOWER_Y
        is = 0;
    else
        is = 1;
    end
end

function update_marks(r_x, r_y)
    import java.util.LinkedList
    global BOUND_LOWER_X
    global BOUND_LOWER_Y
    global BOUND_UPPER_X
    global BOUND_UPPER_Y
    global GRID
    global MARKS
    [r_x_i, r_y_i] = grid_index(r_x, r_y);
    height = BOUND_UPPER_Y - BOUND_LOWER_Y + 1;
    width = BOUND_UPPER_X - BOUND_LOWER_X + 1;
    r_x_i = r_x_i - BOUND_LOWER_X + 1;
    r_y_i = r_y_i - BOUND_LOWER_Y + 1;
    q = LinkedList();
    MARKS = zeros(height, width);
    
    if r_x_i - 1 > 0 && GRID(r_y_i + BOUND_LOWER_Y - 1, r_x_i - 1 + BOUND_LOWER_X - 1) ~= 1
        q.add([r_y_i, r_x_i - 1]);
        MARKS(r_y_i, r_x_i - 1) = 1;
    end
    if r_x_i + 1 <= width && GRID(r_y_i + BOUND_LOWER_Y - 1, r_x_i + 1 + BOUND_LOWER_X - 1) ~= 1
        q.add([r_y_i, r_x_i + 1]);
        MARKS(r_y_i, r_x_i + 1) = 1;
    end
    if r_y_i - 1 > 0 && GRID(r_y_i - 1 + BOUND_LOWER_Y - 1, r_x_i + BOUND_LOWER_X - 1) ~= 1
        q.add([r_y_i - 1, r_x_i]);
        MARKS(r_y_i - 1, r_x_i) = 1;
    end
    if r_y_i + 1 <= height && GRID(r_y_i + 1 + BOUND_LOWER_Y - 1, r_x_i + BOUND_LOWER_X - 1) ~= 1
        q.add([r_y_i + 1, r_x_i]);
        MARKS(r_y_i + 1, r_x_i) = 1;
    end
    
    while q.size() ~= 0
        yxi = q.pollFirst();
        if yxi(2) - 1 > 0 && GRID(yxi(1) + BOUND_LOWER_Y - 1, yxi(2) - 1 + BOUND_LOWER_X - 1) ~= 1 && MARKS(yxi(1), yxi(2) - 1) ~= 1 && ~inQueue(q, yxi(1), yxi(2) - 1)
            q.add([yxi(1), yxi(2) - 1]);
            MARKS(yxi(1), yxi(2) - 1) = 1;
        end
        if yxi(2) + 1 <= width && GRID(yxi(1) + BOUND_LOWER_Y - 1, yxi(2) + 1 + BOUND_LOWER_X - 1) ~= 1 && MARKS(yxi(1), yxi(2) + 1) ~= 1 && ~inQueue(q, yxi(1), yxi(2) + 1)
            q.add([yxi(1), yxi(2) + 1]);
            MARKS(yxi(1), yxi(2) + 1) = 1;
        end
        if yxi(1) - 1 > 0 && GRID(yxi(1) - 1 + BOUND_LOWER_Y - 1, yxi(2) + BOUND_LOWER_X - 1) ~= 1 && MARKS(yxi(1) - 1, yxi(2)) ~= 1 && ~inQueue(q, yxi(1) - 1, yxi(2))
            q.add([yxi(1) - 1, yxi(2)]);
            MARKS(yxi(1) - 1, yxi(2)) = 1;
        end
        if yxi(1) + 1 <= height && GRID(yxi(1) + 1 + BOUND_LOWER_Y - 1, yxi(2) + BOUND_LOWER_X - 1) ~= 1 && MARKS(yxi(1) + 1, yxi(2)) ~= 1 && ~inQueue(q, yxi(1) + 1, yxi(2))
            q.add([yxi(1) + 1, yxi(2)]);
            MARKS(yxi(1) + 1, yxi(2)) = 1;
        end
    end
end

function [t_x, t_y, find] = pick_a_target(r_x, r_y)
    import java.util.LinkedList
    global BOUND_LOWER_X
    global BOUND_LOWER_Y
    global BOUND_UPPER_X
    global BOUND_UPPER_Y
    global GRID
    global LEN
    global BASE_X
    global BASE_Y
    global ANGLE
    global MARKS
    dist_weight = 0.5;
    angle_weight = 1.0;
    [r_x_i, r_y_i] = grid_index(r_x, r_y);
    find = 0;
    t_x = r_x;
    t_y = r_y;
    r_x_i = r_x_i - BOUND_LOWER_X + 1;
    r_y_i = r_y_i - BOUND_LOWER_Y + 1;
    max = -10000000;
    
    for j = BOUND_LOWER_Y:BOUND_UPPER_Y
        for i = BOUND_LOWER_X:BOUND_UPPER_X
            tt_x = BASE_X + i*LEN - LEN/2;
            tt_y = BASE_Y + j*LEN - LEN/2;
            if GRID(j, i) == 0 && MARKS(j - BOUND_LOWER_Y + 1, i - BOUND_LOWER_X + 1) == 1 ...
               && (abs(r_x_i-i) + abs(r_y_i-j))*dist_weight - abs(atan2(tt_y-r_y, tt_x-r_x) - ANGLE)/2/pi*360*angle_weight > max
                max = (abs(r_x_i-i) + abs(r_y_i-j))*dist_weight - abs(atan2(tt_y-r_y, tt_x-r_x) - ANGLE)/2/pi*360*angle_weight;
                t_x = tt_x;
                t_y = tt_y;
                find = 1;
            elseif GRID(j, i) == 0 && MARKS(j - BOUND_LOWER_Y + 1, i - BOUND_LOWER_X + 1) == 0
                GRID(j, i) = 3;
                draw_grid(i, j, [0, 1, 0]);
            end
        end
    end
    
    fprintf('picking target. %.4f %.4f\n', t_x, t_y);
end

function has = inQueue(q, y_i, x_i)
    has = 0;
    for i = 0:q.size()-1
        yxi = q.get(i);
        if yxi(1) == y_i && yxi(2) == x_i
            has = 1;
        end
    end
end

function is = occupied(x, y, right, left, front)
    global GRID
    global LEN
    global ANGLE
    if left || right || front
        if left
            a = ANGLE+120/360*2*pi;
        elseif right
            a = ANGLE+15/360*2*pi;
        else % front
            a = ANGLE+90/360*2*pi;
        end
        x = x+LEN*sin(a);
        y = y-LEN*cos(a);
        [x_i, y_i] = grid_index(x, y);
        if GRID(y_i, x_i) == 1 || GRID(y_i, x_i) == 3
            is = 1;
        else
            is = 0;
        end
    else
        [x_i, y_i] = grid_index(x, y);
        if GRID(y_i, x_i) == 1 || GRID(y_i, x_i) == 3
            is = 1;
        else
            is = 0;
        end
    end
end

function update_grid(x, y, value)
    global GRID
    global NOT_CHANGE_COUNT
    global STOP_COUNT
    [x_i, y_i] = grid_index(x, y);
    if GRID(y_i, x_i) ~= 1 && GRID(y_i, x_i) ~= 3 && GRID(y_i, x_i) ~= value
        GRID(y_i, x_i) = value;
        figure(2);
        if value == 1
            draw_grid(x_i, y_i, [1, 0, 0]);
        else
            draw_grid(x_i, y_i, [0, 0, 1]);
        end
        axis square;
        NOT_CHANGE_COUNT = 0;
        STOP_COUNT = 0;
    else
        NOT_CHANGE_COUNT = NOT_CHANGE_COUNT + 1;
        STOP_COUNT = STOP_COUNT + 1;
    end
end

function draw_grid(x_i, y_i, c)
    global LEN
    global BASE_X
    global BASE_Y
    xs = [(x_i-1)*LEN + BASE_X, (x_i-1)*LEN + BASE_X, x_i*LEN + BASE_X, x_i*LEN + BASE_X];
    ys = [(y_i-1)*LEN + BASE_Y, y_i*LEN + BASE_Y, y_i*LEN + BASE_Y, (y_i-1)*LEN + BASE_Y];
    figure(2)
    patch(xs, ys, c);
end
