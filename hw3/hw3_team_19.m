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
        accept_error = 0.1;

        turn_velocity = 0.4;
        forward_velocity = 0.2;
        forward_limit = 4;
        step_limit = 4;
        time_step = 0.1;
        right_search_limit = 30;
        DIST_CORRECTION = 1.2;
        stop_condition = 2000;
        change_condition = 40;
    end 
    
    global LEN
    global GRID
    global BASE_X
    global BASE_Y
    global NOT_CHANGE_COUNT
    global STOP_COUNT
    LEN = 0.2;
    GRID = zeros(2000, 2000);
    BASE_X = -200;
    BASE_Y = -200;
    NOT_CHANGE_COUNT = 0;
    STOP_COUNT = 0;
    %define state for the state machine
    LINGERING           = 0;
    W_FORWARDING     	= 1;
    W_TURNING           = 2;
    W_STEP              = 3;

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
    while NOT_CHANGE_COUNT ~= stop_condition
        pause(time_step);
        
        if ~(abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error)
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
                        
                    if occupied(X, Y)
                        
                        if BumpRight
                            turn_angle(serPort, turn_velocity, 15+rand()*180);
                        elseif BumpLeft
                            turn_angle(serPort, turn_velocity, 120+rand()*180);
                        elseif BumpFront
                            turn_angle(serPort, turn_velocity, 90+rand()*180);
                        end
                        next_state = LINGERING;
                        
                    else
                        
                        m_x = X;
                        m_y = Y;
                        m_a = ANGLE;
                        update_grid(X, Y, 1);
                        bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                        left = 0;
                        next_state = W_FORWARDING;
                        
                    end
                    
                elseif NOT_CHANGE_COUNT > change_condition
                    
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    forward_count = 0;
                    NOT_CHANGE_COUNT = 0;
                    turn_angle(serPort, turn_velocity, rand()*360);
                    next_state = LINGERING;
                    
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
                    update_grid(X, Y, 1);
                    next_state = W_FORWARDING;
                    
                elseif NOT_CHANGE_COUNT > change_condition || (left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error)

                    fprintf('start wandering\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X, Y, 1);
                    forward_count = 0;
                    NOT_CHANGE_COUNT = 0;
                    turn_angle(serPort, turn_velocity, rand()*360);
					next_state = LINGERING;
                    
                elseif forward_count > forward_limit
                    
					SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    update_grid(X, Y, 1);
                    forward_count = 0;
					next_state = W_TURNING;
                    
                else
                    
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    update_grid(X, Y, 1);
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
                    update_grid(X, Y, 1);
                    next_state = W_FORWARDING;
                
                elseif NOT_CHANGE_COUNT > change_condition || (left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error)
                    
                    fprintf('start wandering\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X, Y, 1);
                    forward_count = 0;
                    NOT_CHANGE_COUNT = 0;
                    turn_angle(serPort, turn_velocity, rand()*360);
					next_state = LINGERING;

                elseif forward_count > step_limit
                    
                    SetFwdVelAngVelCreate(serPort, 0, 0);
					calculate_coord(serPort);
                    update_grid(X, Y, 1);
                    forward_count = 0;
                    next_state = W_TURNING;
                    
                else
                    
                    calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    update_grid(X, Y, 1);
                    next_state = W_STEP;
                    
                end
                
                fprintf('W_STEP : ');
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

function turn_angle(serPort, turn_velocity, angle) %angle should always be positive
    global ANGLE
    global time_step
    fprintf('turning %.4f degress\n', angle);
    
    a = 0;
    reverse = 1;
    if angle > 180
        angle = angle - 360;
        reverse = -1;
    end
    while abs(a) < abs(angle/360*2*pi)
        SetFwdVelAngVelCreate(serPort, 0, reverse*turn_velocity);
        pause(time_step);
        a = a + AngleSensorRoomba(serPort);
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
    fprintf('(%.4f, %.4f), ANGLE = %.4f, not_change_count: %d, stop_count: %d, m:(%.4f, %.4f), a = %.4f\n\n', X, Y, ANGLE, NOT_CHANGE_COUNT, STOP_COUNT, m_x, m_y, m_a);
end

function [x_i, y_i] = grid_index(x, y)
    global LEN
    global BASE_X
    global BASE_Y
    x_i = floor((x - BASE_X)/LEN) + 1;
    y_i = floor((y - BASE_Y)/LEN) + 1;
end

function is = occupied(x, y)
    global GRID
    [x_i, y_i] = grid_index(x, y);
    if GRID(y_i, x_i) == 1
        is = 1;
    else
        is = 0;
    end
end

function update_grid(x, y, value)
    global GRID
    global LEN
    global BASE_X
    global BASE_Y
    global NOT_CHANGE_COUNT
    global STOP_COUNT
    [x_i, y_i] = grid_index(x, y);
    if GRID(y_i, x_i) ~= 1 && GRID(y_i, x_i) ~= value
        GRID(y_i, x_i) = value;
        xs = [(x_i-1)*LEN + BASE_X, (x_i-1)*LEN + BASE_X, x_i*LEN + BASE_X, x_i*LEN + BASE_X];
        ys = [(y_i-1)*LEN + BASE_Y, y_i*LEN + BASE_Y, y_i*LEN + BASE_Y, (y_i-1)*LEN + BASE_Y];
        figure(2);
        if value == 1
            patch(xs, ys, [1, 0, 0]);
        else
            patch(xs, ys, [0, 0, 1]);
        end
        axis square;
        NOT_CHANGE_COUNT = 0;
        STOP_COUNT = 0;
    else
        NOT_CHANGE_COUNT = NOT_CHANGE_COUNT + 1;
        STOP_COUNT = STOP_COUNT + 1;
    end
end
