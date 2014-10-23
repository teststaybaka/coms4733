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
        accept_error = 0.05;

        turn_velocity = 0.2;
        forward_velocity = 0.2;
        forward_limit = 4;
        step_limit = 2;
        time_step = 0.1;
        right_search_limit = 30;
        DIST_CORRECTION = 1.1;
    end 
    
    global LEN
    global GRID_X
    global GRID_Y
    LEN = 0.2;
    GRID_X = containers.Map({0},{0});
    GRID_Y = containers.Map({0},{GRID_X});
    draw_grid();
    %define state for the state machine
    WANDERING           = 0;
    W_FORWARDING     	= 1;
    W_TURNING           = 2;
    W_STEP              = 3;

    current_state  = WANDERING;
    next_state = WANDERING;
    
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
    while 1
        pause(time_step);
        
        if ~(abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error)
            left = 1;
        end
                
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
                
        switch current_state
            
            case WANDERING
                
                if bumped
                    
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                        
                    if occupied(X, Y)
                        
                        turn_angle(serPort, turn_velocity, rand()*360);
                        next_state = WANDERING;
                        
                    else
                        
                        m_x = X;
                        m_y = Y;
                        m_a = ANGLE;
                        update_grid(X, Y, 1);
                        bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                        left = 0;
                        next_state = W_FORWARDING;
                        
                    end
                    
                else
                    
					calculate_coord(serPort);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                    update_grid(X, Y, 2);
                    next_state = WANDERING;
                    
                end
                
                fprintf('WANDERING : ');
                print_status(m_x, m_y, m_a);
                
            case W_FORWARDING
                
                if bumped
                    
                    bump_turn(serPort, turn_velocity, BumpRight, BumpLeft, BumpFront);
                    forward_count = 0;
                    update_grid(X, Y, 1);
                    next_state = W_FORWARDING;
                    
                elseif left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error

                    fprintf('start wandering\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X, Y, 1);
                    turn_angle(serPort, turn_velocity, rand()*360);
					next_state = WANDERING;
                    
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
                
                elseif left && abs(m_x - X) < accept_error && abs(m_y - Y) < accept_error
                    
                    fprintf('start wandering\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0);
                    calculate_coord(serPort);
                    update_grid(X, Y, 1);
                    turn_angle(serPort, turn_velocity, rand()*360);
					next_state = WANDERING;
                    
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
	fprintf('%f %f\n', x, y);
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
    while abs(a) < abs(angle/360*2*pi)
        SetFwdVelAngVelCreate(serPort, 0, turn_velocity);
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
    fprintf('(%.4f, %.4f), ANGLE = %.4f, m:(%.4f, %.4f), a = %.4f\n\n', X, Y, ANGLE, m_x, m_y, m_a);
end

function is = occupied(x, y)
    global GRID
    global LEN
    x_i = floor(x/LEN) + 1;
    y_i = floor(y/LEN) + 1;
    if GRID(y_i, x_i) == 1
        is = 1;
    else
        is = 0;
    end
end

function draw_grid()
    global GRID
    global LEN
    [height, width] = size(GRID);
    for y_i = 1:height
      for x_i = 1:width
          x = [(x_i-1)*LEN, (x_i-1)*LEN, x_i*LEN, x_i*LEN];
          y = [(y_i-1)*LEN, y_i*LEN, y_i*LEN, (y_i-1)*LEN];
          figure(2);
          patch(x, y, [1, 1, 1]);
          axis square;
      end
    end
end

function update_grid(x, y, value)
    global GRID
    global LEN
    x_i = floor(x/LEN) + 1;
    y_i = floor(y/LEN) + 1;
    if GRID(y_i, x_i) ~= 1
        GRID(y_i, x_i) = value;
        x = [(x_i-1)*LEN, (x_i-1)*LEN, x_i*LEN, x_i*LEN];
        y = [(y_i-1)*LEN, y_i*LEN, y_i*LEN, (y_i-1)*LEN];
        figure(2);
        if value == 1
            patch(x, y, [1, 0, 0]);
        else
            patch(x, y, [0, 0, 1]);
        end
        axis square;
    end
end
