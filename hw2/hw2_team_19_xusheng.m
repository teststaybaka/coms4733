%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS 4733 - Computational Aspect of Robotics
%
%	Columbia University FALL 2014 
%
%	Team    : 19
%	Members : Xusheng Gao 				        (xg2193)
%		      Yuxuan Xie 				        (yx2284)
%		      Zachary Hideichi Watanabe-Gastel	(zhw2102)
%
%	Homework Assignment 2
%	Using the iRobot Create Matlab Simulator and the iRobot Create
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw2_team_19_xusheng(serPort)
    
    simulator = true;
    
    try
        arg_check = strcmp(serPort.type, 'serial');
        
        if arg_check
            fprintf('Running on Roomba.\n');
            
            simulator = false;
            
            distTravel = 0;
            accept_error = 0.01;

            forward_velocity = 0.05;
            %forward_limit = 3;
            forward_corner_limit = 10;
            time_step = 0.1;
            right_search_limit = 3;
            left_search_limit = 6;
        end
    catch 
        fprintf('Running on simulator.\n');
        distTravel = 0;
        accept_error = 0.05;

        forward_velocity = 0.2;
        %forward_limit = 3;
        forward_corner_limit = 13;
        time_step = 0.1;
        right_search_limit = 3;
        left_search_limit = 6;
    end 
    
    %define state for the state machine
    M_MOVING        = 0;
    N_MOVING       	= 1;
    WALL_FINDING 	= 2;
    CORNER_FORWARD  = 3;
    CORNER_TURN     = 4;
    CORNER_MOVE     = 5;

    current_state  = M_MOVING;
    next_state = M_MOVING;
    
    % INITIAL:
    t_x = 4;
    forward_count = 0;
    right_search_count = 0;
    left_search_count = 0;
    
    [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    hasWall = WallSensorReadRoomba(serPort);
    hasWall = 0;
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    x = 0;
    y = 0;
	m_x = 0;
	m_y = 0;
    angle = 0;
    % INITIAL END;
    
    while abs(x - 4) > accept_error || abs(y) > accept_error
        pause(time_step);
        
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
        
        hasWall = WallSensorReadRoomba(serPort);
        
        fprintf('Wall = %d\n\n', hasWall);
        
        %if bumped, stop the roomba right away
        if bumped
             SetFwdVelAngVelCreate(serPort, 0, 0);
        end
                
        switch current_state
            
            case M_MOVING
                
                if bumped
                    %SetFwdVelAngVelCreate(serPort, -1 * forward_velocity, 0);
                    %pause(time_step);
                    %[x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
					m_x = x;
					m_y = y;
                    next_state = WALL_FINDING;
                else
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					SetFwdVelAngVelCreate(serPort, 2*forward_velocity, 0);
                end
                
                fprintf('M_MOVING : ');
                print_status(x, y, angle, distTravel);
                
            case N_MOVING

                if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
                    next_state = WALL_FINDING;
                else
                    if hasWall
                        SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        next_state = N_MOVING;
                    else
                        SetFwdVelAngVelCreate(serPort, 0, 0);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        forward_count = 0;
                        next_state = CORNER_FORWARD;
                    end 
                    
                    if abs(t_x - x) < abs(t_x - m_x) && abs(y) < accept_error
                        SetFwdVelAngVelCreate(serPort, 0, 0);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.2, 1);
                        turnAngle(serPort, 0.2, -angle/2/pi*360);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        
                        next_state = M_MOVING;
                    end
                    
                end
                
                fprintf('N_MOVING : ');
                print_status(x, y, angle, distTravel);
                
            case WALL_FINDING;
                
                if simulator
                    if hasWall
                        left_search_count = 0;
                        right_search_count = 0;
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);

                        next_state = N_MOVING;

                    elseif right_search_count > right_search_limit

                        if left_search_count > left_search_limit
                            left_search_count = 0;
                            right_search_count = 0;
                            next_state = CORNER_FORWARD;
                        else
                            SetFwdVelAngVelCreate(serPort, 0, 1.0);
                            [x, y, angle] = calculate_coord(serPort, x, y, angle);
                            left_search_count = left_search_count + 1;
                        end

                    else
                        SetFwdVelAngVelCreate(serPort, 0, -1.0);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        right_search_count = right_search_count + 1;
                    end
                else
                    
                    turn_count = 0;
                    turn_limit = 10;
                    
                    %SetFwdVelAngVelCreate(serPort, -2*forward_velocity, 0);
                    
                    while ~hasWall && turn_count < turn_limit
                        pause(time_step);
                        SetFwdVelAngVelCreate(serPort, 0, 0.5);
                        hasWall = WallSensorReadRoomba(serPort);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);

                        if hasWall 
                            SetFwdVelAngVelCreate(serPort, 0, 0);
                            next_state = N_MOVING;
                        end
                    end  
                    
                    if hasWall 
                        SetFwdVelAngVelCreate(serPort, 0, 0);
                        next_state = N_MOVING;
                    end
                end
            
                fprintf('WALL_FINDING : ');
                print_status(x, y, angle, distTravel);
                
			case CORNER_FORWARD 
                
                if forward_count > forward_corner_limit
					forward_count = 0;
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
					next_state = CORNER_TURN;
                elseif hasWall
                    forward_count = 0;
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
                    next_state = N_MOVING;
                else
                    turnAngle(serPort, 0.2, -2);
                    SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
					forward_count = forward_count + 1;
                end
                
                fprintf('CORNER_FORWARD : ');
                print_status(x, y, angle, distTravel);
                
			case CORNER_TURN
                             
                if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
                    next_state = WALL_FINDING;
                else
                    SetFwdVelAngVelCreate(serPort, 0, 0);      
                    turnAngle(serPort, 0.2, -65);
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);

                    next_state = CORNER_MOVE;
                end
                
                fprintf('CORNER_TURN : ');
                print_status(x, y, angle, distTravel);
                
            case CORNER_MOVE
                
                if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
                    next_state = WALL_FINDING;
                elseif forward_count > forward_corner_limit
					forward_count = 0;
					next_state = N_MOVING;
                else
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                    [x, y, angle] = calculate_coord(serPort, x, y, angle);
					forward_count = forward_count + 1;
                end
                
                fprintf('CORNER_MOVE : ');
                print_status(x, y, angle, distTravel);
                
            otherwise
                fprintf('ERR: Should not end up here.');
        end
        
        % update current state with next state
        current_state = next_state;
    end %end of while loop
    
	SetFwdVelAngVelCreate(serPort, 0, 0);
	fprintf('%f %f %d\n', x, y, abs(x - 10) < accept_error);
end

function [x, y, angle] = calculate_coord(serPort, x, y, angle)
    dist = DistanceSensorRoomba(serPort);
    
    angle = angle + AngleSensorRoomba(serPort);
    
    while angle > pi
		angle = angle - 2*pi;
    end
    
	while angle < -pi
		angle = angle + 2*pi;
	end
    
	x = x + dist*cos(angle);
	y = y + dist*sin(angle);
end

function [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle)

	SetFwdVelAngVelCreate(serPort, 0, 0);
    [x, y, angle] = calculate_coord(serPort, x, y, angle);

    if BumpRight
        turnAngle(serPort, 0.2, 20);
    elseif BumpLeft
        turnAngle(serPort, 0.2, 105);
    elseif BumpFront
        turnAngle(serPort, 0.2, 90);
    end
	
    fprintf('%.3f\n', angle);
	[x, y, angle] = calculate_coord(serPort, x, y, angle);
end

function print_status(x, y, angle, distTravel)
    fprintf('(%.4f, %.4f), Angle = %.4f, D = %.4f\n\n', x, y, angle, distTravel);
end