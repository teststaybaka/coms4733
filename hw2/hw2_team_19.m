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

function finalRad = hw2_team_19(serPort)

    try
        arg_check = strcmp(serPort.type, 'serial');
        
        if arg_check
            fprintf('Running on Roomba.\n');
            distTravel = 0;
            accept_error = 0.02;

            t_x = 4;
            forward_velocity = 0.1;
            forward_limit = 6;
			forward_corner_limit = 6;
            time_step = 0.1;
            right_search_limit = 20;
            left_search_limit = 28;
        end
    catch 
        fprintf('Running on simulator.\n');
        distTravel = 0;
        accept_error = 0.05;

        t_x = 4;
        forward_velocity = 0.2;
        forward_limit = 4;
		forward_corner_limit = 4;
        time_step = 0.1;
        right_search_limit = 30;
        left_search_limit = 38;
    end 
    
    %define state for the state machine
    M_MOVING         = 0;
    N_MOVING       	 = 1;
    N_WALL_FINDING 	 = 2;
    N_CORNER_FORWARD = 3;
    N_CORNER_TURN    = 4;

    current_state  = M_MOVING;
    next_state = M_MOVING;
    
    % INITIAL:
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
    
    while abs(x - t_x) > accept_error || abs(y) > accept_error
        pause(time_step);
        
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
                
        switch current_state
            
            case M_MOVING
                
                if bumped
                    
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
					m_x = x;
					m_y = y;
                    
                    next_state = N_MOVING;
                else
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
                end
                
                fprintf('M_MOVING : ');
                print_status(x, y, angle, distTravel);
                
            case N_MOVING
                
                if bumped
                    
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    forward_count = 0;
                    
                    next_state = N_MOVING;
                    
				elseif abs(t_x - x) < abs(t_x - m_x) && abs(y) < accept_error

                    if x > t_x
                        SetFwdVelAngVelCreate(serPort, 0, 0);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.1, 10);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.1, -angle/2/pi*360+180);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.1, -angle/2/pi*360+180);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                    else
                        SetFwdVelAngVelCreate(serPort, 0, 0);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.1, 10);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.1, -angle/2/pi*360);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                        turnAngle(serPort, 0.1, -angle/2/pi*360);
                        [x, y, angle] = calculate_coord(serPort, x, y, angle);
                    end

					next_state = M_MOVING;
                    
                elseif forward_count > forward_limit
                    
					SetFwdVelAngVelCreate(serPort, 0, 0);
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
                    forward_count = 0;
					next_state = N_WALL_FINDING;
                    
                else
                    
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    
                end
                
                fprintf('N_MOVING : ');
                print_status(x, y, angle, distTravel);
                
            case N_WALL_FINDING;
                
                hasWall = WallSensorReadRoomba(serPort);
                
                if hasWall
                    
                    left_search_count = 0;
                    right_search_count = 0;
                    
					% a = AngleSensorRoomba(serPort);
                    % turnAngle(serPort, 0.2, -a/2/pi*360);
                    SetFwdVelAngVelCreate(serPort, 0, 0.0);
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					next_state = N_MOVING;
                    
                elseif right_search_count > right_search_limit
                    
                    if left_search_count > left_search_limit
                        
                        left_search_count = 0;
						right_search_count = 0;
						a = AngleSensorRoomba(serPort);
						turnAngle(serPort, 0.2, -a/2/pi*360);
                        SetFwdVelAngVelCreate(serPort, 0, 0.0);
						[x, y, angle] = calculate_coord(serPort, x, y, angle);

						next_state = N_CORNER_FORWARD;
                        
                    else
                        
                        SetFwdVelAngVelCreate(serPort, 0, 0.5);
                        left_search_count = left_search_count + 1;
                        
                    end
                    
                else
                    fprintf('right count:%d %d\n', right_search_count, right_search_limit)
                    SetFwdVelAngVelCreate(serPort, 0, -0.5);
                    right_search_count = right_search_count + 1;
                    
                end
                
                fprintf('N_WALL_FINDING : ');
                print_status(x, y, angle, distTravel);
                
			case N_CORNER_FORWARD
                
				if bumped
                    
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    next_state = N_MOVING;
                    
                end
                
                if forward_count > forward_corner_limit
                    
					forward_count = 0;
					next_state = N_CORNER_TURN;
                    
                else
                    
					SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
					forward_count = forward_count + 1;
                    
                end
                
                fprintf('N_CORNER_FORWARD : ');
                print_status(x, y, angle, distTravel);
                
			case N_CORNER_TURN
                
				if bumped
                    
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    next_state = N_MOVING;
                    
				end
				
				SetFwdVelAngVelCreate(serPort, 0, 0);
				[x, y, angle] = calculate_coord(serPort, x, y, angle);
				turnAngle(serPort, 0.2, -90);
				[x, y, angle] = calculate_coord(serPort, x, y, angle);
                
				next_state = N_MOVING;
                
                fprintf('N_CORNER_TURN : ');
                print_status(x, y, angle, distTravel);
                
            otherwise
                fprintf('ERR: Should not end up here.');
        end
        
        % update current state with next state
        current_state = next_state;
    end %end of while loop
    
	SetFwdVelAngVelCreate(serPort, 0, 0);
	fprintf('%f %f\n', x, y);
end

function [x, y, angle] = calculate_coord(serPort, x, y, angle)
    dist = DistanceSensorRoomba(serPort);
    a = AngleSensorRoomba(serPort);
    angle = angle + a;
	while angle > pi
		angle = angle - 2*pi;
	end
	while angle < -pi
		angle = angle + 2*pi;
	end
	x = x + dist*cos(angle);
	y = y + dist*sin(angle);
    % fprintf('angle sensor:%f\n', a);
end

function [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle)
	fprintf('bumped\n');
	
	SetFwdVelAngVelCreate(serPort, 0, 0);
    [x, y, angle] = calculate_coord(serPort, x, y, angle);
	
    if BumpRight
        turnAngle(serPort, 0.1, 15);
    elseif BumpLeft
        turnAngle(serPort, 0.1, 120);
    elseif BumpFront
        turnAngle(serPort, 0.1, 90);
    end
	
	[x, y, angle] = calculate_coord(serPort, x, y, angle);
end

function print_status(x, y, angle, distTravel)
    fprintf('(%.4f, %.4f), Angle = %.4f, D = %.4f\n\n', x, y, angle, distTravel);
end