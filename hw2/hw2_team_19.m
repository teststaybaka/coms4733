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

    %define state for the state machine
    M_MOVING         = 0;
    N_MOVING       	 = 1;
    N_WALL_FINDING 	 = 2;
	N_CORNER_FORWARD = 3;
	N_CORNER_TURN    = 4;
    
    state  = M_MOVING;
    
    distTravel = 0;
    accept_error = 0.02;
    
    forward_velocity = 0.1;
    forward_count = 0;
    forward_limit = 10;
	forward_corner_limit = 10;
    time_step = 0.1;
    right_search_count = 0;
    right_search_limit = 5;
    left_search_count = 0;
    left_search_limit = 10;

    % INITIAL:
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
    
    while abs(x - 10) > accept_error
        pause(time_step);
        
        [BumpRight, BumpLeft, ~ , ~, ~, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = BumpRight || BumpLeft || BumpFront;
                
        switch state
            case M_MOVING
				fprintf('M_MOVING\n');
                if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    state = N_MOVING;
					m_x = x;
					m_y = y;
                else
					SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
                end
            case N_MOVING
				fprintf('N_MOVING\n');
                if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    state = N_MOVING;
                    forward_count = 0;
				elseif x - m_x > 0 && abs(y) < accpet_error
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					turnAngle(serPort, 0.2, -angle/2/pi*360);
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					state = M_MOVING;
                elseif forward_count > forward_limit
					SetFwdVelRadiusRoomba(serPort, 0, inf);
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					forward_count = 0;
					state = N_WALL_FINDING;
				else
					SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
					forward_count = forward_count + 1;
                end
            case N_WALL_FINDING;
				fprintf('N_WALL_FINDING\n');
                hasWall = WallSensorReadRoomba(serPort);
                if hasWall
                    left_search_count = 0;
                    right_search_count = 0;
					a = AngleSensorRoomba(serPort);
                    turnAngle(serPort, 0.2, -a/2/pi*360);
					[x, y, angle] = calculate_coord(serPort, x, y, angle);
					state = N_MOVING;
                elseif right_search_count > right_search_limit
                    if left_search_count > left_search_limit
                        left_search_count = 0;
						right_search_count = 0;
						state = N_CORNER_FORWARD;
                    else
                        SetFwdVelAngVelCreate(serPort, 0, 1.0);
                        left_search_count = left_search_count + 1;
                    end
                else
                    SetFwdVelAngVelCreate(serPort, 0, -1.0);
                    right_search_count = right_search_count + 1;
                end
			case N_CORNER_FORWARD
				fprintf('N_CORNER_FORWARD\n');
				if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    state = N_MOVING;
				end
				
				if forward_count > forward_corner_limit
					forward_count = 0;
					state = N_CORNER_TURN;
				else
					SetFwdVelRadiusRoomba(serPort, forward_velocity, inf);
					forward_count = forward_count + 1;
				end
			case N_CORNER_TURN
				fprintf('N_CORNER_TURN\n');
				if bumped
                    [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle);
                    state = N_MOVING;
				end
				
				[x, y, angle] = calculate_coord(serPort, x, y, angle);
				turnAngle(serPort, 0.2, -90);
				[x, y, angle] = calculate_coord(serPort, x, y, angle);
				state = N_MOVING;
            otherwise
                fprintf('ERR: Should not end up here.');
        end
    end %end of while loop
    
    SetFwdVelRadiusRoomba(serPort, 0.0, inf);
	fprintf('%f %f %d\n', x, y, abs(x - 10) < accept_error);
end

function [x, y, angle] = calculate_coord(serPort, x, y, angle)
	SetFwdVelRadiusRoomba(serPort, 0, inf);
    dist = DistanceSensorRoomba(serPort);
    angle = angle + AngleSensorRoomba(serPort);
	x = x + dist*cos(angle);
	y = y + dist*sin(angle);
end

function [x, y, angle] = bump_turn(serPort, BumpRight, BumpLeft, BumpFront, x, y, angle)
    [x, y, angle] = calculate_coord(serPort, x, y, angle);
	
    if BumpRight
        turnAngle(serPort, 0.2, 30);
    elseif BumpLeft
        turnAngle(serPort, 0.2, 120);
    elseif BumpFront
        turnAngle(serPort, 0.2, 90);
    end
	
	[x, y, angle] = calculate_coord(serPort, x, y, angle);
end

function print_status(x, y, angle, dist, distTravel)
    fprintf('%.3f, %.3f, %.3f, %.3f, %.3f\n', x, y, angle, dist, distTravel);
end