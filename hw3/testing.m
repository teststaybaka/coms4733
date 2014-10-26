function testing(serPort)
	turn_velocity = 0.2;
    radius = 0.2;
	SetFwdVelAngVelCreate(serPort, 0, turn_velocity);
    
    v = radius*turn_velocity;
	SetDriveWheelsCreate(serPort, -v, v);
end