function forward_paces(serPort, forward_velocity, paces)
    global time_step
    for i = 1:paces
        
    SetFwdVelAngVelCreate(serPort, forward_velocity, 0);
        pause(time_step);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
end
