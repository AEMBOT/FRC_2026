package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;

public interface HoodIO extends Loggable{

    public MotorIOTalonFX getMotor();

    
}