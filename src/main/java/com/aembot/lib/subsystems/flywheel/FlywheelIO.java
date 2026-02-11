package com.aembot.lib.subsystems.flywheel;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;

public interface FlywheelIO extends Loggable {

  public MotorIO getLeadMotor();

  public MotorIO[] getFollowerMotors();
}
