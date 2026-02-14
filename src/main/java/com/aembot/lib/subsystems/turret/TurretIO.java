package com.aembot.lib.subsystems.turret;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;

public interface TurretIO extends Loggable {

  public MotorIO getSingleMotor();

  // public MotorIO[] getFollowerMotor();
}

// public class TurretIO {

// }
