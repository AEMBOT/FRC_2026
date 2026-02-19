package com.aembot.lib.subsystems.flywheel.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.flywheel.FlywheelInputs;

public interface FlywheelIO extends Loggable {
  public void updateInputs(FlywheelInputs inputs);

  public MotorIO getMotor();
}
