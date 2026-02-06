package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.hood.HoodInputs;

public interface HoodIO extends Loggable {

  public MotorIO getMotor();

  public void updateInputs(HoodInputs inputs);
}
