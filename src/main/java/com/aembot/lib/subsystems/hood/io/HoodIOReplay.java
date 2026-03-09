package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.subsystems.hood.HoodInputs;
import com.aembot.lib.tracing.Traced;

/** Base IO layer for the hood subsystem */
public class HoodIOReplay implements HoodIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  @Traced(category = "Hood")
  public void updateInputs(HoodInputs inputs) {}

  @Override
  @Traced(category = "Hood")
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
