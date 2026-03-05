package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.subsystems.hood.HoodInputs;

/** Base IO layer for the hood subsystem */
public class HoodIOReplay implements HoodIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  public void updateInputs(HoodInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
