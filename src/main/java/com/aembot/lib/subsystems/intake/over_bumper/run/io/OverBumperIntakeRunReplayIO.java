package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOReplay;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRunInputs;

public class OverBumperIntakeRunReplayIO implements OverBumperIntakeRunIO {

  @Override
  public MotorIO getMotor() {
    return new MotorIOReplay();
  }

  @Override
  public void updateInputs(OverBumperIntakeRunInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}
